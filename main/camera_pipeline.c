/*
 * camera_pipeline.c — 摄像头采集 → 硬件 JPEG 编码 → 帧环发布
 *
 * 链路（依据 docs/API_NOTES.md 的源码侦察结论）：
 *   DVP(/dev/video2, PSRAM 缓冲)
 *     capture_task: ioctl(DQBUF) 拿 YUV422 帧并打 t_capture_us（esp_timer，单调）
 *                   →（若编码器输入槽空闲）换序/拷贝进编码器 OUTPUT mmap 缓冲 + 可选 OSD
 *                     → ioctl(QBUF, OUTPUT)
 *     （输入槽不空闲则丢帧：保证时延不累积）
 *   JPEG(/dev/video10, M2M)
 *     encode_task: ioctl(DQBUF, CAPTURE) —— esp_video 对 M2M 设备在 DQBUF 内同步触发
 *                   jpeg_encoder_process（硬件编码，单帧超时 40ms）—— 返回即编码完成，
 *                   打 t_encode_done_us → 发布到帧环（丢帧式，客户端永远取最新帧）
 *
 * 分辨率/质量在线切换（cam_pipe_apply）：
 *   通过事件组先把两个任务停到安全点 → STREAMOFF/关 fd/释放帧环 → 重新 S_SENSOR_FMT/S_FMT/
 *   REQBUFS/mmap → 重启任务。任一步失败都回滚到"停止"状态并返回错误，不死锁不死机。
 */
#include "camera_pipeline.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_cam_sensor.h"
#include "esp_video_device_internal.h"   /* 私有 API：取 sensor 句柄（见 API_NOTES.md 第 4 节） */
#include "bsp/esp32_s31_korvo_1.h"
#include "yuv_osd.h"

static const char *TAG = "cam_pipe";

#define CAM_DEV        ESP_VIDEO_DVP_DEVICE_NAME   /* /dev/video2 */
#define JPEG_DEV       ESP_VIDEO_JPEG_DEVICE_NAME  /* /dev/video10 */
#define CAM_BUFS       3
#define JPEG_IN_BUFS   2
#define JPEG_OUT_BUFS  3
#define DQBUF_TIMEOUT_MS 200

/* 任务停靠协议：BIT0=运行；apply 时清 RUN，两任务停在安全点后置各自 PARK 位，
 * apply 等待两个 PARK 位齐才允许销毁队列/fd（否则会删掉有任务阻塞的队列 → panic） */
#define EV_RUN        (1 << 0)
#define EV_CAP_PARK   (1 << 1)
#define EV_ENC_PARK   (1 << 2)

typedef struct {
    int cam_fd, jpeg_fd;
    esp_cam_sensor_device_t *sensor;
    esp_cam_sensor_format_array_t fmt_array;   /* sensor 支持的全部格式（指向驱动静态表） */
    const esp_cam_sensor_format_t *cur_fmt;    /* 当前 sensor 格式 */

    /* DVP 采集缓冲（mmap） */
    struct {
        uint8_t *start; size_t length;
    } cam_buf[CAM_BUFS];

    /* 编码器 OUTPUT（输入 YUV）与 CAPTURE（输出 JPEG）缓冲 */
    struct {
        uint8_t *start; size_t length;
    } j_in[JPEG_IN_BUFS], j_out[JPEG_OUT_BUFS];
    uint64_t j_in_tcap[JPEG_IN_BUFS];          /* 每个输入槽装帧时刻（算编码耗时用） */
    QueueHandle_t free_in;                     /* 空闲输入槽下标队列 */
    SemaphoreHandle_t enc_trigger;             /* 采集侧 QBUF 后通知编码任务（避免空转触发 M2M） */

    frame_ring_t *ring;
    cam_pipe_info_t info;
    cam_pipe_stats_t stats;
    uint8_t quality;                           /* 1..100，越大越清晰（V4L2_CID_JPEG_COMPRESSION_QUALITY） */
    int fps_limit;                             /* 0=不限 */
    bool overlay;
    EventGroupHandle_t ev;
    TaskHandle_t cap_task_h, enc_task_h;
    SemaphoreHandle_t api_lock;                /* 串行化 apply */
    bool running, inited;
} cam_pipe_t;

static cam_pipe_t s_p;

/* ---------- sensor 格式 → V4L2 fourcc（与 esp_video_dvp_device.c 映射一致） ---------- */
static uint32_t sensor_fmt_to_v4l2(esp_cam_sensor_output_format_t f)
{
    switch (f) {
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_UYVY: return V4L2_PIX_FMT_UYVY;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV: return V4L2_PIX_FMT_YUYV;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565_LE:   return V4L2_PIX_FMT_RGB565;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE:   return V4L2_PIX_FMT_RGB565X;
    default: return 0;
    }
}

/* 该格式能否被硬件 JPEG 编码（UYVY/RGB565 可直喂；YUYV 需在拷贝时换序；见 API_NOTES.md） */
static bool fmt_hw_encodable(esp_cam_sensor_output_format_t f)
{
    return f == ESP_CAM_SENSOR_PIXFORMAT_YUV422_UYVY ||
           f == ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV ||
           f == ESP_CAM_SENSOR_PIXFORMAT_RGB565_LE ||
           f == ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE;
}

static osd_fmt_t fmt_to_osd(uint32_t v4l2_fourcc)
{
    if (v4l2_fourcc == V4L2_PIX_FMT_UYVY) return OSD_FMT_UYVY;
    if (v4l2_fourcc == V4L2_PIX_FMT_YUYV) return OSD_FMT_YUYV;
    return OSD_FMT_RGB565;
}

/* ---------- 拷贝进编码器输入槽：YUYV→UYVY 换序 / RGB565_BE→LE 交换，其余直拷 ---------- */
static void fill_encoder_input(uint8_t *dst, const uint8_t *src, size_t bytes, uint32_t fourcc, uint64_t now_us)
{
    if (fourcc == V4L2_PIX_FMT_YUYV) {
        /* YUYV = Y0 U0 Y1 V0 → 逐 16-bit 字节交换 = UYVY = U0 Y0 V0 Y1（推导见文件头/API_NOTES） */
        const uint16_t *s = (const uint16_t *)src;
        uint16_t *d = (uint16_t *)dst;
        for (size_t i = 0; i < bytes / 2; i++) d[i] = __builtin_bswap16(s[i]);
    } else if (fourcc == V4L2_PIX_FMT_RGB565X) {
        /* 传感器 BE → 编码器要 LE RGB565 */
        const uint16_t *s = (const uint16_t *)src;
        uint16_t *d = (uint16_t *)dst;
        for (size_t i = 0; i < bytes / 2; i++) d[i] = __builtin_bswap16(s[i]);
    } else {
        memcpy(dst, src, bytes);
    }
    if (s_p.overlay) {
        yuv_osd_draw_ms_counter(dst, bytes / (s_p.info.h), s_p.info.w, s_p.info.h,
                                fmt_to_osd(fourcc == V4L2_PIX_FMT_YUYV ? V4L2_PIX_FMT_UYVY : fourcc),
                                now_us);
    }
}

/* ---------- 设备打开/格式设置/缓冲管理 ---------- */
static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

static void teardown_pipeline_locked(void)
{
    if (s_p.cam_fd >= 0) {
        uint32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(s_p.cam_fd, VIDIOC_STREAMOFF, &type);
        close(s_p.cam_fd);
        s_p.cam_fd = -1;
    }
    if (s_p.jpeg_fd >= 0) {
        uint32_t t_out = V4L2_BUF_TYPE_VIDEO_OUTPUT, t_cap = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(s_p.jpeg_fd, VIDIOC_STREAMOFF, &t_out);
        ioctl(s_p.jpeg_fd, VIDIOC_STREAMOFF, &t_cap);
        close(s_p.jpeg_fd);
        s_p.jpeg_fd = -1;
    }
    if (s_p.free_in) { vQueueDelete(s_p.free_in); s_p.free_in = NULL; }
    if (s_p.enc_trigger) { vSemaphoreDelete(s_p.enc_trigger); s_p.enc_trigger = NULL; }
    /* 注意：帧环不销毁——流客户端任务可能仍持有其指针；init 时按最大分辨率一次分配 */
    s_p.running = false;
}

static esp_err_t mmap_camera_buffers(void)
{
    struct v4l2_requestbuffers req = {
        .count = CAM_BUFS, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP,
    };
    ESP_RETURN_ON_ERROR(xioctl(s_p.cam_fd, VIDIOC_REQBUFS, &req) ? ESP_FAIL : ESP_OK, TAG, "cam REQBUFS");
    for (int i = 0; i < CAM_BUFS; i++) {
        struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i };
        ESP_RETURN_ON_ERROR(xioctl(s_p.cam_fd, VIDIOC_QUERYBUF, &buf) ? ESP_FAIL : ESP_OK, TAG, "cam QUERYBUF");
        s_p.cam_buf[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                    s_p.cam_fd, buf.m.offset);
        s_p.cam_buf[i].length = buf.length;
        ESP_RETURN_ON_FALSE(s_p.cam_buf[i].start != MAP_FAILED, ESP_FAIL, TAG, "cam mmap");
        ESP_RETURN_ON_ERROR(xioctl(s_p.cam_fd, VIDIOC_QBUF, &buf) ? ESP_FAIL : ESP_OK, TAG, "cam QBUF");
    }
    uint32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_ERROR(xioctl(s_p.cam_fd, VIDIOC_STREAMON, &type) ? ESP_FAIL : ESP_OK, TAG, "cam STREAMON");
    return ESP_OK;
}

static esp_err_t setup_jpeg_device(uint32_t fourcc_in)
{
    uint32_t w = s_p.info.w, h = s_p.info.h;
    /* 输入侧（OUTPUT）：与摄像头同分辨率；UYVY/RGB565（见 API_NOTES.md 第 3 节格式表） */
    uint32_t in_fmt = (fourcc_in == V4L2_PIX_FMT_YUYV || fourcc_in == V4L2_PIX_FMT_RGB565X)
                      ? ((fourcc_in == V4L2_PIX_FMT_YUYV) ? V4L2_PIX_FMT_UYVY : V4L2_PIX_FMT_RGB565)
                      : fourcc_in;
    struct v4l2_format fo = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
                              .fmt.pix.width = w, .fmt.pix.height = h, .fmt.pix.pixelformat = in_fmt };
    ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_S_FMT, &fo) ? ESP_FAIL : ESP_OK, TAG, "jpeg S_FMT OUTPUT");

    struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                              .fmt.pix.width = w, .fmt.pix.height = h, .fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG };
    ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_S_FMT, &fc) ? ESP_FAIL : ESP_OK, TAG, "jpeg S_FMT CAPTURE");

    /* 质量（1..100，越大越清晰）。
     * 注意：V4L2_CID_JPEG_CHROMA_SUBSAMPLING 在 esp_video 2.2.0 中把 ctrl->value 直接当作
     * jpeg_down_sampling_type_t（fourcc）使用，传 0/1/2/3 菜单索引会触发 HAL 断言；
     * 其默认值即 JPEG_DOWN_SAMPLING_YUV422，故这里不发该控件（见 docs/API_NOTES.md）。 */
    struct v4l2_ext_control ctrl = { .id = V4L2_CID_JPEG_COMPRESSION_QUALITY, .value = s_p.quality };
    struct v4l2_ext_controls ctrls = { .count = 1, .controls = &ctrl };
    ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_S_EXT_CTRLS, &ctrls) ? ESP_FAIL : ESP_OK, TAG, "jpeg quality");

    struct v4l2_requestbuffers rq = { .count = JPEG_IN_BUFS, .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_MMAP };
    ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_REQBUFS, &rq) ? ESP_FAIL : ESP_OK, TAG, "jpeg REQBUFS OUT");
    struct v4l2_requestbuffers rc = { .count = JPEG_OUT_BUFS, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_REQBUFS, &rc) ? ESP_FAIL : ESP_OK, TAG, "jpeg REQBUFS CAP");

    s_p.free_in = xQueueCreate(JPEG_IN_BUFS, sizeof(int));
    s_p.enc_trigger = xSemaphoreCreateCounting(JPEG_IN_BUFS, 0);
    for (int i = 0; i < JPEG_IN_BUFS; i++) {
        struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_MMAP, .index = i };
        ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_QUERYBUF, &buf) ? ESP_FAIL : ESP_OK, TAG, "jpeg QUERYBUF OUT");
        s_p.j_in[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_p.jpeg_fd, buf.m.offset);
        s_p.j_in[i].length = buf.length;
        ESP_RETURN_ON_FALSE(s_p.j_in[i].start != MAP_FAILED, ESP_FAIL, TAG, "jpeg mmap OUT");
        xQueueSend(s_p.free_in, &i, 0);
    }
    for (int i = 0; i < JPEG_OUT_BUFS; i++) {
        struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i };
        ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_QUERYBUF, &buf) ? ESP_FAIL : ESP_OK, TAG, "jpeg QUERYBUF CAP");
        s_p.j_out[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_p.jpeg_fd, buf.m.offset);
        s_p.j_out[i].length = buf.length;
        ESP_RETURN_ON_FALSE(s_p.j_out[i].start != MAP_FAILED, ESP_FAIL, TAG, "jpeg mmap CAP");
        xioctl(s_p.jpeg_fd, VIDIOC_QBUF, &buf);   /* 输出槽全部入队 */
    }
    uint32_t t_out = V4L2_BUF_TYPE_VIDEO_OUTPUT, t_cap = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_STREAMON, &t_out) ? ESP_FAIL : ESP_OK, TAG, "jpeg STREAMON OUT");
    ESP_RETURN_ON_ERROR(xioctl(s_p.jpeg_fd, VIDIOC_STREAMON, &t_cap) ? ESP_FAIL : ESP_OK, TAG, "jpeg STREAMON CAP");
    return ESP_OK;
}

/* 重建链路：按 cur_fmt 设置摄像头与编码器（调用方已持有 api_lock、任务已停靠） */
static esp_err_t build_pipeline_locked(void)
{
    const esp_cam_sensor_format_t *f = s_p.cur_fmt;
    s_p.info.w = f->width; s_p.info.h = f->height;
    s_p.info.fps = f->fps; s_p.info.pclk_hz = f->xclk;   /* 格式表只有 xclk 输入时钟，无 PCLK 字段 */
    strlcpy(s_p.info.fmt_name, f->name, sizeof(s_p.info.fmt_name));
    uint32_t fourcc = sensor_fmt_to_v4l2(f->format);
    esp_cam_sensor_output_format_t of = f->format;
    memcpy(s_p.info.v4l2_fourcc, &fourcc, 4); s_p.info.v4l2_fourcc[4] = 0;

    s_p.cam_fd = open(CAM_DEV, O_RDWR);
    ESP_RETURN_ON_FALSE(s_p.cam_fd >= 0, ESP_FAIL, TAG, "open %s", CAM_DEV);

    /* 1) 切 sensor 格式（分辨率切换的唯一途径，见 API_NOTES.md 第 4 节） */
    esp_cam_sensor_format_t fmt_copy = *f;
    ESP_RETURN_ON_ERROR(xioctl(s_p.cam_fd, VIDIOC_S_SENSOR_FMT, &fmt_copy) ? ESP_FAIL : ESP_OK,
                        TAG, "S_SENSOR_FMT");
    /* 2) 设置 DVP capture 格式并分配缓冲 */
    struct v4l2_format vfmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                .fmt.pix.width = f->width, .fmt.pix.height = f->height,
                                .fmt.pix.pixelformat = fourcc };
    ESP_RETURN_ON_ERROR(xioctl(s_p.cam_fd, VIDIOC_S_FMT, &vfmt) ? ESP_FAIL : ESP_OK, TAG, "cam S_FMT");
    ESP_RETURN_ON_ERROR(mmap_camera_buffers(), TAG, "cam buffers");

    /* 3) 打开编码器并配置 */
    s_p.jpeg_fd = open(JPEG_DEV, O_RDWR);
    ESP_RETURN_ON_FALSE(s_p.jpeg_fd >= 0, ESP_FAIL, TAG, "open %s (检查 CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE)", JPEG_DEV);
    ESP_RETURN_ON_ERROR(setup_jpeg_device(fourcc), TAG, "jpeg setup");

    /* 4) 帧环：init 时已按最大分辨率分配（apply 重建时绝不销毁，见 teardown 注释） */
    ESP_RETURN_ON_FALSE(s_p.ring, ESP_ERR_NO_MEM, TAG, "frame ring missing");
    (void)of;
    s_p.running = true;
    s_p.info.quality = s_p.quality;
    s_p.info.fps_limit = s_p.fps_limit;
    ESP_LOGI(TAG, "pipeline: %s %ux%u %s q%u fps<=%d",
             s_p.info.sensor_name, s_p.info.w, s_p.info.h, s_p.info.fmt_name,
             s_p.quality, s_p.fps_limit);
    return ESP_OK;
}

/* ---------- 任务 ---------- */
static void capture_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (!(xEventGroupGetBits(s_p.ev) & EV_RUN) || !s_p.running) {
            xEventGroupSetBits(s_p.ev, EV_CAP_PARK);   /* 已停在安全点 */
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        xEventGroupClearBits(s_p.ev, EV_CAP_PARK);
        TickType_t period = s_p.fps_limit ? pdMS_TO_TICKS(1000 / s_p.fps_limit) : 0;
        struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
        if (xioctl(s_p.cam_fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGW(TAG, "cam DQBUF err");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        uint64_t t_cap = esp_timer_get_time();   /* ★ P0-3 采集打点 */
        if (buf.flags & V4L2_BUF_FLAG_ERROR) {
            xioctl(s_p.cam_fd, VIDIOC_QBUF, &buf);
            continue;
        }
        s_p.stats.cap_frames++;
        int idx;
        if (xQueueReceive(s_p.free_in, &idx, 0) == pdTRUE) {
            size_t bytes = buf.bytesused ? buf.bytesused : s_p.cam_buf[buf.index].length;
            bytes = MIN(bytes, s_p.j_in[idx].length);
            fill_encoder_input(s_p.j_in[idx].start, s_p.cam_buf[buf.index].start, bytes,
                               sensor_fmt_to_v4l2(s_p.cur_fmt->format), t_cap);
            s_p.j_in_tcap[idx] = t_cap;
            struct v4l2_buffer vb = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_MMAP,
                                      .index = idx, .bytesused = bytes };
            if (xioctl(s_p.jpeg_fd, VIDIOC_QBUF, &vb) != 0) {
                xQueueSend(s_p.free_in, &idx, 0);
                s_p.stats.capture_drops++;
            } else {
                xSemaphoreGive(s_p.enc_trigger);   /* 帧已入编码队列 → 唤醒编码任务 */
            }
        } else {
            s_p.stats.capture_drops++;   /* 编码器忙：丢帧保时延 */
        }
        xioctl(s_p.cam_fd, VIDIOC_QBUF, &buf);   /* 归还采集缓冲 */

        if (period) vTaskDelayUntil(&last_wake, period);   /* 软件 fps 上限 */
        else taskYIELD();
    }
}

static void encode_task(void *arg)
{
    while (1) {
        if (!(xEventGroupGetBits(s_p.ev) & EV_RUN) || !s_p.running) {
            xEventGroupSetBits(s_p.ev, EV_ENC_PARK);   /* 已停在安全点 */
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        xEventGroupClearBits(s_p.ev, EV_ENC_PARK);
        /* 只在有输入帧后触发 M2M（DQBUF 内部同步编码），避免空转触发报错刷屏 */
        if (xSemaphoreTake(s_p.enc_trigger, pdMS_TO_TICKS(200)) != pdTRUE) continue;

        struct v4l2_buffer vb = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
        vb.bytesused = 0;
        if (xioctl(s_p.jpeg_fd, VIDIOC_DQBUF, &vb) != 0) {
            ESP_LOGW(TAG, "jpeg DQBUF CAPTURE err");
            continue;
        }
        uint64_t t_enc = esp_timer_get_time();   /* ★ DQBUF 返回即硬件编码完成 */

        /* 取回被消费的输入槽（拿该帧的 t_capture），并把输入槽还回空闲队列 */
        struct v4l2_buffer ob = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_MMAP };
        uint64_t t_cap = t_enc;
        if (xioctl(s_p.jpeg_fd, VIDIOC_DQBUF, &ob) == 0) {
            if (ob.index < JPEG_IN_BUFS) t_cap = s_p.j_in_tcap[ob.index];
            int idx = ob.index;
            xQueueSend(s_p.free_in, &idx, 0);
        }

        if (vb.flags & V4L2_BUF_FLAG_ERROR || vb.bytesused == 0) {
            ESP_LOGW(TAG, "jpeg encode error frame");
        } else {
            uint64_t t0 = t_cap;
            bool ok = frame_ring_publish(s_p.ring, s_p.j_out[vb.index].start, vb.bytesused,
                                         t_cap, t_enc, s_p.info.w, s_p.info.h, s_p.quality);
            if (ok) {
                s_p.stats.enc_frames++;
                s_p.stats.enc_bytes += vb.bytesused;
                s_p.stats.enc_time_acc_us += (t_enc - t0);
            } else {
                s_p.stats.encode_drops++;
            }
        }
        xioctl(s_p.jpeg_fd, VIDIOC_QBUF, &vb);   /* 输出槽归还 */
    }
}

/* ---------- 公共 API ---------- */
esp_err_t cam_pipe_init(void)
{
    memset(&s_p, 0, sizeof(s_p));
    s_p.cam_fd = s_p.jpeg_fd = -1;
    s_p.quality = 20;
    s_p.ev = xEventGroupCreate();
    s_p.api_lock = xSemaphoreCreateMutex();

    ESP_RETURN_ON_ERROR(bsp_camera_start(NULL), TAG, "bsp_camera_start");
    vTaskDelay(pdMS_TO_TICKS(50));

    s_p.sensor = esp_video_get_dvp_video_device_sensor();
    ESP_RETURN_ON_FALSE(s_p.sensor, ESP_FAIL, TAG, "no sensor detected");
    strlcpy(s_p.info.sensor_name, s_p.sensor->name, sizeof(s_p.info.sensor_name));

    ESP_RETURN_ON_ERROR(esp_cam_sensor_query_format(s_p.sensor, &s_p.fmt_array), TAG, "query formats");

    /* 打印侦测结果（验收要求：日志可见 sensor 型号/分辨率列表/格式/PCLK） */
    ESP_LOGI(TAG, "===== sensor detect: %s（共 %u 种格式） =====", s_p.sensor->name, s_p.fmt_array.count);
    for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
        const esp_cam_sensor_format_t *f = &s_p.fmt_array.format_array[i];
        ESP_LOGI(TAG, "  [%u] %s  %ux%u@%dfps xclk=%d", i, f->name, f->width, f->height, f->fps, f->xclk);
    }
    size_t ps = esp_psram_get_size();
    ESP_LOGI(TAG, "PSRAM: %d kB", (int)(ps / 1024));

    /* 帧环一次性分配：槽容量取 sensor 支持的最大格式（apply 重建链路时不销毁，
     * 避免流客户端任务 use-after-free），槽数 3（最新帧 + 引用中的帧余量） */
    size_t max_cap = 0;
    for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
        const esp_cam_sensor_format_t *f2 = &s_p.fmt_array.format_array[i];
        if (!fmt_hw_encodable(f2->format)) continue;
        size_t cap = (size_t)f2->width * f2->height * 3 / 4 + 8192;
        if (cap > max_cap) max_cap = cap;
    }
    if (max_cap == 0) {
        ESP_LOGE(TAG, "no encodable format for ring sizing");
        return ESP_ERR_NOT_SUPPORTED;
    }
    s_p.ring = frame_ring_create(3, max_cap);
    ESP_RETURN_ON_FALSE(s_p.ring, ESP_ERR_NO_MEM, TAG, "frame ring %ukB x3", (unsigned)(max_cap / 1024));

    /* 默认取当前 sensor 已配置格式（Kconfig DEFAULT_FMT）；找不到则用第一个可编码格式 */
    esp_cam_sensor_format_t cur;
    s_p.cur_fmt = NULL;
    if (esp_cam_sensor_get_format(s_p.sensor, &cur) == ESP_OK) {
        for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
            if (strcmp(s_p.fmt_array.format_array[i].name, cur.name) == 0) {
                s_p.cur_fmt = &s_p.fmt_array.format_array[i];
                break;
            }
        }
    }
    if (!s_p.cur_fmt) {
        for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
            if (fmt_hw_encodable(s_p.fmt_array.format_array[i].format)) {
                s_p.cur_fmt = &s_p.fmt_array.format_array[i];
                break;
            }
        }
    }
    ESP_RETURN_ON_FALSE(s_p.cur_fmt, ESP_ERR_NOT_SUPPORTED, TAG, "no hw-encodable format");
    s_p.inited = true;
    return ESP_OK;
}

void cam_pipe_start(void)
{
    xSemaphoreTake(s_p.api_lock, portMAX_DELAY);
    if (!s_p.ring) {
        esp_err_t err = build_pipeline_locked();
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    }
    if (!s_p.cap_task_h) {
        xTaskCreatePinnedToCore(capture_task, "cam_cap", 6144, NULL, 14, &s_p.cap_task_h, 1);
        xTaskCreatePinnedToCore(encode_task, "cam_enc", 6144, NULL, 12, &s_p.enc_task_h, 1);
        xEventGroupSetBits(s_p.ev, EV_RUN);
    }
    xSemaphoreGive(s_p.api_lock);
}

esp_err_t cam_pipe_apply(int w, int h, uint8_t quality, int fps_limit)
{
    xSemaphoreTake(s_p.api_lock, portMAX_DELAY);
    /* 停任务（清 RUN，等两任务都停在安全点；最长 2 s） */
    xEventGroupClearBits(s_p.ev, EV_RUN);
    xEventGroupWaitBits(s_p.ev, EV_CAP_PARK | EV_ENC_PARK, pdFALSE, pdTRUE,
                        pdMS_TO_TICKS(2000));
    vTaskDelay(pdMS_TO_TICKS(30));   /* 双保险：等退出中的 ioctl 全部返回 */

    const esp_cam_sensor_format_t *want = NULL;
    if (w && h) {
        /* 同分辨率多格式时优先 YUV422（UYVY/YUYV），RGB565 次之（色彩子采样更符合预期） */
        const esp_cam_sensor_format_t *fallback = NULL;
        for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
            const esp_cam_sensor_format_t *f = &s_p.fmt_array.format_array[i];
            if (f->width == w && f->height == h && fmt_hw_encodable(f->format)) {
                if (f->format == ESP_CAM_SENSOR_PIXFORMAT_YUV422_UYVY ||
                    f->format == ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV) { want = f; break; }
                if (!fallback) fallback = f;
            }
        }
        if (!want) want = fallback;
        if (!want) { /* 恢复运行后报不支持 */
            xEventGroupSetBits(s_p.ev, EV_RUN);
            xSemaphoreGive(s_p.api_lock);
            return ESP_ERR_NOT_SUPPORTED;
        }
        s_p.cur_fmt = want;
    }
    if (quality) s_p.quality = quality;
    s_p.fps_limit = fps_limit;
    s_p.info.quality = s_p.quality;

    teardown_pipeline_locked();
    esp_err_t err = build_pipeline_locked();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "apply failed (%s)，链路已停止，可用同参数重试", esp_err_to_name(err));
    }
    xEventGroupSetBits(s_p.ev, EV_RUN);
    xSemaphoreGive(s_p.api_lock);
    return err;
}

frame_ring_t *cam_pipe_ring(void) { return s_p.ring; }
cam_pipe_info_t *cam_pipe_info(void) { return &s_p.info; }
cam_pipe_stats_t *cam_pipe_stats(void) { return &s_p.stats; }
bool cam_pipe_overlay(void) { return s_p.overlay; }
void cam_pipe_set_overlay(bool on) { s_p.overlay = on; }

/* 轻量改质量：只改编码器 ext ctrl，不重建链路（UI 拖动更顺滑） */
esp_err_t cam_pipe_set_quality(uint8_t quality)
{
    if (quality < 1 || quality > 100) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_p.api_lock, portMAX_DELAY);
    s_p.quality = quality;
    s_p.info.quality = quality;
    esp_err_t err = ESP_OK;
    if (s_p.jpeg_fd >= 0) {
        struct v4l2_ext_control ctrl = { .id = V4L2_CID_JPEG_COMPRESSION_QUALITY, .value = quality };
        struct v4l2_ext_controls ctrls = { .count = 1, .controls = &ctrl };
        err = xioctl(s_p.jpeg_fd, VIDIOC_S_EXT_CTRLS, &ctrls) ? ESP_FAIL : ESP_OK;
    }
    xSemaphoreGive(s_p.api_lock);
    return err;
}

void cam_pipe_set_fps_limit(int fps)
{
    s_p.fps_limit = fps;
    s_p.info.fps_limit = fps;
}

bool cam_pipe_res_supported(int w, int h)
{
    for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
        const esp_cam_sensor_format_t *f = &s_p.fmt_array.format_array[i];
        if (f->width == w && f->height == h && fmt_hw_encodable(f->format)) return true;
    }
    return false;
}

int cam_pipe_supported_res(char *out, size_t outlen)
{
    int n = 0;
    out[0] = 0;
    for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
        const esp_cam_sensor_format_t *f = &s_p.fmt_array.format_array[i];
        if (!fmt_hw_encodable(f->format)) continue;
        char item[16];
        snprintf(item, sizeof(item), "%ux%u", f->width, f->height);
        if (strstr(out, item)) continue;
        strlcat(out, item, outlen);
        n++;
        if (i + 1 < s_p.fmt_array.count && n < 12) strlcat(out, ",", outlen);
    }
    return n;
}
