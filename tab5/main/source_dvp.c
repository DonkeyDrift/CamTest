/*
 * source_dvp.c — Tab5 板载 MIPI 相机（SC202CS，/dev/video0 经 ISP 转 UYVY）采集后端
 *
 * 链路：SC202CS(RAW8 1280x720@30, 1-lane) → MIPI CSI → ISP → UYVY
 *   capture_task: DQBUF 打 t_capture_us（传感器输出时刻语义）→ 缩放/裁剪 →
 *                 esp_driver_jpeg 硬件 JPEG（与 source_usb reencode 同一引擎 API，
 *                 两源互斥运行时各自开关）→ 发布帧环
 * S31 的 camera_pipeline（V4L2 M2M JPEG 设备）不适用：Tab5 用 esp_driver_jpeg
 * 直驱（esp_video 2.2.0 的 JPEG 设备语义与 P4 jpeg 驱动能力重复）。
 */
#include "source_if.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "driver/jpeg_encode.h"
#include "board_power.h"
#include "yuv_osd.h"
#include "sdkconfig.h"

#define CAM_DEV        ESP_VIDEO_MIPI_CSI_DEVICE_NAME   /* /dev/video0 */
#define CAM_BUFS       3
#define DQBUF_TIMEOUT_MS 200
#define ENC_TIMEOUT_MS 100

#define EV_RUN        (1 << 0)
#define EV_PARK       (1 << 1)

static const char *TAG = "src_dvp";

struct dvp_priv_s {
    int cam_fd;
    /* 采集缓冲（mmap） */
    struct { uint8_t *start; size_t length; } cam_buf[CAM_BUFS];

    /* JPEG 编码（esp_driver_jpeg 直驱；start 开 / stop 关，与 USB 源互斥） */
    jpeg_encoder_handle_t jenc;
    uint8_t *j_in, *j_out;
    size_t j_in_len, j_out_len;

    /* 原生 → 输出映射 */
    int src_w, src_h;
    uint16_t out_w, out_h;
    uint8_t quality;
    volatile int fps_limit;
    uint64_t next_due_us;
    bool overlay;
    float inherent_ms;

    src_stats_t stats;
    src_info_t info;

    EventGroupHandle_t ev;
    TaskHandle_t task_h;
    SemaphoreHandle_t lock;
    bool inited;
};

static struct dvp_priv_s s_d;

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

static bool plan_virtual(int sw, int sh, int tw, int th, int *kx, int *ky, int *x0, int *y0)
{
    if ((tw & 1) || tw < 16 || th < 16) return false;
    if (tw == sw && th == sh) { *kx=*ky=1; *x0=*y0=0; return true; }
    if (sw % tw == 0 && sh % th == 0) { *kx=sw/tw; *ky=sh/th; *x0=*y0=0; return true; }
    if (tw <= sw && th <= sh) { *kx=*ky=1; *x0=(sw-tw)/2 & ~1; *y0=(sh-th)/2 & ~1; return true; }
    return false;
}

/* UYVY → UYVY：整数抽取/中心裁剪 + 可选 OSD（fill 与 source_usb 同源算法） */
static void fill_jpeg_input(uint8_t *dst, const uint8_t *src, int sw, int sh,
                            int tw, int th, int kx, int ky, int x0, int y0, uint64_t now_us)
{
    const size_t srow = (size_t)sw * 2, drow = (size_t)tw * 2;
    if (tw == sw && th == sh && kx == 1 && ky == 1 && x0 == 0 && y0 == 0) {
        memcpy(dst, src, drow * sh);
    } else {
        for (int ty = 0; ty < th; ty++) {
            const uint8_t *sr = src + (size_t)(y0 + ty * ky) * srow;
            uint8_t *dr = dst + (size_t)ty * drow;
            for (int tx = 0; tx < tw; tx += 2) {
                const uint8_t *sp = sr + ((size_t)(x0 / 2) + (size_t)(tx / 2) * kx) * 4;
                uint8_t *dp = dr + (size_t)tx * 2;
                dp[0]=sp[0]; dp[1]=sp[1]; dp[2]=sp[2]; dp[3]=sp[3];
            }
        }
    }
    if (s_d.overlay) {
        yuv_osd_draw_ms_counter(dst, drow, tw, th, OSD_FMT_UYVY, now_us);
    }
}

static void jpeg_enc_close(void)
{
    if (s_d.jenc) {
        jpeg_del_encoder_engine(s_d.jenc);
        s_d.jenc = NULL;
    }
    if (s_d.j_in) { heap_caps_free(s_d.j_in); s_d.j_in = NULL; }
    if (s_d.j_out) { heap_caps_free(s_d.j_out); s_d.j_out = NULL; }
}

static esp_err_t jpeg_enc_open(void)
{
    jpeg_encode_engine_cfg_t ecfg = { .intr_priority = 0, .timeout_ms = ENC_TIMEOUT_MS };
    esp_err_t err = jpeg_new_encoder_engine(&ecfg, &s_d.jenc);
    ESP_RETURN_ON_ERROR(err, TAG, "jpeg engine");
    jpeg_encode_memory_alloc_cfg_t icfg = { .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER };
    s_d.j_in = jpeg_alloc_encoder_mem((size_t)s_d.out_w * s_d.out_h * 2, &icfg, &s_d.j_in_len);
    jpeg_encode_memory_alloc_cfg_t ocfg = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    s_d.j_out = jpeg_alloc_encoder_mem((size_t)s_d.out_w * s_d.out_h + 65536, &ocfg, &s_d.j_out_len);
    if (!s_d.j_in || !s_d.j_out) {
        ESP_LOGE(TAG, "JPEG 缓冲分配失败");
        jpeg_enc_close();
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void capture_task(void *arg)
{
    for (;;) {
        /* 停靠：apply/stop 清 RUN 后任务在此等待 */
        if (!(xEventGroupGetBits(s_d.ev) & EV_RUN)) {
            xEventGroupSetBits(s_d.ev, EV_PARK);
            EventBits_t bits;
            do {
                vTaskDelay(pdMS_TO_TICKS(20));
                bits = xEventGroupGetBits(s_d.ev);
            } while (!(bits & EV_RUN));
            xEventGroupClearBits(s_d.ev, EV_PARK);
        }
        struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
        if (xioctl(s_d.cam_fd, VIDIOC_DQBUF, &buf) != 0) {
            static uint32_t dq_err;
            if (++dq_err % 60 == 1)
                ESP_LOGW(TAG, "DQBUF 失败 errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        s_d.stats.cap_frames++;
        uint64_t t_cap = esp_timer_get_time();   /* 传感器输出时刻语义（DQBUF 返回即帧就绪） */

        /* 软件限帧 */
        if (s_d.fps_limit > 0) {
            if (s_d.next_due_us == 0) s_d.next_due_us = t_cap;
            if (t_cap < s_d.next_due_us) {
                xioctl(s_d.cam_fd, VIDIOC_QBUF, &buf);
                continue;
            }
            s_d.next_due_us += 1000000ULL / s_d.fps_limit;
            if ((int64_t)(t_cap - s_d.next_due_us) > 500000) s_d.next_due_us = t_cap;
        }

        int kx, ky, x0, y0;
        plan_virtual(s_d.src_w, s_d.src_h, s_d.out_w, s_d.out_h, &kx, &ky, &x0, &y0);
        fill_jpeg_input(s_d.j_in, s_d.cam_buf[buf.index].start,
                        s_d.src_w, s_d.src_h, s_d.out_w, s_d.out_h, kx, ky, x0, y0, t_cap);
        jpeg_encode_cfg_t ec = {
            .width = s_d.out_w,
            .height = s_d.out_h,
            .src_type = JPEG_ENCODE_IN_FORMAT_YUV422,
            .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
            .image_quality = s_d.quality,
            .pixel_reverse = false,
        };
        uint32_t out_size = 0;
        esp_err_t eerr = jpeg_encoder_process(s_d.jenc, &ec, s_d.j_in,
                                              s_d.out_w * s_d.out_h * 2,
                                              s_d.j_out, s_d.j_out_len, &out_size);
        xioctl(s_d.cam_fd, VIDIOC_QBUF, &buf);   /* 采集缓冲立即归还 */
        uint64_t t_enc = esp_timer_get_time();
        if (eerr == ESP_OK && out_size > 0) {
            frame_meta_t meta = { .source = FRAME_SRC_DVP, .scaled = 0,
                                  .ts_meaning = FRAME_TS_SENSOR_OUT,
                                  .codec = FRAME_CODEC_JPEG, .key = 0 };
            bool ok = frame_ring_publish_ex(src_if_ring(), s_d.j_out, out_size,
                                            t_cap, t_enc, s_d.out_w, s_d.out_h,
                                            s_d.quality, &meta);
            if (ok) s_d.stats.out_frames++; else s_d.stats.out_drops++;
            s_d.stats.out_bytes += out_size;
            s_d.stats.proc_acc_us += t_enc - t_cap;
        } else {
            static uint32_t jerr;
            if (++jerr % 30 == 1)
                ESP_LOGW(TAG, "jpeg encode 失败 %s", esp_err_to_name(eerr));
            s_d.stats.out_drops++;
        }
    }
}

/* ---------- 公共 API（source_if vtable） ---------- */
esp_err_t source_dvp_init(void)
{
    memset(&s_d, 0, sizeof(s_d));
    s_d.quality = 30;
    s_d.inherent_ms = -1.0f;
    s_d.lock = xSemaphoreCreateMutex();
    s_d.ev = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_d.lock && s_d.ev, ESP_ERR_NO_MEM, TAG, "sync");

    /* 相机电源（IO 扩展器）+ esp_video CSI 初始化（SC202CS 经 SCCB I2C0 自动侦测） */
    ESP_RETURN_ON_ERROR(board_camera_enable(true), TAG, "cam pwr");
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_master_bus_handle_t i2c = (i2c_master_bus_handle_t)board_i2c_handle();
    esp_video_init_csi_config_t csi = {
        .sccb_config = { .init_sccb = false, .i2c_handle = i2c, .freq = 400000 },
        .reset_pin = -1,
        .pwdn_pin = -1,
    };
    esp_video_init_config_t cam_cfg = { .csi = &csi };
    ESP_RETURN_ON_ERROR(esp_video_init(&cam_cfg), TAG, "esp_video_init");

    /* 常驻采集任务（停靠协议：EV_RUN 控制，stop/apply 时停靠到安全点） */
    if (xTaskCreatePinnedToCore(capture_task, "dvp_cap", 5120, NULL, 12, &s_d.task_h, 1) != pdPASS)
        return ESP_ERR_NO_MEM;

    s_d.inited = true;
    ESP_LOGI(TAG, "DVP 后端就绪（SC202CS MIPI CSI，%s）", CAM_DEV);
    return ESP_OK;
}

static void stop_streaming_locked(void);

static esp_err_t start_streaming_locked(void)
{
    /* 幂等：src_if_switch 同源修复路径不 stop 直接 start——旧流未停则先停，
     * 否则二次 open 撞上 esp_video 的 started 状态（REQBUFS EBUSY） */
    if (s_d.cam_fd >= 0) {
        stop_streaming_locked();
    }
    s_d.cam_fd = open(CAM_DEV, O_RDWR);
    if (s_d.cam_fd < 0) {
        ESP_LOGE(TAG, "open %s 失败 errno=%d", CAM_DEV, errno);
        return ESP_FAIL;
    }
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    if (xioctl(s_d.cam_fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT 失败 errno=%d", errno);
        goto fail;
    }
    s_d.src_w = fmt.fmt.pix.width;
    s_d.src_h = fmt.fmt.pix.height;
    ESP_LOGI(TAG, "sensor 原生格式：MIPI %dx%d fourcc=0x%08lx",
             s_d.src_w, s_d.src_h, (unsigned long)fmt.fmt.pix.pixelformat);
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY) {
        /* ISP 默认输出 RGB565——显式切 UYVY（jpeg 编码器输入格式，S31 同款语义） */
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = s_d.src_w;
        fmt.fmt.pix.height = s_d.src_h;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        if (xioctl(s_d.cam_fd, VIDIOC_S_FMT, &fmt) != 0 ||
            fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY) {
            ESP_LOGE(TAG, "S_FMT UYVY 失败（fourcc=0x%08lx）——色彩将不正确",
                     (unsigned long)fmt.fmt.pix.pixelformat);
        } else {
            ESP_LOGI(TAG, "输出格式已切 UYVY %dx%d", fmt.fmt.pix.width, fmt.fmt.pix.height);
            s_d.src_w = fmt.fmt.pix.width;
            s_d.src_h = fmt.fmt.pix.height;
        }
    }
    /* 输出档位：请求 >0 则用之（须可缩放），否则原生 */
    if (s_d.out_w == 0) {
        s_d.out_w = s_d.src_w;
        s_d.out_h = s_d.src_h;
    }
    int kx, ky, x0, y0;
    if (!plan_virtual(s_d.src_w, s_d.src_h, s_d.out_w, s_d.out_h, &kx, &ky, &x0, &y0)) {
        ESP_LOGE(TAG, "输出 %ux%u 不可由原生 %dx%d 得到", s_d.out_w, s_d.out_h, s_d.src_w, s_d.src_h);
        goto fail;
    }
    if (jpeg_enc_open() != ESP_OK) goto fail;

    struct v4l2_requestbuffers rq = { .count = CAM_BUFS, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                      .memory = V4L2_MEMORY_MMAP };
    if (xioctl(s_d.cam_fd, VIDIOC_REQBUFS, &rq) != 0) {
        /* esp_video 2.2.0 的 stream->started 跨 close/open 持久（close 的 deinit
         * 不复位该标志）——上次 STREAMOFF 若失败，这里 EBUSY。补救：再 STREAMOFF
         * + REQBUFS(0) 释放后重试一次 */
        ESP_LOGW(TAG, "REQBUFS errno=%d，尝试 STREAMOFF+REQBUFS(0) 补救", errno);
        uint32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(s_d.cam_fd, VIDIOC_STREAMOFF, &type);
        struct v4l2_requestbuffers rq0 = { .count = 0, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                           .memory = V4L2_MEMORY_MMAP };
        xioctl(s_d.cam_fd, VIDIOC_REQBUFS, &rq0);
        vTaskDelay(pdMS_TO_TICKS(50));
        rq.count = CAM_BUFS;
        ESP_RETURN_ON_FALSE(xioctl(s_d.cam_fd, VIDIOC_REQBUFS, &rq) == 0,
                            ESP_FAIL, TAG, "REQBUFS 重试仍失败 errno=%d", errno);
    }
    for (int i = 0; i < CAM_BUFS; i++) {
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i };
        if (xioctl(s_d.cam_fd, VIDIOC_QUERYBUF, &b) != 0 ||
            (s_d.cam_buf[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                         s_d.cam_fd, b.m.offset)) == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap buf[%d] 失败 errno=%d", i, errno);
            goto fail;
        }
        s_d.cam_buf[i].length = b.length;
        xioctl(s_d.cam_fd, VIDIOC_QBUF, &b);
    }
    uint32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(s_d.cam_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON 失败 errno=%d", errno);
        goto fail;
    }
    s_d.next_due_us = 0;
    xEventGroupSetBits(s_d.ev, EV_RUN);
    xEventGroupClearBits(s_d.ev, EV_PARK);
    ESP_LOGI(TAG, "DVP 推流开始：%dx%d → JPEG %ux%u q%u", s_d.src_w, s_d.src_h,
             s_d.out_w, s_d.out_h, s_d.quality);
    return ESP_OK;
fail:
    stop_streaming_locked();
    return ESP_FAIL;
}

static void stop_streaming_locked(void)
{
    xEventGroupClearBits(s_d.ev, EV_RUN);
    /* 等任务停靠（DQBUF 超时 200ms 内应到安全点）；未停靠不得 STREAMOFF
     * （在途 DQBUF 会让 esp_video STREAMOFF 失败 → started 标志跨 open 持久） */
    for (int i = 0; i < 50 && !(xEventGroupGetBits(s_d.ev) & EV_PARK); i++)
        vTaskDelay(pdMS_TO_TICKS(20));
    bool parked = xEventGroupGetBits(s_d.ev) & EV_PARK;
    if (s_d.cam_fd >= 0) {
        uint32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(s_d.cam_fd, VIDIOC_STREAMOFF, &type) != 0) {
            ESP_LOGW(TAG, "STREAMOFF 失败 errno=%d（parked=%d）——状态可能残留", errno, parked);
        }
        /* 显式释放驱动缓冲：不清则下次 REQBUFS 可能失败 */
        struct v4l2_requestbuffers rq0 = { .count = 0, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                           .memory = V4L2_MEMORY_MMAP };
        if (parked && xioctl(s_d.cam_fd, VIDIOC_REQBUFS, &rq0) != 0) {
            ESP_LOGW(TAG, "REQBUFS(0) 释放失败 errno=%d", errno);
        }
        close(s_d.cam_fd);
        s_d.cam_fd = -1;
        memset(s_d.cam_buf, 0, sizeof(s_d.cam_buf));
    }
    jpeg_enc_close();
}

esp_err_t source_dvp_start(void)
{
    if (!s_d.inited) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    esp_err_t err = start_streaming_locked();
    xSemaphoreGive(s_d.lock);
    return err;
}

void source_dvp_stop(void)
{
    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    stop_streaming_locked();
    xSemaphoreGive(s_d.lock);
}

esp_err_t source_dvp_apply(int w, int h, uint8_t quality, int fps_limit)
{
    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    bool rebuild = (w && h && (w != s_d.out_w || h != s_d.out_h)) ||
                   (quality && quality != s_d.quality);
    if (quality) s_d.quality = quality;
    if (fps_limit >= 0) s_d.fps_limit = fps_limit;
    esp_err_t err = ESP_OK;
    if (rebuild) {
        bool was_streaming = s_d.cam_fd >= 0;
        stop_streaming_locked();
        if (w && h) { s_d.out_w = w; s_d.out_h = h; }
        if (was_streaming) err = start_streaming_locked();
    }
    xSemaphoreGive(s_d.lock);
    return err;
}

esp_err_t source_dvp_set_quality(uint8_t q)
{
    s_d.quality = q;   /* 每次 process 的 cfg 带 quality，无需 ioctl */
    return ESP_OK;
}

void source_dvp_set_fps_limit(int fps) { s_d.fps_limit = fps > 0 ? fps : 0; }
void source_dvp_set_overlay(bool on) { s_d.overlay = on; }
bool source_dvp_overlay(void) { return s_d.overlay; }

src_info_t *source_dvp_info(void)
{
    static src_info_t info;
    memset(&info, 0, sizeof(info));
    info.source = VIDEO_SOURCE_DVP;
    info.needs_encode = true;
    info.quality = s_d.quality;
    info.ts_meaning = TS_MEANING_SENSOR_OUT;
    info.w = s_d.cam_fd >= 0 ? s_d.out_w : 0;
    info.h = s_d.cam_fd >= 0 ? s_d.out_h : 0;
    info.scaled = (s_d.out_w != s_d.src_w || s_d.out_h != s_d.src_h);
    info.native_w = s_d.src_w;
    info.native_h = s_d.src_h;
    info.nominal_fps = 30;
    strlcpy(info.sensor_name, "SC202CS", sizeof(info.sensor_name));
    strlcpy(info.pix_fmt_str, "UYVY>JPEG", sizeof(info.pix_fmt_str));
    snprintf(info.fmt_name, sizeof(info.fmt_name), "MIPI ISP JPEG %ux%u", info.w, info.h);
    return &info;
}

src_stats_t *source_dvp_stats(void) { return &s_d.stats; }
frame_ring_t *source_dvp_ring(void) { return src_if_ring(); }

int source_dvp_supported_res(char *out, size_t outlen)
{
    /* 原生 1280x720（RAW8 档）+ 裁剪/抽取虚拟档 */
    static const int tiers[][2] = { {1280,720},{800,600},{640,480},{480,320},{320,240},{160,120} };
    int n = 0;
    out[0] = 0;
    for (size_t i = 0; i < sizeof(tiers) / sizeof(tiers[0]); i++) {
        int kx, ky, x0, y0;
        if (!plan_virtual(1280, 720, tiers[i][0], tiers[i][1], &kx, &ky, &x0, &y0)) continue;
        char item[16];
        snprintf(item, sizeof(item), "%dx%d", tiers[i][0], tiers[i][1]);
        if (n) strlcat(out, ",", outlen);
        strlcat(out, item, outlen);
        n++;
    }
    return n;
}

bool source_dvp_res_supported(int w, int h)
{
    int kx, ky, x0, y0;
    return plan_virtual(1280, 720, w, h, &kx, &ky, &x0, &y0);
}

float source_dvp_inherent(void) { return s_d.inherent_ms; }

esp_err_t source_dvp_h264_set_bitrate(uint32_t kbps) { (void)kbps; return ESP_ERR_NOT_SUPPORTED; }
