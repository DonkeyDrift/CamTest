/*
 * source_usb.c — USB UVC 摄像头采集源后端（usb_host_uvc 2.5.2 原生 API，U1/U2/U3/U5）
 *
 * 为何不走 esp_video 的 /dev/video40 V4L2 封装（源码侦察结论，详见 README）：
 *   esp_video 2.2.0 的 uvc_video_deinit()/uvc_video_stop() 在设备断线后（dev_addr==0）
 *   提前返回 ESP_ERR_NOT_FOUND → esp_video_close() 不清 inited 标志、uvc_host_stream_close()
 *   永不被调用 → 每次拔插泄漏一条 stream（URB 为内部 RAM）；open 时还会同步阻塞等待枚举
 *   （默认 10 s）。不满足「拔插 3 次不卡死 + 自动重连」的验收要求，故按任务书预留的
 *   降级路线直接使用 usb_host_uvc 原生 API（其对死设备容错：stream_close 的 stop 错误
 *   被忽略，帧全部归还后 uvc_device_remove 释放全部资源）。
 *
 * 数据通路（与 DVP 侧语义对齐，度量体系共用）：
 *   frame_cb（uvc 驱动任务）    打 t_arrival = esp_timer_get_time() —— U4 的
 *                              「完整一帧到达 ESP32 时刻」（含摄像头内部曝光/ISP/
 *                              编码/USB 传输的不可见延迟），入队，队列满即丢帧
 *   usb_work（core1）          passthrough：发布帧环（t_cap=t_enc=t_arrival，quality=0）
 *                              reencode：YUY2→UYVY/缩放/OSD → /dev/video10 硬件编码
 *                              （M2M，DQBUF 内同步，与 DVP 同款编码器语义）→ 发布
 *   流客户端                    从帧环自取（与 DVP 完全一致，不感知源）
 *
 * 热插拔（U5）：
 *   - 自建 usb_host client 收 NEW_DEV/DEV_GONE（广播给所有 client）；开机用
 *     addr_list 补扫已插入设备（client 注册不补发已连接设备的事件）
 *   - 断线：掉线计数 +1 → worker 排空并归还全部帧 → uvc_host_stream_close（死设备容错）
 *   - 重新插入：重新枚举档位 → 若 USB 为当前源则自动重新打开推流
 *   - 打开/协商失败进入 ERROR 态低频重试（典型诱因：TPS2051C 500 mA 供电不足）
 */
#include "source_if.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/mman.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"    /* VIDIOC_S_DQBUF_TIMEOUT（esp_video 自定义 ioctl） */
#include "camera_pipeline.h"     /* cam_pipe_ring()：共享帧环；编码器设备号约定 */
#include "yuv_osd.h"
#include "sdkconfig.h"

#if CONFIG_CAMTEST_ENABLE_USB

static const char *TAG = "src_usb";

#define USB_DEV_ANY          0
#define FRAME_BUFFERS        4      /* uvc 驱动帧缓冲数（按协商出的 dwMaxVideoFrameSize 自分配 PSRAM） */
#define NUM_URBS             4
#define URB_SIZE             (10 * 1024)
#define WORK_QUEUE_LEN       4
#define STREAM_OPEN_TIMEOUT_MS 5000
#define RETRY_BACKOFF_MS     1000   /* ERROR 态重试间隔（供电不足场景别疯狂重试） */
#define ENC_IN_BUFS          2
#define ENC_OUT_BUFS         3
#define ENC_DQBUF_TIMEOUT_MS 200

/* ---------- 私有状态（usb_state_t 已被 source_if.h 的对外状态枚举占用） ---------- */
struct usb_priv_s {
    /* USB 总线层 */
    usb_host_client_handle_t client;
    volatile bool dev_present;
    uint8_t  dev_addr;
    uint8_t  stream_index;
    uint16_t vid, pid;
    char     dev_name[SRC_USB_NAME_MAX];        /* "vid:pid" */

    /* 描述符枚举结果（U3） */
    uvc_host_frame_info_t frame_info[USB_TIER_MAX * 4];
    size_t  frame_info_n;
    usb_tier_t tiers[USB_TIER_MAX];
    int     tier_n;
    bool    mjpeg_ok, yuy2_ok;

    /* 运行配置 */
    usb_mode_t mode;                            /* passthrough / reencode */
    uint16_t req_w, req_h;                      /* 请求的输出分辨率（0=默认档） */
    uint16_t w, h;                              /* 当前协商的摄像头原生分辨率 */
    uint16_t out_w, out_h;                      /* 输出（重编码缩放后）分辨率 */
    uint8_t  quality;
    volatile int fps_limit;
    uint64_t next_due_us;                       /* 软件限帧 */

    /* uvc 流 */
    volatile uvc_host_stream_hdl_t stream;
    volatile bool stream_up;

    /* 编码器（仅 reencode；/dev/video10 与 DVP 同一台，两源互斥运行） */
    int enc_fd;
    struct { uint8_t *start; size_t length; } enc_in[ENC_IN_BUFS], enc_out[ENC_OUT_BUFS];
    QueueHandle_t enc_free_in;

    /* 同步 */
    QueueHandle_t work_q;                       /* uvc_host_frame_t* */
    SemaphoreHandle_t lock;                     /* 状态机互斥 */
    volatile bool worker_run;                   /* worker 消费循环使能 */
    volatile bool worker_busy;                  /* worker 正在处理一帧（teardown 握手） */
    volatile bool want_stream;                  /* USB 为当前源且期望推流 */
    volatile bool teardown_req;                 /* 断线/停止清理请求（monitor 执行） */
    int64_t  next_retry_us;

    /* 统计 */
    src_stats_t stats;
    uint32_t disconnects;
    uint32_t open_errors;
    uint32_t overflow_events;

    /* 叠加与标定 */
    bool overlay;
    float inherent_ms;                          /* 摄像头内部固有延迟估算；<0 = 未标定 */

    bool inited;
};

static struct usb_priv_s s_u;

/* ---------- 小工具（与 camera_pipeline.c 同源算法的 USB 侧拷贝，避免改动 DVP 文件） ---------- */
static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

/* 原生(sw,sh)→目标(tw,th)：相等 / 双轴整数抽取 / 中心裁剪；YUV422 要求目标宽为偶数 */
static bool plan_virtual(int sw, int sh, int tw, int th, int *kx, int *ky, int *x0, int *y0)
{
    if ((tw & 1) || tw < 16 || th < 16) return false;
    if (tw == sw && th == sh) { *kx=*ky=1; *x0=*y0=0; return true; }
    if (sw % tw == 0 && sh % th == 0) { *kx=sw/tw; *ky=sh/th; *x0=*y0=0; return true; }
    if (tw <= sw && th <= sh) { *kx=*ky=1; *x0=(sw-tw)/2 & ~1; *y0=(sh-th)/2; return true; }
    return false;
}

/* YUY2 源 → 编码器输入（UYVY）：整数抽取/中心裁剪 + 字节序交换 + 可选 OSD */
static void fill_encoder_input(uint8_t *dst, const uint8_t *src, int sw, int sh,
                               int tw, int th, int kx, int ky, int x0, int y0, uint64_t now_us)
{
    const size_t srow = (size_t)sw * 2, drow = (size_t)tw * 2;
    const bool passthrough = (tw == sw && th == sh && kx == 1 && ky == 1 && x0 == 0 && y0 == 0);
    if (passthrough) {
        const uint16_t *sp = (const uint16_t *)src;
        uint16_t *dp = (uint16_t *)dst;
        for (size_t i = 0; i < drow * sh / 2; i++) dp[i] = __builtin_bswap16(sp[i]);
    } else {
        for (int ty = 0; ty < th; ty++) {
            const uint8_t *sr = src + (size_t)(y0 + ty * ky) * srow;
            uint8_t *dr = dst + (size_t)ty * drow;
            for (int tx = 0; tx < tw; tx += 2) {
                const uint8_t *sp = sr + ((size_t)(x0 / 2) + (size_t)(tx / 2) * kx) * 4;
                uint8_t *dp = dr + (size_t)tx * 2;
                dp[0]=sp[1]; dp[1]=sp[0]; dp[2]=sp[3]; dp[3]=sp[2];
            }
        }
    }
    if (s_u.overlay) {
        yuv_osd_draw_ms_counter(dst, drow, tw, th, OSD_FMT_UYVY, now_us);
    }
}

static const char *fmt_enum_name(enum uvc_host_stream_format f)
{
    switch (f) {
    case UVC_VS_FORMAT_MJPEG: return "MJPEG";
    case UVC_VS_FORMAT_YUY2:  return "YUY2";
    case UVC_VS_FORMAT_H264:  return "H264";
    case UVC_VS_FORMAT_H265:  return "H265";
    case UVC_VS_FORMAT_NV12:  return "NV12";
    default: return "?";
    }
}

/* ---------- 描述符枚举（U3）：打印完整 (格式,分辨率,帧率) 列表并生成档位表 ---------- */
static void rebuild_tiers(void)
{
    s_u.tier_n = 0;
    s_u.mjpeg_ok = s_u.yuy2_ok = false;
    memset(s_u.tiers, 0, sizeof(s_u.tiers));

    ESP_LOGI(TAG, "===== UVC 支持的 (格式, 分辨率, 帧率) 完整列表 =====");
    for (size_t i = 0; i < s_u.frame_info_n; i++) {
        uvc_host_frame_info_t *fi = &s_u.frame_info[i];
        ESP_LOGI(TAG, "  [%u] %-5s %ux%u default=%ufps interval_type=%u",
                 (unsigned)i, fmt_enum_name(fi->format), fi->h_res, fi->v_res,
                 (unsigned)(10000000ULL / (fi->default_interval ? fi->default_interval : 1)),
                 fi->interval_type);
        if (fi->interval_type == 0) {
            ESP_LOGI(TAG, "      连续区间 min=%u max=%u step=%u",
                     fi->interval_min, fi->interval_max, fi->interval_step);
        } else {
            for (int j = 0; j < fi->interval_type && j < USB_TIER_FPS_MAX; j++) {
                ESP_LOGI(TAG, "      interval[%d]=%u → %ufps", j,
                         fi->interval[j], (unsigned)(10000000ULL / fi->interval[j]));
            }
        }

        if (fi->format != UVC_VS_FORMAT_MJPEG && fi->format != UVC_VS_FORMAT_YUY2) continue;
        if (fi->format == UVC_VS_FORMAT_MJPEG) s_u.mjpeg_ok = true;
        else s_u.yuy2_ok = true;

        /* 合并同 (格式,分辨率) 档位，收集帧率列表 */
        usb_tier_t *t = NULL;
        for (int k = 0; k < s_u.tier_n; k++) {
            if (s_u.tiers[k].w == fi->h_res && s_u.tiers[k].h == fi->v_res &&
                ((fi->format == UVC_VS_FORMAT_MJPEG) == s_u.tiers[k].mjpeg)) { t = &s_u.tiers[k]; break; }
        }
        if (!t && s_u.tier_n < USB_TIER_MAX) {
            t = &s_u.tiers[s_u.tier_n++];
            strlcpy(t->fmt, fmt_enum_name(fi->format), sizeof(t->fmt));
            t->w = fi->h_res; t->h = fi->v_res;
            t->mjpeg = (fi->format == UVC_VS_FORMAT_MJPEG);
        }
        if (!t) continue;

        int def_fps = (int)(10000000ULL / (fi->default_interval ? fi->default_interval : 1));
        if (fi->interval_type == 0) {
            if (def_fps > 0 && t->fps_n < USB_TIER_FPS_MAX) t->fps[t->fps_n++] = def_fps;
        } else {
            for (int j = 0; j < fi->interval_type && t->fps_n < USB_TIER_FPS_MAX; j++) {
                int f = (int)(10000000ULL / fi->interval[j]);
                bool dup = false;
                for (int q = 0; q < t->fps_n; q++) dup = dup || t->fps[q] == f;
                if (!dup && f > 0) t->fps[t->fps_n++] = f;
            }
        }
        if (t->fps_n == 0 && def_fps > 0) t->fps[t->fps_n++] = def_fps;
    }
    ESP_LOGI(TAG, "===== 档位汇总：MJPEG=%s YUY2=%s，共 %d 档；默认模式=%s =====",
             s_u.mjpeg_ok ? "支持" : "无", s_u.yuy2_ok ? "支持" : "无", s_u.tier_n,
             s_u.mode == USB_MODE_PASSTHROUGH ? "passthrough" : "reencode");
}

/* ---------- 设备到达/离开 ---------- */
static void probe_device(uint8_t dev_addr)
{
    if (s_u.dev_present && s_u.dev_addr == dev_addr) return;   /* 去重：client 事件 + 驱动事件双路径 */

    /* 取 VID/PID（usb 1.5.0 无字符串描述符公共 API，iProduct 见 README 待核实清单） */
    usb_device_handle_t dev = NULL;
    char name[SRC_USB_NAME_MAX] = "";
    if (usb_host_device_open(s_u.client, dev_addr, &dev) == ESP_OK) {
        const usb_device_desc_t *desc = NULL;
        if (usb_host_get_device_descriptor(dev, &desc) == ESP_OK && desc) {
            s_u.vid = desc->idVendor;
            s_u.pid = desc->idProduct;
            snprintf(name, sizeof(name), "%04x:%04x", s_u.vid, s_u.pid);
        }
        usb_host_device_close(s_u.client, dev);
    }
    if (!name[0]) strlcpy(name, "unknown", sizeof(name));

    /* 探测 UVC 功能与流索引，列出档位 */
    size_t need = 0;
    uint8_t found_idx = 0;
    bool is_uvc = false;
    for (uint8_t idx = 0; idx < 6; idx++) {
        need = 0;
        if (uvc_host_get_frame_list(dev_addr, idx, NULL, &need) == ESP_OK && need > 0) {
            is_uvc = true;
            found_idx = idx;
            break;
        }
    }
    if (!is_uvc) {
        ESP_LOGI(TAG, "USB 设备 %s (addr %u) 不是 UVC 摄像头，忽略", name, dev_addr);
        return;
    }

    xSemaphoreTake(s_u.lock, portMAX_DELAY);
    s_u.dev_present = true;
    s_u.dev_addr = dev_addr;
    s_u.stream_index = found_idx;
    strlcpy(s_u.dev_name, name, sizeof(s_u.dev_name));
    size_t cap = sizeof(s_u.frame_info) / sizeof(s_u.frame_info[0]);
    s_u.frame_info_n = 0;
    if (uvc_host_get_frame_list(dev_addr, found_idx, (uvc_host_frame_info_t (*)[])s_u.frame_info, &cap) == ESP_OK) {
        s_u.frame_info_n = cap;
    }
    xSemaphoreGive(s_u.lock);

    ESP_LOGI(TAG, "===== UVC 设备已连接：name=%s VID/PID=%04x/%04x addr=%u stream_idx=%u =====",
             name, s_u.vid, s_u.pid, dev_addr, found_idx);
    rebuild_tiers();
    /* monitor 任务按 want_stream 自动开始推流 */
}

static void on_device_gone(void)
{
    bool was_streaming = s_u.stream_up;
    xSemaphoreTake(s_u.lock, portMAX_DELAY);
    bool had_dev = s_u.dev_present;
    s_u.dev_present = false;
    s_u.dev_addr = USB_DEV_ANY;
    s_u.dev_name[0] = 0;
    s_u.frame_info_n = 0;
    s_u.tier_n = 0;
    s_u.mjpeg_ok = s_u.yuy2_ok = false;
    xSemaphoreGive(s_u.lock);
    if (had_dev) {
        s_u.disconnects++;
        ESP_LOGW(TAG, "UVC 设备已断开（累计 %u 次）%s", (unsigned)s_u.disconnects,
                 was_streaming ? "，清理中，等待重新插入…" : "");
    }
    if (was_streaming) s_u.teardown_req = true;
}

/* ---------- 事件回调 ---------- */
static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        probe_device(msg->new_dev.address);
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        on_device_gone();
        break;
    default:
        break;
    }
}

static void drv_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "UVC 驱动事件：设备已连接 addr=%u stream_idx=%u（枚举由 client 事件路径完成）",
                 event->device_connected.dev_addr, event->device_connected.uvc_stream_index);
    }
}

static void stream_event_cb(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "UVC 流事件：设备断开");
        s_u.stream_up = false;
        s_u.teardown_req = true;    /* monitor 任务负责关闭与重连 */
        break;
    case UVC_HOST_TRANSFER_ERROR:
        s_u.open_errors++;          /* 传输错误计数（供电不足时显著增长） */
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        s_u.overflow_events++;
        s_u.stats.cap_drops++;
        break;
    default:
        break;
    }
}

static bool frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    /* ★ U4 打点：完整一帧到达 ESP32 的时刻（含摄像头内部曝光/ISP/编码/USB 传输延迟） */
    s_u.stats.cap_frames++;
    uvc_host_stream_hdl_t h = s_u.stream;
    if (!h) return false;   /* 收尾中的竞态：帧随 stream_close 一起释放 */
    if (xQueueSend(s_u.work_q, &frame, 0) != pdPASS) {
        s_u.stats.cap_drops++;      /* 队列满：丢帧保时延 */
        uvc_host_frame_return(h, (uvc_host_frame_t *)frame);
    }
    return false;   /* 帧由 worker 用完显式归还 */
}

/* ---------- 编码器（仅 reencode 模式；与 DVP 的 /dev/video10 用法一致） ---------- */
static void encoder_close(void)
{
    if (s_u.enc_fd >= 0) {
        uint32_t t_out = V4L2_BUF_TYPE_VIDEO_OUTPUT, t_cap = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(s_u.enc_fd, VIDIOC_STREAMOFF, &t_out);
        ioctl(s_u.enc_fd, VIDIOC_STREAMOFF, &t_cap);
        close(s_u.enc_fd);
        s_u.enc_fd = -1;
    }
}

static esp_err_t encoder_open(int w, int h, uint8_t quality)
{
    s_u.enc_fd = open(ESP_VIDEO_JPEG_DEVICE_NAME, O_RDWR);   /* /dev/video10 */
    ESP_RETURN_ON_FALSE(s_u.enc_fd >= 0, ESP_FAIL, TAG, "open %s", ESP_VIDEO_JPEG_DEVICE_NAME);
    struct timeval tv = { .tv_sec = 0, .tv_usec = ENC_DQBUF_TIMEOUT_MS * 1000 };
    xioctl(s_u.enc_fd, VIDIOC_S_DQBUF_TIMEOUT, &tv);

    struct v4l2_format fo = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
                              .fmt.pix.width = w, .fmt.pix.height = h,
                              .fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY };
    ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_S_FMT, &fo) ? ESP_FAIL : ESP_OK, TAG, "enc S_FMT OUT");
    struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                              .fmt.pix.width = w, .fmt.pix.height = h,
                              .fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG };
    ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_S_FMT, &fc) ? ESP_FAIL : ESP_OK, TAG, "enc S_FMT CAP");
    /* 不发 CHROMA_SUBSAMPLING 控件（esp_video 2.2.0 HAL 断言坑，见 API_NOTES.md） */
    struct v4l2_ext_control ctrl = { .id = V4L2_CID_JPEG_COMPRESSION_QUALITY, .value = quality };
    struct v4l2_ext_controls ctrls = { .count = 1, .controls = &ctrl };
    ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_S_EXT_CTRLS, &ctrls) ? ESP_FAIL : ESP_OK, TAG, "enc quality");

    struct v4l2_requestbuffers rq = { .count = ENC_IN_BUFS, .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_MMAP };
    ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_REQBUFS, &rq) ? ESP_FAIL : ESP_OK, TAG, "enc REQBUFS OUT");
    struct v4l2_requestbuffers rc = { .count = ENC_OUT_BUFS, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_REQBUFS, &rc) ? ESP_FAIL : ESP_OK, TAG, "enc REQBUFS CAP");

    xQueueReset(s_u.enc_free_in);
    for (int i = 0; i < ENC_IN_BUFS; i++) {
        struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_MMAP, .index = i };
        ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_QUERYBUF, &buf) ? ESP_FAIL : ESP_OK, TAG, "enc QUERYBUF OUT");
        s_u.enc_in[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_u.enc_fd, buf.m.offset);
        s_u.enc_in[i].length = buf.length;
        ESP_RETURN_ON_FALSE(s_u.enc_in[i].start != MAP_FAILED, ESP_FAIL, TAG, "enc mmap OUT");
        xQueueSend(s_u.enc_free_in, &i, 0);
    }
    for (int i = 0; i < ENC_OUT_BUFS; i++) {
        struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i };
        ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_QUERYBUF, &buf) ? ESP_FAIL : ESP_OK, TAG, "enc QUERYBUF CAP");
        s_u.enc_out[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_u.enc_fd, buf.m.offset);
        s_u.enc_out[i].length = buf.length;
        ESP_RETURN_ON_FALSE(s_u.enc_out[i].start != MAP_FAILED, ESP_FAIL, TAG, "enc mmap CAP");
        xioctl(s_u.enc_fd, VIDIOC_QBUF, &buf);
    }
    uint32_t t_out = V4L2_BUF_TYPE_VIDEO_OUTPUT, t_cap = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_STREAMON, &t_out) ? ESP_FAIL : ESP_OK, TAG, "enc STREAMON OUT");
    ESP_RETURN_ON_ERROR(xioctl(s_u.enc_fd, VIDIOC_STREAMON, &t_cap) ? ESP_FAIL : ESP_OK, TAG, "enc STREAMON CAP");
    return ESP_OK;
}

/* ---------- 档位选择：返回 frame_info 下标；-1 = 不支持 ---------- */
static int pick_native(bool mjpeg_only, int want_w, int want_h)
{
    int fallback = -1;      /* 未指定分辨率时的默认档 */
    int scalable = -1;      /* 重编码可缩放达标的档 */
    for (size_t i = 0; i < s_u.frame_info_n; i++) {
        uvc_host_frame_info_t *fi = &s_u.frame_info[i];
        if (mjpeg_only && fi->format != UVC_VS_FORMAT_MJPEG) continue;
        if (!mjpeg_only && fi->format != UVC_VS_FORMAT_YUY2) continue;
        if (want_w <= 0 || want_h <= 0) {
            if (fallback < 0) fallback = i;
            continue;
        }
        if (fi->h_res == want_w && fi->v_res == want_h) return i;   /* 原生精确优先 */
        int kx, ky, x0, y0;
        /* 直通不解码 → 必须原生精确；重编码允许缩放到不大于原生的目标 */
        if (!mjpeg_only && scalable < 0 &&
            plan_virtual(fi->h_res, fi->v_res, want_w, want_h, &kx, &ky, &x0, &y0)) {
            scalable = i;
        }
    }
    if (fallback >= 0 && (want_w <= 0 || want_h <= 0)) return fallback;
    return scalable;
}

/* ---------- 清理：顺序敏感（先暂停流 → 归还全部帧 → 关流 → 关编码器） ---------- */
static void teardown_stream(void)
{
    s_u.worker_run = false;
    if (s_u.stream) {
        uvc_host_stream_hdl_t h = s_u.stream;
        /* 先暂停（无新帧、无回调）。活设备走控制传输；死设备报错——两种情况都忽略返回值，
         * 断线场景 uvc_host 的 DEV_GONE 处理里已经 pause 过 */
        uvc_host_stream_stop(h);
        /* 排空队列，把所有帧归还驱动（stream_close 要求全部归还） */
        uvc_host_frame_t *f;
        while (xQueueReceive(s_u.work_q, &f, 0) == pdTRUE) {
            uvc_host_frame_return(h, f);
        }
        /* 等 worker 处理完手中帧（含编码，最长约 50 ms；编码 DQBUF 有 200 ms 超时兜底） */
        int wait = 0;
        while (s_u.worker_busy && wait++ < 100) vTaskDelay(pdMS_TO_TICKS(10));

        s_u.stream = NULL;
        s_u.stream_up = false;
        for (int i = 0; i < 5; i++) {
            if (uvc_host_stream_close(h) == ESP_OK) break;
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    } else {
        s_u.stream_up = false;
    }
    encoder_close();   /* 必须在 worker 完全停手之后（编码器只被 worker 访问） */
}

static esp_err_t open_stream_locked(void)
{
    if (!s_u.dev_present) return ESP_ERR_NOT_FOUND;

    bool passthrough = (s_u.mode == USB_MODE_PASSTHROUGH);
    int idx = pick_native(passthrough, s_u.req_w, s_u.req_h);
    if (idx < 0) {
        ESP_LOGE(TAG, "%s 模式下不支持 %dx%d（见档位列表）",
                 passthrough ? "passthrough" : "reencode", s_u.req_w, s_u.req_h);
        return ESP_ERR_NOT_SUPPORTED;
    }
    uvc_host_frame_info_t *fi = &s_u.frame_info[idx];

    /* 输出分辨率：直通=原生；重编码=请求值（默认原生），由 plan_virtual 保证可行 */
    int out_w = passthrough ? fi->h_res : (s_u.req_w ? s_u.req_w : fi->h_res);
    int out_h = passthrough ? fi->v_res : (s_u.req_h ? s_u.req_h : fi->v_res);

    uvc_host_stream_config_t cfg = {
        .event_cb = stream_event_cb,
        .frame_cb = frame_cb,
        .user_ctx = NULL,
        .usb = { .dev_addr = s_u.dev_addr, .vid = UVC_HOST_ANY_VID, .pid = UVC_HOST_ANY_PID,
                 .uvc_stream_index = s_u.stream_index },
        .vs_format = { .h_res = fi->h_res, .v_res = fi->v_res,
                       .fps = 0,   /* 0 = 摄像头默认帧间隔 */
                       .format = fi->format },
        .advanced = {
            .number_of_frame_buffers = FRAME_BUFFERS,
            .frame_size = 0,        /* 0 = 用协商出的 dwMaxVideoFrameSize */
            .frame_heap_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
            .number_of_urbs = NUM_URBS,
            .urb_size = URB_SIZE,
            .user_frame_buffers = NULL,   /* 驱动自分配（重编码的大帧也放得下） */
        },
    };
    uvc_host_stream_hdl_t h = NULL;
    esp_err_t err = uvc_host_stream_open(&cfg, pdMS_TO_TICKS(STREAM_OPEN_TIMEOUT_MS), &h);
    ESP_RETURN_ON_ERROR(err, TAG, "stream_open (%s %ux%u)", fmt_enum_name(fi->format), fi->h_res, fi->v_res);
    s_u.stream = h;

    err = uvc_host_stream_start(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "stream_start failed (%s)", esp_err_to_name(err));
        uvc_host_stream_close(h);
        s_u.stream = NULL;
        return err;
    }

    if (!passthrough) {
        err = encoder_open(out_w, out_h, s_u.quality);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "encoder_open failed");
            uvc_host_stream_stop(h);
            uvc_host_frame_t *f;
            while (xQueueReceive(s_u.work_q, &f, 0) == pdTRUE) uvc_host_frame_return(h, f);
            uvc_host_stream_close(h);
            s_u.stream = NULL;
            return err;
        }
    }

    s_u.w = fi->h_res;
    s_u.h = fi->v_res;
    s_u.out_w = out_w;
    s_u.out_h = out_h;
    s_u.next_due_us = 0;
    s_u.stream_up = true;
    s_u.worker_run = true;
    xQueueReset(s_u.work_q);
    bool scaled = (out_w != fi->h_res || out_h != fi->v_res);
    ESP_LOGI(TAG, "UVC 推流开始：%s %ux%u%s%s quality=%u fps_limit=%d",
             fmt_enum_name(fi->format), fi->h_res, fi->v_res,
             scaled ? " → " : "", scaled ? "ESP32缩放" : "",
             passthrough ? 0 : s_u.quality, s_u.fps_limit);
    return ESP_OK;
}

/* ---------- worker：消费帧 → 直通发布 / 重编码后发布 ---------- */
static void worker_task(void *arg)
{
    uvc_host_frame_t *f;
    for (;;) {
        if (!s_u.worker_run) {
            /* 停止态：归还队列中所有帧（teardown 已先排空过，这里兜底竞态余量） */
            uvc_host_stream_hdl_t h = s_u.stream;
            while (xQueueReceive(s_u.work_q, &f, 0) == pdTRUE) {
                if (h) uvc_host_frame_return(h, f);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (xQueueReceive(s_u.work_q, &f, pdMS_TO_TICKS(200)) != pdTRUE) continue;
        s_u.worker_busy = true;

        /* ★ t_arrival 用 frame_cb 内的统计口径近似：队列深度 ≤4、驱动任务优先级 12，
         * 排队延迟通常 < 5 ms；精确到达时刻即 frame_cb 执行时刻（README 误差节说明） */
        uint64_t t_arr = esp_timer_get_time();
        uvc_host_stream_hdl_t h = s_u.stream;

        /* 软件限帧（与 DVP 的 fps_limit 语义一致） */
        if (s_u.fps_limit > 0) {
            if (s_u.next_due_us == 0) s_u.next_due_us = t_arr;
            if (t_arr < s_u.next_due_us) {
                if (h) uvc_host_frame_return(h, f);
                s_u.worker_busy = false;
                continue;
            }
            s_u.next_due_us += 1000000ULL / s_u.fps_limit;
            if ((int64_t)(t_arr - s_u.next_due_us) > 500000) s_u.next_due_us = t_arr;   /* 欠账过多则重置 */
        }

        if (s_u.mode == USB_MODE_PASSTHROUGH) {
            /* ★ 模式 A：MJPEG 直通，不解码不编码，原样转发（quality 不可控，填 0 勿伪造） */
            uint64_t t0 = esp_timer_get_time();
            frame_meta_t meta = { .source = FRAME_SRC_USB, .scaled = 0,
                                  .ts_meaning = FRAME_TS_FRAME_ARRIVAL };
            bool ok = h && frame_ring_publish_ex(cam_pipe_ring(), f->data, f->data_len,
                                                 t_arr, t_arr, f->vs_format.h_res, f->vs_format.v_res,
                                                 0, &meta);
            uint64_t t1 = esp_timer_get_time();
            s_u.stats.proc_acc_us += t1 - t0;
            if (ok) s_u.stats.out_frames++; else s_u.stats.out_drops++;
            s_u.stats.out_bytes += f->data_len;
            if (h) uvc_host_frame_return(h, f);
        } else {
            /* ★ 模式 B：YUY2 → 硬件 JPEG 重编码（quality 可控，与 DVP 同档可比） */
            int idx;
            if (!h || xQueueReceive(s_u.enc_free_in, &idx, 0) != pdTRUE) {
                s_u.stats.cap_drops++;   /* 编码器忙：丢帧保时延 */
                if (h) uvc_host_frame_return(h, f);
                s_u.worker_busy = false;
                continue;
            }
            int kx, ky, x0, y0;
            plan_virtual(s_u.w, s_u.h, s_u.out_w, s_u.out_h, &kx, &ky, &x0, &y0);
            size_t out_bytes = MIN((size_t)s_u.out_w * s_u.out_h * 2, s_u.enc_in[idx].length);
            fill_encoder_input(s_u.enc_in[idx].start, f->data,
                               s_u.w, s_u.h, s_u.out_w, s_u.out_h, kx, ky, x0, y0, t_arr);
            struct v4l2_buffer vb = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_MMAP,
                                      .index = idx, .bytesused = out_bytes };
            if (xioctl(s_u.enc_fd, VIDIOC_QBUF, &vb) != 0) {
                xQueueSend(s_u.enc_free_in, &idx, 0);
                s_u.stats.cap_drops++;
                uvc_host_frame_return(h, f);
                s_u.worker_busy = false;
                continue;
            }
            struct v4l2_buffer ob = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
            bool enc_ok = xioctl(s_u.enc_fd, VIDIOC_DQBUF, &ob) == 0;   /* M2M：DQBUF 内同步编码 */
            uint64_t t_enc = esp_timer_get_time();
            xQueueSend(s_u.enc_free_in, &idx, 0);
            uvc_host_frame_return(h, f);

            if (enc_ok && ob.bytesused > 0 && !(ob.flags & V4L2_BUF_FLAG_ERROR)) {
                bool scaled = (s_u.out_w != s_u.w || s_u.out_h != s_u.h);
                frame_meta_t meta = { .source = FRAME_SRC_USB, .scaled = scaled,
                                      .ts_meaning = FRAME_TS_FRAME_ARRIVAL };
                bool ok = frame_ring_publish_ex(cam_pipe_ring(), s_u.enc_out[ob.index].start, ob.bytesused,
                                                t_arr, t_enc, s_u.out_w, s_u.out_h,
                                                s_u.quality, &meta);
                s_u.stats.proc_acc_us += t_enc - t_arr;
                if (ok) s_u.stats.out_frames++; else s_u.stats.out_drops++;
                s_u.stats.out_bytes += ob.bytesused;
            } else {
                s_u.stats.out_drops++;
            }
            xioctl(s_u.enc_fd, VIDIOC_QBUF, &ob);   /* 输出槽归还 */
        }
        s_u.worker_busy = false;
    }
}

/* ---------- monitor：USB 总线事件泵 + 状态机 + 自动重连（U5） ---------- */
static void monitor_task(void *arg)
{
    uint32_t flags;
    for (;;) {
        usb_host_lib_handle_events(pdMS_TO_TICKS(50), &flags);
        usb_host_client_handle_events(s_u.client, 0);   /* 非阻塞派发 NEW_DEV/DEV_GONE */

        if (s_u.teardown_req) {
            s_u.teardown_req = false;
            xSemaphoreTake(s_u.lock, portMAX_DELAY);
            teardown_stream();
            xSemaphoreGive(s_u.lock);
            s_u.next_retry_us = esp_timer_get_time() + RETRY_BACKOFF_MS * 1000;
        }

        /* 自动（重）连接：USB 为当前源 && 有设备 && 未推流 → 周期性尝试 */
        if (s_u.want_stream && s_u.dev_present && !s_u.stream_up && !s_u.teardown_req) {
            int64_t now = esp_timer_get_time();
            if (now >= s_u.next_retry_us) {
                xSemaphoreTake(s_u.lock, portMAX_DELAY);
                esp_err_t err = open_stream_locked();
                xSemaphoreGive(s_u.lock);
                if (err != ESP_OK) {
                    s_u.open_errors++;
                    s_u.next_retry_us = now + RETRY_BACKOFF_MS * 1000;
                    ESP_LOGW(TAG, "UVC 打开失败（%s），%d s 后重试；若反复失败请检查 USB 供电"
                             "（Korvo-1 Type-A 经 TPS2051C 限流 500 mA，须 Type-C 口补供电）",
                             esp_err_to_name(err), RETRY_BACKOFF_MS / 1000);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------- 公共 API（backend vtable 见 source_if.c） ---------- */
esp_err_t source_usb_init(void)
{
    memset(&s_u, 0, sizeof(s_u));
    s_u.enc_fd = -1;
    s_u.mode = USB_MODE_PASSTHROUGH;
#if defined(CONFIG_CAMTEST_USB_DEFAULT_MODE_REENCODE)
    s_u.mode = USB_MODE_REENCODE;
#endif
    s_u.quality = 20;
    s_u.fps_limit = 0;
    s_u.inherent_ms = -1.0f;
    s_u.lock = xSemaphoreCreateMutex();
    s_u.work_q = xQueueCreate(WORK_QUEUE_LEN, sizeof(uvc_host_frame_t *));
    s_u.enc_free_in = xQueueCreate(ENC_IN_BUFS, sizeof(int));
    ESP_RETURN_ON_FALSE(s_u.lock && s_u.work_q && s_u.enc_free_in, ESP_ERR_NO_MEM, TAG, "sync");

    /* USB Host 库（peripheral_map=0 → S31 默认 HS 外设；常驻不卸载以支持热插拔） */
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_RETURN_ON_ERROR(usb_host_install(&host_config), TAG, "usb_host_install");

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 8,
        .async = { .client_event_callback = client_event_cb, .callback_arg = NULL },
    };
    ESP_RETURN_ON_ERROR(usb_host_client_register(&client_config, &s_u.client), TAG, "client_register");

    uvc_host_driver_config_t drv_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 12,
        .xCoreID = 1,                   /* 帧回调打点后立即交给同核 worker */
        .create_background_task = true,
        .event_cb = drv_event_cb,
        .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(uvc_host_install(&drv_config), TAG, "uvc_host_install");

    if (xTaskCreatePinnedToCore(worker_task, "usb_work", 6144, NULL, 13, NULL, 1) != pdPASS)
        return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(monitor_task, "usb_mon", 4096, NULL, 4, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;

    s_u.inited = true;

    /* 开机已插入的设备不会补发 NEW_DEV 事件（usb_host_client_register 无回放逻辑），
     * 主动补扫设备地址列表（给枚举一点时间） */
    for (int round = 0; round < 10; round++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        uint8_t addr_list[8] = {0};
        int n = 0;
        usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &n);
        for (int i = 0; i < n; i++) {
            probe_device(addr_list[i]);
        }
        if (s_u.dev_present || n > 0) break;
    }
    if (!s_u.dev_present) {
        ESP_LOGI(TAG, "USB Host 就绪，未发现 UVC 设备（热插拔监听中；注意 Korvo-1 Type-A 口限流 500 mA）");
    }
    return ESP_OK;
}

void source_usb_set_active(bool active)
{
    s_u.want_stream = active;
    if (!active && s_u.stream_up) {
        s_u.teardown_req = true;   /* 切走：停流（monitor 执行清理） */
        int wait = 0;
        while (s_u.stream_up && wait++ < 100) vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (active) s_u.next_retry_us = 0;   /* 立即尝试 */
}

esp_err_t source_usb_start(void)
{
    if (!s_u.inited) return ESP_ERR_INVALID_STATE;
    if (!s_u.dev_present) {
        ESP_LOGW(TAG, "USB 摄像头未连接，无法启动 USB 源（插入后可重试或等待自动重连）");
        return ESP_ERR_NOT_FOUND;
    }
    source_usb_set_active(true);
    /* 等 monitor 首次尝试（open 含控制传输，通常 < 1 s） */
    for (int i = 0; i < 150 && !s_u.stream_up; i++) {
        if (!s_u.dev_present) return ESP_ERR_NOT_FOUND;   /* 期间被拔 */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return s_u.stream_up ? ESP_OK : ESP_FAIL;
}

void source_usb_stop(void)
{
    source_usb_set_active(false);
}

esp_err_t source_usb_apply(int w, int h, uint8_t quality, int fps_limit)
{
    if (quality) {
        if (s_u.mode == USB_MODE_PASSTHROUGH) {
            ESP_LOGW(TAG, "passthrough 模式画质由摄像头固件决定，quality=%u 忽略", quality);
        } else if (quality != s_u.quality) {
            s_u.quality = quality;
            if (s_u.enc_fd >= 0) {
                struct v4l2_ext_control ctrl = { .id = V4L2_CID_JPEG_COMPRESSION_QUALITY, .value = quality };
                struct v4l2_ext_controls ctrls = { .count = 1, .controls = &ctrl };
                xioctl(s_u.enc_fd, VIDIOC_S_EXT_CTRLS, &ctrls);
            }
        }
    }
    if (fps_limit >= 0) s_u.fps_limit = fps_limit;

    /* 分辨率变化需要重新协商流（直通必须原生精确；重编码经缩放） */
    bool res_change = (w && h) && (w != s_u.out_w || h != s_u.out_h);
    if (res_change) {
        source_usb_set_active(false);
        s_u.req_w = w;
        s_u.req_h = h;
        source_usb_set_active(true);
        for (int i = 0; i < 150 && !s_u.stream_up; i++) {
            if (!s_u.dev_present) return ESP_ERR_NOT_FOUND;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (!s_u.stream_up) return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t source_usb_set_quality(uint8_t q)
{
    if (s_u.mode == USB_MODE_PASSTHROUGH) return ESP_ERR_NOT_SUPPORTED;
    s_u.quality = q;
    if (s_u.enc_fd >= 0) {
        struct v4l2_ext_control ctrl = { .id = V4L2_CID_JPEG_COMPRESSION_QUALITY, .value = q };
        struct v4l2_ext_controls ctrls = { .count = 1, .controls = &ctrl };
        return xioctl(s_u.enc_fd, VIDIOC_S_EXT_CTRLS, &ctrls) ? ESP_FAIL : ESP_OK;
    }
    return ESP_OK;
}

void source_usb_set_fps_limit(int fps)
{
    s_u.fps_limit = fps > 0 ? fps : 0;
}

void source_usb_set_overlay(bool on)
{
    /* 直通模式无法叠加（不解码）；仅重编码模式生效，UI 提示 */
    if (on && s_u.mode == USB_MODE_PASSTHROUGH) {
        ESP_LOGW(TAG, "passthrough 模式不解码，无法叠加毫秒计数器；请切换 reencode 模式做光学闭环");
    }
    s_u.overlay = on;
}

bool source_usb_overlay(void) { return s_u.overlay; }

src_info_t *source_usb_info(void)
{
    static src_info_t info;   /* 单消费者轮询（http/metrics），无并发写冲突 */
    memset(&info, 0, sizeof(info));
    info.source = VIDEO_SOURCE_USB;
    info.needs_encode = (s_u.mode == USB_MODE_REENCODE);
    info.quality = (s_u.mode == USB_MODE_REENCODE) ? s_u.quality : 0;
    info.usb_mode = s_u.mode;
    info.ts_meaning = TS_MEANING_FRAME_ARRIVAL;
    info.w = s_u.stream_up ? s_u.out_w : 0;
    info.h = s_u.stream_up ? s_u.out_h : 0;
    info.scaled = s_u.stream_up && (s_u.out_w != s_u.w || s_u.out_h != s_u.h);
    info.native_w = s_u.w;
    info.native_h = s_u.h;
    info.nominal_fps = 0;
    strlcpy(info.sensor_name, s_u.dev_name[0] ? s_u.dev_name : "USB-UVC", sizeof(info.sensor_name));
    strlcpy(info.usb_device_name, s_u.dev_name, sizeof(info.usb_device_name));
    strlcpy(info.pix_fmt_str, s_u.mode == USB_MODE_PASSTHROUGH ? "JPEG" : "YUY2→JPEG", sizeof(info.pix_fmt_str));
    snprintf(info.fmt_name, sizeof(info.fmt_name), "%s%s%ux%u",
             s_u.mode == USB_MODE_PASSTHROUGH ? "MJPEG直通" : "YUY2重编码",
             info.scaled ? "缩放" : "", info.w, info.h);
    return &info;
}

src_stats_t *source_usb_stats(void) { return &s_u.stats; }

frame_ring_t *source_usb_ring(void) { return cam_pipe_ring(); }

int source_usb_supported_res(char *out, size_t outlen)
{
    /* 当前模式下的可用档：直通=MJPEG 原生精确档；重编码=YUY2 原生 + 可缩放的虚拟档 */
    int n = 0;
    out[0] = 0;
    bool passthrough = (s_u.mode == USB_MODE_PASSTHROUGH);
    for (int i = 0; i < s_u.tier_n; i++) {
        usb_tier_t *t = &s_u.tiers[i];
        if (passthrough != t->mjpeg) continue;
        char item[16];
        snprintf(item, sizeof(item), "%dx%d", t->w, t->h);
        if (strstr(out, item)) continue;
        if (n) strlcat(out, ",", outlen);
        strlcat(out, item, outlen);
        n++;
    }
    if (passthrough) return n;
    /* 重编码：补齐可由 YUY2 原生档缩放得到的标准虚拟档 */
    static const int tiers[][2] = { {160,120},{320,240},{480,320},{640,480} };
    for (size_t t = 0; t < sizeof(tiers) / sizeof(tiers[0]); t++) {
        for (int i = 0; i < s_u.tier_n; i++) {
            usb_tier_t *ti = &s_u.tiers[i];
            if (ti->mjpeg) continue;
            int kx, ky, x0, y0;
            if (!plan_virtual(ti->w, ti->h, tiers[t][0], tiers[t][1], &kx, &ky, &x0, &y0)) continue;
            char item[16];
            snprintf(item, sizeof(item), "%dx%d", tiers[t][0], tiers[t][1]);
            if (strstr(out, item)) continue;
            if (n) strlcat(out, ",", outlen);
            strlcat(out, item, outlen);
            n++;
            break;
        }
    }
    return n;
}

bool source_usb_res_supported(int w, int h)
{
    for (int i = 0; i < s_u.tier_n; i++) {
        usb_tier_t *t = &s_u.tiers[i];
        if (s_u.mode == USB_MODE_PASSTHROUGH) {
            if (t->mjpeg && t->w == w && t->h == h) return true;
        } else if (!t->mjpeg) {
            int kx, ky, x0, y0;
            if (t->w == w && t->h == h) return true;
            if (plan_virtual(t->w, t->h, w, h, &kx, &ky, &x0, &y0)) return true;
        }
    }
    return false;
}

/* ---------- USB 专用查询（source_if.h 对外） ---------- */
usb_state_t source_usb_state(void)
{
    if (!s_u.inited) return USB_STATE_DISABLED;
    if (s_u.stream_up) return USB_STATE_STREAMING;
    if (!s_u.dev_present) return USB_STATE_NO_DEVICE;
    if (s_u.want_stream && s_u.open_errors > 0 && !s_u.stream_up) return USB_STATE_ERROR;
    return USB_STATE_DEVICE_READY;
}

usb_mode_t source_usb_mode(void) { return s_u.mode; }

esp_err_t source_usb_set_mode(usb_mode_t m)
{
    if (m != USB_MODE_PASSTHROUGH && m != USB_MODE_REENCODE) return ESP_ERR_INVALID_ARG;
    if (m == s_u.mode) return ESP_OK;
    bool was_active = s_u.want_stream;
    usb_mode_t old = s_u.mode;
    s_u.mode = m;
    if (was_active && s_u.dev_present) {
        source_usb_set_active(false);
        s_u.next_retry_us = 0;
        source_usb_set_active(true);
        for (int i = 0; i < 150 && !s_u.stream_up; i++) vTaskDelay(pdMS_TO_TICKS(20));
        if (!s_u.stream_up) {
            /* 新模式失败 → 回滚旧模式（保持可用而非死机） */
            ESP_LOGW(TAG, "切换到 %s 失败，回滚 %s", 
                     m == USB_MODE_PASSTHROUGH ? "passthrough" : "reencode",
                     old == USB_MODE_PASSTHROUGH ? "passthrough" : "reencode");
            s_u.mode = old;
            s_u.next_retry_us = 0;
            source_usb_set_active(true);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

uint32_t source_usb_disconnects(void) { return s_u.disconnects; }
const char *source_usb_device_name(void) { return s_u.dev_name; }

int source_usb_get_tiers(usb_tier_t *out, int max)
{
    int n = s_u.tier_n < max ? s_u.tier_n : max;
    memcpy(out, s_u.tiers, n * sizeof(usb_tier_t));
    return n;
}

float source_usb_inherent_ms(void) { return s_u.inherent_ms; }
void source_usb_set_inherent_ms(float ms) { s_u.inherent_ms = ms; }

#endif /* CONFIG_CAMTEST_ENABLE_USB */
