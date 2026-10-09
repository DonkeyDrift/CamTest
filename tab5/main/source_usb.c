/*
 * source_usb.c — USB UVC 摄像头采集源后端（Tab5/ESP32-P4，usb_host_uvc 原生 API）
 *
 * 移植自 S31 CamTest（状态机/热插拔/三层看门狗/帧归还协议逐字保留——那些都是真机
 * 流血换来的），重写编码段：
 *
 * 三模式（usb_mode_t）：
 *   passthrough  MJPEG 直通（原样转发，quality 不可控；帧率上限最高）
 *   h264  ★默认  MJPEG → esp_driver_jpeg 硬解码(UYVY) → esp_h264 硬件 H.264
 *                → 帧环(codec=H264, Annex-B AU)。码率可控（governor/网页），可叠 OSD
 *   reencode     MJPEG → 硬解码 → esp_driver_jpeg 硬件 JPEG（S31 语义保留，标定/对比）
 *   （YUY2 档仅在摄像头无 MJPEG 档时作回退输入，字节序交换后进编码）
 *
 * P4 与 S31 的关键差异：USB 走 HS 480Mbps（Tab5 Type-A 直连 P4 OTG HS），
 * 720p@60 MJPEG 带宽不再受限；Wi-Fi 在 C6 协处理器上（esp_hosted SDIO），
 * 链路吞吐未知→ 传输侧帧率上限待真机标定，先沿用 S31 的保守档位策略。
 */
#include "source_if.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/param.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "driver/jpeg_decode.h"
#include "driver/jpeg_encode.h"
#include "driver/ppa.h"
#include "h264_pipeline.h"
#include "jpeg_dec_share.h"
#include "yuv_osd.h"
#include "sdkconfig.h"

#if CONFIG_CAMTEST_ENABLE_USB

static const char *TAG = "src_usb";

#define USB_DEV_ANY          0
#define FRAME_BUFFERS        4      /* uvc 驱动帧缓冲数（PSRAM；平滑 60fps 突发） */
#define NUM_URBS             8      /* ISOC 在途 URB 数（S31 实测 8 个零丢帧） */
#define URB_SIZE             (10 * 1024)
#define WORK_QUEUE_LEN       4
#define STREAM_OPEN_TIMEOUT_MS 5000
#define RETRY_BACKOFF_MS     1000
/* 帧停滞看门狗（S31 坑 #10：驱动静默停流不回调不打日志，停后流仍报 up=1） */
#define STALL_TIMEOUT_US     (5 * 1000000)
#define STALL_MAX_ROUNDS     6
/* 连续打开失败满 60 s → 重启：只有重启触发 USB 重枚举才自愈（S31 实测） */
#define OPEN_FAIL_REBOOT_US  (60 * 1000000)
#define DEC_TIMEOUT_MS       100
#define ENC_TIMEOUT_MS       100

/* ---------- 私有状态 ---------- */
struct usb_priv_s {
    /* USB 总线层 */
    usb_host_client_handle_t client;
    volatile bool dev_present;
    uint8_t  dev_addr;
    uint8_t  stream_index;
    uint8_t  idx_candidates[8];
    int      idx_cand_n;
    uint16_t vid, pid;
    char     dev_name[SRC_USB_NAME_MAX];

    /* 描述符枚举结果 */
    uvc_host_frame_info_t frame_info[USB_TIER_MAX * 4];
    size_t  frame_info_n;
    usb_tier_t tiers[USB_TIER_MAX];
    int     tier_n;
    bool    mjpeg_ok, yuy2_ok;

    /* 运行配置 */
    usb_mode_t mode;                            /* passthrough / h264 / reencode */
    uint16_t req_w, req_h;
    uint16_t w, h;                              /* 当前协商的摄像头原生分辨率 */
    uint16_t out_w, out_h;                      /* 输出（重编码缩放后）分辨率 */
    uint8_t  quality;                           /* 仅 reencode（JPEG 1..100） */
    uint32_t h264_kbps;                         /* H.264 目标码率 */
    uint8_t  h264_gop;
    volatile int fps_limit;
    uint64_t next_due_us;

    /* uvc 流 */
    volatile uvc_host_stream_hdl_t stream;
    volatile bool stream_up;
    volatile uint32_t stream_gen;

    /* MJPEG 硬解码（h264/reencode 共用）：引擎走 jpeg_dec_share（全局单例，
     * 与 lcd_ui 预览互斥共享），此处只管输出缓冲 */
    uint8_t *dec_out;                           /* 解码输出（native 16对齐尺寸 UYVY） */
    size_t dec_out_len;
    bool dec_src_mjpeg;                         /* 当前流是否 MJPEG 档 */
    uint8_t *yuy2_buf;                          /* YUY2 档回退时的字节序交换暂存（native UYVY） */

    /* H.264 编码器（仅 h264 模式）：UYVY 软件重排 O_UYY_E_VYY 后直喂（缩放走 PPA） */
    h264_pipeline_handle_t h264;
    ppa_client_handle_t ppa;                    /* SRM 客户端（420 裁剪/缩放，2D-DMA） */
    uint8_t *h264_in;                           /* 编码器输入（O_UYY_E_VYY，64 对齐） */
    size_t h264_in_len;
    uint8_t *conv_buf;                          /* 缩放路径的 native 420 重排目标 */

    /* JPEG 编码器（仅 reencode 模式；P4 esp_driver_jpeg 直驱） */
    jpeg_encoder_handle_t jenc;
    struct { uint8_t *start; size_t length; } jenc_in, jenc_out;

    /* 同步 */
    QueueHandle_t work_q;
    SemaphoreHandle_t lock;
    volatile bool worker_run;
    volatile bool worker_busy;
    volatile bool want_stream;
    volatile bool teardown_req;
    int64_t  next_retry_us;

    /* 统计 */
    src_stats_t stats;
    uint32_t disconnects;
    uint32_t open_errors;
    uint32_t overflow_events;
    volatile int frames_held;
    volatile uint32_t cb_nohandle;

    /* 叠加与标定 */
    bool overlay;
    float inherent_ms;

    bool inited;
};

static struct usb_priv_s s_u;

/* ---------- 小工具 ---------- */
/* ---------- UYVY(YVYU) 422 → O_UYY_E_VYY 420 重排 ----------
 * ★ rev<3.0 的 P4：jpeg 解码器禁止 422→420 转换、PPA 不收 YUV422 输入，故此步
 * 只能软件完成。无损：422 色度逐行都有，重排仅改布局（奇数输出行携带该行的
 * U，偶数输出行携带该行的 V，Y 全保留）。
 * 输入行：[U0 Y0 V0 Y1][U1 Y2 V1 Y3]…（2px/4B）；输出行：[U0 Y0 Y1][U1 Y2 Y3]…（2px/3B），
 * 偶数输出行同位取下一行的 V。24→32 位重打包（u32 读写、零 memcpy），PSRAM 带宽受限操作。 */
static inline uint32_t ouev_pack_u(uint32_t x)   /* [U Y0 V Y1] → [U Y0 Y1|0] */
{
    return (x & 0x0000FFFF) | ((x >> 8) & 0x00FF0000);
}
static inline uint32_t ouev_pack_v(uint32_t x)   /* [U Y0 V Y1] → [V Y0 Y1|0] */
{
    return ((x >> 16) & 0xFF) | (x & 0x00FF00) | ((x >> 8) & 0x00FF0000);
}
static void uyvy_to_ouev(const uint8_t *src, int w, int h, uint8_t *dst)
{
    const int pairs = w / 2;                       /* 每行像素对数 */
    const int quads = pairs / 4;                   /* 每行 4 对一组（16B→12B） */
    const size_t srow = (size_t)w * 2, drow = (size_t)pairs * 3;
    for (int y = 0; y < h; y += 2) {
        const uint32_t *ru = (const uint32_t *)(src + (size_t)y * srow);
        const uint32_t *rv = (const uint32_t *)(src + (size_t)(y + 1 < h ? y + 1 : y) * srow);
        uint32_t *du = (uint32_t *)(dst + (size_t)y * drow);
        uint32_t *dv = (uint32_t *)(dst + (size_t)(y + 1 < h ? y + 1 : y) * drow);
        for (int q = 0; q < quads; q++) {
            /* 4 对输入（u0..u3 / v0..v3）→ 12B 输出：3×u32 交叠写 */
            uint32_t u0 = ouev_pack_u(ru[q*4+0]), u1 = ouev_pack_u(ru[q*4+1]);
            uint32_t u2 = ouev_pack_u(ru[q*4+2]), u3 = ouev_pack_u(ru[q*4+3]);
            du[q*3+0] = u0 | (u1 << 24);
            du[q*3+1] = (u1 >> 8) | (u2 << 16);
            du[q*3+2] = (u2 >> 16) | (u3 << 8);
            uint32_t v0 = ouev_pack_v(rv[q*4+0]), v1 = ouev_pack_v(rv[q*4+1]);
            uint32_t v2 = ouev_pack_v(rv[q*4+2]), v3 = ouev_pack_v(rv[q*4+3]);
            dv[q*3+0] = v0 | (v1 << 24);
            dv[q*3+1] = (v1 >> 8) | (v2 << 16);
            dv[q*3+2] = (v2 >> 16) | (v3 << 8);
        }
        /* 尾部奇数对（w 非 8 倍数时）：保守标量处理 */
        for (int p = quads * 4; p < pairs; p++) {
            uint32_t iu = ru[p], iv = rv[p];
            uint32_t ou = ouev_pack_u(iu), ov = ouev_pack_v(iv);
            __builtin_memcpy((uint8_t *)du + p * 3, &ou, 3);
            __builtin_memcpy((uint8_t *)dv + p * 3, &ov, 3);
        }
    }
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

/* → 编码器输入（UYVY）：整数抽取/中心裁剪 + 可选字节序交换 + 可选 OSD。
 * swap_bytes：YUY2 摄像头源需交换为 UYVY；MJPEG 硬解码输出已是 UYVY，传 false */
static void fill_encoder_input(uint8_t *dst, const uint8_t *src, int sw, int sh,
                               int tw, int th, int kx, int ky, int x0, int y0,
                               bool swap_bytes, uint64_t now_us)
{
    const size_t srow = (size_t)sw * 2, drow = (size_t)tw * 2;
    const bool passthrough = (tw == sw && th == sh && kx == 1 && ky == 1 && x0 == 0 && y0 == 0);
    if (passthrough) {
        const uint16_t *sp = (const uint16_t *)src;
        uint16_t *dp = (uint16_t *)dst;
        if (swap_bytes) {
            for (size_t i = 0; i < drow * sh / 2; i++) dp[i] = __builtin_bswap16(sp[i]);
        } else {
            memcpy(dp, sp, drow * sh);
        }
    } else {
        for (int ty = 0; ty < th; ty++) {
            const uint8_t *sr = src + (size_t)(y0 + ty * ky) * srow;
            uint8_t *dr = dst + (size_t)ty * drow;
            for (int tx = 0; tx < tw; tx += 2) {
                const uint8_t *sp = sr + ((size_t)(x0 / 2) + (size_t)(tx / 2) * kx) * 4;
                uint8_t *dp = dr + (size_t)tx * 2;
                if (swap_bytes) { dp[0]=sp[1]; dp[1]=sp[0]; dp[2]=sp[3]; dp[3]=sp[2]; }
                else            { dp[0]=sp[0]; dp[1]=sp[1]; dp[2]=sp[2]; dp[3]=sp[3]; }
            }
        }
    }
    if (s_u.overlay) {
        /* reencode 缓冲为 YVYU（jpeg 解码 YUV422 输出 / 编码器输入同 fourcc），
         * Y 在偶字节 → OSD_FMT_YUYV */
        yuv_osd_draw_ms_counter(dst, drow, tw, th, OSD_FMT_YUYV, now_us);
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

/* ---------- 描述符枚举：打印完整 (格式,分辨率,帧率) 列表并生成档位表 ---------- */
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
             s_u.mode == USB_MODE_PASSTHROUGH ? "passthrough" :
             s_u.mode == USB_MODE_H264 ? "h264" : "reencode");
}

/* ---------- 设备到达/离开 ---------- */
static void probe_device(uint8_t dev_addr)
{
    if (s_u.dev_present && s_u.dev_addr == dev_addr) return;

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

    int cand_n = 0;
    for (uint8_t idx = 0; idx < 6 && cand_n < (int)sizeof(s_u.idx_candidates); idx++) {
        size_t need = 0;
        if (uvc_host_get_frame_list(dev_addr, idx, NULL, &need) == ESP_OK && need > 0) {
            s_u.idx_candidates[cand_n++] = idx;
        }
    }
    if (cand_n == 0) {
        ESP_LOGI(TAG, "USB 设备 %s (addr %u) 不是 UVC 摄像头，忽略", name, dev_addr);
        return;
    }

    xSemaphoreTake(s_u.lock, portMAX_DELAY);
    s_u.dev_present = true;
    s_u.dev_addr = dev_addr;
    s_u.idx_cand_n = cand_n;
    s_u.stream_index = s_u.idx_candidates[0];
    strlcpy(s_u.dev_name, name, sizeof(s_u.dev_name));
    size_t cap = sizeof(s_u.frame_info) / sizeof(s_u.frame_info[0]);
    s_u.frame_info_n = 0;
    if (uvc_host_get_frame_list(dev_addr, s_u.stream_index, (uvc_host_frame_info_t (*)[])s_u.frame_info, &cap) == ESP_OK) {
        s_u.frame_info_n = cap;
    }
    xSemaphoreGive(s_u.lock);

    ESP_LOGI(TAG, "===== UVC 设备已连接：name=%s VID/PID=%04x/%04x addr=%u 流索引候选=[%d 个: %u%s] =====",
             name, s_u.vid, s_u.pid, dev_addr, cand_n, s_u.idx_candidates[0],
             cand_n > 1 ? ",…" : "");
    rebuild_tiers();
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
        s_u.teardown_req = true;
        break;
    case UVC_HOST_TRANSFER_ERROR:
        s_u.open_errors++;
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        s_u.overflow_events++;
        s_u.stats.cap_drops++;
        break;
    case UVC_HOST_DEVICE_SUSPENDED:
        /* 驱动收到挂起已静默 pause（帧停、零日志）——置 teardown 让 monitor 关流，
         * RESUMED 后按 want_stream 自动重开 */
        ESP_LOGW(TAG, "UVC 流事件：设备挂起（suspend），停流待 monitor 重开");
        s_u.stream_up = false;
        s_u.teardown_req = true;
        break;
    case UVC_HOST_DEVICE_RESUMED:
        ESP_LOGW(TAG, "UVC 流事件：设备恢复（resume），monitor 将按 want 自动重开");
        break;
    default:
        break;
    }
}

/* 帧归还唯一出口：任何路径漏归还都会让 uvc_host_stream_close 永久失败
 * （"Not all frames are returned" → 流泄漏占住接口），统一从此走并记账 */
static inline void ret_frame(uvc_host_stream_hdl_t h, uvc_host_frame_t *f)
{
    if (h) {
        uvc_host_frame_return(h, f);
    } else {
        s_u.cb_nohandle++;
    }
    if (s_u.frames_held > 0) s_u.frames_held--;
}

static bool frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    /* ★ t_arrival 打点：完整一帧到达 ESP32 的时刻（含摄像头内部曝光/ISP/编码/USB 传输延迟） */
    s_u.stats.cap_frames++;
    uvc_host_stream_hdl_t h = s_u.stream;
    if (!h) {
        s_u.cb_nohandle++;
        return false;
    }
    s_u.frames_held++;
    if (xQueueSend(s_u.work_q, &frame, 0) != pdPASS) {
        s_u.stats.cap_drops++;      /* 队列满：丢帧保时延 */
        ret_frame(h, (uvc_host_frame_t *)frame);
    }
    return false;   /* 帧由 worker 用完显式归还 */
}

/* ---------- 编码链路（h264 / reencode；passthrough 无编码器） ---------- */
static void encoder_close(void)
{
    if (s_u.h264) {
        h264_pipeline_close(s_u.h264);
        s_u.h264 = NULL;
    }
    if (s_u.h264_in) {
        heap_caps_free(s_u.h264_in);
        s_u.h264_in = NULL;
        s_u.h264_in_len = 0;
    }
    if (s_u.conv_buf) {
        heap_caps_free(s_u.conv_buf);
        s_u.conv_buf = NULL;
    }
    if (s_u.yuy2_buf) {
        heap_caps_free(s_u.yuy2_buf);
        s_u.yuy2_buf = NULL;
    }
    if (s_u.jenc) {
        jpeg_del_encoder_engine(s_u.jenc);
        s_u.jenc = NULL;
    }
    if (s_u.jenc_in.start) {
        heap_caps_free(s_u.jenc_in.start);
        s_u.jenc_in.start = NULL;
        s_u.jenc_in.length = 0;
    }
    if (s_u.jenc_out.start) {
        heap_caps_free(s_u.jenc_out.start);
        s_u.jenc_out.start = NULL;
        s_u.jenc_out.length = 0;
    }
    if (s_u.dec_out) {
        heap_caps_free(s_u.dec_out);
        s_u.dec_out = NULL;
        s_u.dec_out_len = 0;
    }
    /* 解码引擎已改全局共享（jpeg_dec_share）：此处不再销毁（lcd_ui 预览共用） */
}

static esp_err_t encoder_open(int out_w, int out_h, int fps)
{
    ESP_LOGI(TAG, "encoder_open: %dx%d mode=%s%s", out_w, out_h,
             s_u.mode == USB_MODE_H264 ? "h264" : "reencode",
             s_u.dec_src_mjpeg ? "（MJPEG 输入＋硬解码）" : "");
    if (s_u.h264 || s_u.jenc) encoder_close();   /* 重入防护 */
    const bool need_dec = s_u.dec_src_mjpeg;
    if (need_dec) {
        int nw = (s_u.w + 15) & ~15, nh = (s_u.h + 15) & ~15;
        jpeg_decode_memory_alloc_cfg_t mcfg = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
        /* 两种模式都解码 YUV422（rev<3.0 禁止 422→420 转换）：h264 软件重排为
         * O_UYY_E_VYY 后进编码器；reencode 直喂 jpeg 编码器（同 fourcc 零转换） */
        s_u.dec_out = jpeg_alloc_decoder_mem((size_t)nw * nh * 2, &mcfg, &s_u.dec_out_len);
        if (!s_u.dec_out) {
            ESP_LOGE(TAG, "解码输出缓冲分配失败（%dx%d UYVY）", nw, nh);
            encoder_close();
            return ESP_FAIL;
        }
        if (s_u.mode == USB_MODE_H264 && (s_u.out_w != s_u.w || s_u.out_h != s_u.h)) {
            /* 缩放路径：先重排为 native 420，再 PPA 裁剪/缩放到目标 */
            s_u.conv_buf = heap_caps_aligned_alloc(64, (size_t)nw * nh * 3 / 2,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!s_u.conv_buf) {
                ESP_LOGE(TAG, "重排缓冲分配失败（%dx%d 420）", nw, nh);
                encoder_close();
                return ESP_FAIL;
            }
        }
        ESP_LOGI(TAG, "MJPEG 硬解码就绪：%dx%d→YUV422（缓冲 %u KB）%s", s_u.w, s_u.h,
                 (unsigned)(s_u.dec_out_len / 1024),
                 s_u.conv_buf ? "＋重排缓冲" : "");
    }

    if (s_u.mode == USB_MODE_H264) {
        /* 输入尺寸必须 16 对齐（H.264 宏块）——pick_native 已按 plan_virtual 保证可裁剪达标 */
        uint16_t ew = (s_u.out_w + 15) & ~15, eh = (s_u.out_h + 15) & ~15;
        esp_err_t herr = h264_pipeline_open(&s_u.h264, ew, eh, fps > 0 ? fps : 30,
                                            s_u.h264_kbps, s_u.h264_gop);
        if (herr != ESP_OK) {
            ESP_LOGE(TAG, "H.264 编码器打开失败");
            encoder_close();
            return ESP_FAIL;
        }
        /* PPA 缩放输出 = 编码器输入缓冲（O_UYY_E_VYY；无缩放时编码器直接吃解码缓冲，
         * 此缓冲仅在有缩放需求时被用到，仍预分配——PSRAM 充裕） */
        s_u.h264_in_len = (size_t)ew * eh * 3 / 2;
        s_u.h264_in = heap_caps_aligned_alloc(64, s_u.h264_in_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_u.h264_in) {
            ESP_LOGE(TAG, "H.264 输入缓冲分配失败（%u KB）", (unsigned)(s_u.h264_in_len / 1024));
            encoder_close();
            return ESP_FAIL;
        }
        if (ew != s_u.out_w || eh != s_u.out_h) {
            ESP_LOGW(TAG, "H.264 输入对齐到 %ux%u（请求 %ux%u，出帧头仍标 %ux%u）",
                     ew, eh, s_u.out_w, s_u.out_h);
        }
    }
    /* JPEG 编码引擎：reencode 主路径 / h264 预览流（无 WebCodecs 的浏览器靠它出画面；
     * 解码帧本就在手，仅多一次 ~6ms 硬编码，15fps 帧预算 66ms 充裕） */
    {
        jpeg_encode_engine_cfg_t ecfg = { .intr_priority = 0, .timeout_ms = ENC_TIMEOUT_MS };
        esp_err_t eerr = jpeg_new_encoder_engine(&ecfg, &s_u.jenc);
        if (eerr != ESP_OK) {
            ESP_LOGE(TAG, "jpeg 编码引擎创建失败 %s", esp_err_to_name(eerr));
            encoder_close();
            return ESP_FAIL;
        }
        jpeg_encode_memory_alloc_cfg_t icfg = { .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER };
        s_u.jenc_in.start = jpeg_alloc_encoder_mem((size_t)s_u.out_w * s_u.out_h * 2, &icfg,
                                                   &s_u.jenc_in.length);
        jpeg_encode_memory_alloc_cfg_t ocfg = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
        s_u.jenc_out.start = jpeg_alloc_encoder_mem((size_t)s_u.out_w * s_u.out_h + 65536, &ocfg,
                                                    &s_u.jenc_out.length);
        if (!s_u.jenc_in.start || !s_u.jenc_out.start) {
            ESP_LOGE(TAG, "JPEG 编码缓冲分配失败");
            encoder_close();
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "encoder_open OK");
    return ESP_OK;
}

/* ---------- 档位选择 ---------- */
static int pick_native(bool want_mjpeg, int want_w, int want_h, bool scalable_ok)
{
    int fallback = -1;
    int fallback_640 = -1;
    int scalable = -1;
    for (size_t i = 0; i < s_u.frame_info_n; i++) {
        uvc_host_frame_info_t *fi = &s_u.frame_info[i];
        if (want_mjpeg && fi->format != UVC_VS_FORMAT_MJPEG) continue;
        if (!want_mjpeg && fi->format != UVC_VS_FORMAT_YUY2) continue;
        if (want_w <= 0 || want_h <= 0) {
            if (fallback < 0) fallback = i;
            if (fi->h_res == 640 && fi->v_res == 480) fallback_640 = i;
            continue;
        }
        if (fi->h_res == want_w && fi->v_res == want_h) return i;
        int kx, ky, x0, y0;
        if (scalable_ok && scalable < 0 &&
            plan_virtual(fi->h_res, fi->v_res, want_w, want_h, &kx, &ky, &x0, &y0)) {
            scalable = i;
        }
    }
    if (fallback_640 >= 0 && (want_w <= 0 || want_h <= 0)) return fallback_640;
    if (fallback >= 0 && (want_w <= 0 || want_h <= 0)) return fallback;
    return scalable;
}

static esp_err_t try_open_stream_on_index(uint8_t stream_idx);

/* ---------- 清理：顺序敏感（先停流 → 归还全部帧 → 关流 → 关编码器） ----------
 * ★ s_u.stream 必须在 close 成功之后才能置 NULL（S31 坑 #5/#9：提前置 NULL 会让
 *   worker 丢弃竞态帧 → close 永久失败 → 流泄漏占住接口） */
static int drain_work_queue(uvc_host_stream_hdl_t h)
{
    uvc_host_frame_t *f;
    int n = 0;
    while (xQueueReceive(s_u.work_q, &f, 0) == pdTRUE) {
        ret_frame(h, f);
        n++;
    }
    return n;
}

static bool safe_close_stream(uvc_host_stream_hdl_t h)
{
    if (!h) return true;
    for (int i = 0; i < 20; i++) {
        int leaked = drain_work_queue(h);
        if (leaked) {
            ESP_LOGW(TAG, "safe_close：第 %d 轮追回 %d 个竞态帧", i, leaked);
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        if (uvc_host_stream_close(h) == ESP_OK) return true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGE(TAG, "stream_close 20 轮仍失败（held=%d nohandle=%u）——保留句柄待迟到帧归还后重试",
             s_u.frames_held, (unsigned)s_u.cb_nohandle);
    return false;
}

static void teardown_stream(void)
{
    ESP_LOGW(TAG, "teardown_stream 被调用（stream=%d want=%d held=%d）",
             s_u.stream ? 1 : 0, (int)s_u.want_stream, s_u.frames_held);
    s_u.worker_run = false;
    if (s_u.stream) {
        uvc_host_stream_hdl_t h = s_u.stream;
        uvc_host_stream_stop(h);
        int wait = 0;
        while (s_u.worker_busy && wait++ < 100) vTaskDelay(pdMS_TO_TICKS(10));
        bool worker_idle = !s_u.worker_busy;

        s_u.stream_up = false;
        if (safe_close_stream(h)) {
            s_u.stream = NULL;
        } else {
            ESP_LOGE(TAG, "close 未完成，保留流句柄（held=%d），monitor 将重试", s_u.frames_held);
        }
        if (worker_idle) {
            encoder_close();
        } else {
            ESP_LOGW(TAG, "worker 仍忙碌，编码器暂不关闭（monitor 重试时补关）");
        }
    } else {
        s_u.stream_up = false;
        encoder_close();
    }
}

static esp_err_t open_stream_locked(void)
{
    if (!s_u.dev_present) return ESP_ERR_NOT_FOUND;

    if (s_u.stream) {
        s_u.worker_run = false;
        if (safe_close_stream(s_u.stream)) {
            s_u.stream = NULL;
        } else {
            return ESP_FAIL;
        }
    }

    for (int cand = 0; cand < s_u.idx_cand_n; cand++) {
        esp_err_t err = try_open_stream_on_index(s_u.idx_candidates[cand]);
        if (err == ESP_OK) return ESP_OK;
        if (err == ESP_ERR_NOT_SUPPORTED) return err;
        ESP_LOGW(TAG, "流索引 %u 打开失败（%s），尝试下一个候选",
                 s_u.idx_candidates[cand], esp_err_to_name(err));
    }
    return ESP_FAIL;
}

/* 帧率选择：≤640x480 传 0（设备默认=最高档）；更大档位选 ≤30fps 的最大档。
 * ★ fps 必须与驱动同源浮点计算（10000000.0f/interval）——显式整数会匹配失败（S31 坑 #6）。
 * P4 侧初始沿用该保守策略（C6 Wi-Fi 链路吞吐未知），真机标定后放开 */
static float pick_tier_fps(const uvc_host_frame_info_t *fi)
{
    if ((int)fi->h_res * (int)fi->v_res <= 640 * 480) return 0.0f;
    float best = 0.0f;
    for (int j = 0; j < fi->interval_type && j < 8; j++) {
        float f = 10000000.0f / (float)fi->interval[j];
        if (f > 0.5f && f <= 30.0f + 0.5f && f > best) best = f;
    }
    return best;
}

/* 在指定 UVC 流索引上完成 取档位表 → 选档 → open → start（→ 编码器） */
static esp_err_t try_open_stream_on_index(uint8_t stream_idx)
{
    bool passthrough = (s_u.mode == USB_MODE_PASSTHROUGH);
    bool needs_mjpeg_pref = !passthrough;   /* h264/reencode 优先 MJPEG 档（60fps + 可解码） */

    size_t cap = sizeof(s_u.frame_info) / sizeof(s_u.frame_info[0]);
    s_u.frame_info_n = 0;
    ESP_RETURN_ON_ERROR(uvc_host_get_frame_list(s_u.dev_addr, stream_idx,
                                                (uvc_host_frame_info_t (*)[])s_u.frame_info, &cap),
                        TAG, "get_frame_list idx=%u", stream_idx);
    s_u.frame_info_n = cap;

    int idx;
    bool src_mjpeg;
    if (passthrough) {
        idx = pick_native(true, s_u.req_w, s_u.req_h, false);
        src_mjpeg = true;
    } else if (s_u.mode == USB_MODE_H264) {
        /* h264 仅走 MJPEG 档（解码直出 H264 所需的 YUV420 布局；
         * YUY2 档只有 10fps 且需软件转换，不支持） */
        idx = pick_native(true, s_u.req_w, s_u.req_h, true);
        src_mjpeg = true;
    } else if ((idx = pick_native(true, s_u.req_w, s_u.req_h, true)) >= 0) {
        src_mjpeg = true;
    } else {
        idx = pick_native(false, s_u.req_w, s_u.req_h, true);
        src_mjpeg = false;
    }
    if (idx < 0) {
        ESP_LOGE(TAG, "%s 模式下不支持 %dx%d（见档位列表）",
                 passthrough ? "passthrough" :
                 s_u.mode == USB_MODE_H264 ? "h264" : "reencode", s_u.req_w, s_u.req_h);
        return ESP_ERR_NOT_SUPPORTED;
    }
    uvc_host_frame_info_t *fi = &s_u.frame_info[idx];

    int out_w = passthrough ? fi->h_res : (s_u.req_w ? s_u.req_w : fi->h_res);
    int out_h = passthrough ? fi->v_res : (s_u.req_h ? s_u.req_h : fi->v_res);
    s_u.w = fi->h_res;
    s_u.h = fi->v_res;
    s_u.out_w = out_w;
    s_u.out_h = out_h;
    s_u.dec_src_mjpeg = !passthrough && src_mjpeg;

    uvc_host_stream_config_t cfg = {
        .event_cb = stream_event_cb,
        .frame_cb = frame_cb,
        .user_ctx = NULL,
        .usb = { .dev_addr = s_u.dev_addr, .vid = UVC_HOST_ANY_VID, .pid = UVC_HOST_ANY_PID,
                 .uvc_stream_index = stream_idx },
        .vs_format = { .h_res = fi->h_res, .v_res = fi->v_res,
                       .fps = pick_tier_fps(fi),
                       .format = fi->format },
        .advanced = {
            .number_of_frame_buffers = FRAME_BUFFERS,
            .frame_size = 0,
            .frame_heap_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
            .number_of_urbs = NUM_URBS,
            .urb_size = URB_SIZE,
            .user_frame_buffers = NULL,
        },
    };
    uvc_host_stream_hdl_t h = NULL;
    esp_err_t err = uvc_host_stream_open(&cfg, pdMS_TO_TICKS(STREAM_OPEN_TIMEOUT_MS), &h);
    ESP_RETURN_ON_ERROR(err, TAG, "stream_open (%s %ux%u)", fmt_enum_name(fi->format), fi->h_res, fi->v_res);
    s_u.stream = h;

    err = uvc_host_stream_start(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "stream_start failed (%s)", esp_err_to_name(err));
        s_u.worker_run = false;
        uvc_host_stream_stop(h);
        if (safe_close_stream(h)) s_u.stream = NULL;
        return err;
    }

    if (!passthrough) {
        int enc_fps = (int)pick_tier_fps(fi);
        if (enc_fps <= 0) enc_fps = 30;   /* h264 cfg 需显式 fps */
        err = encoder_open(out_w, out_h, enc_fps);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "encoder_open failed");
            s_u.worker_run = false;
            uvc_host_stream_stop(h);
            if (safe_close_stream(h)) s_u.stream = NULL;
            return err;
        }
    }

    s_u.stream_index = stream_idx;
    s_u.w = fi->h_res;
    s_u.h = fi->v_res;
    s_u.out_w = out_w;
    s_u.out_h = out_h;
    s_u.next_due_us = 0;
    (void)needs_mjpeg_pref;
    rebuild_tiers();   /* ★ 必须先于 stream_up=true */
    s_u.stream_up = true;
    s_u.stream_gen++;
    if (s_u.h264) h264_pipeline_force_idr(s_u.h264);   /* 首帧 IDR：浏览器连上即起播 */
    int stale = drain_work_queue(s_u.stream);
    if (stale) ESP_LOGW(TAG, "开流收尾归还 %d 个排队帧", stale);
    s_u.worker_run = true;
    bool scaled = (out_w != fi->h_res || out_h != fi->v_res);
    ESP_LOGI(TAG, "UVC 推流开始（流索引 %u）：%s %ux%u%s%s quality=%u fps_limit=%d mode=%s",
             stream_idx,
             fmt_enum_name(fi->format), fi->h_res, fi->v_res,
             scaled ? " → " : "", scaled ? "ESP32缩放" : "",
             passthrough ? 0 : s_u.quality, s_u.fps_limit,
             s_u.mode == USB_MODE_H264 ? "h264" : s_u.mode == USB_MODE_REENCODE ? "reencode" : "passthrough");
    return ESP_OK;
}

/* ---------- worker：消费帧 → 直通发布 / 硬解+硬编后发布 ---------- */
static void worker_task(void *arg)
{
    uvc_host_frame_t *f;
    for (;;) {
        if (!s_u.worker_run) {
            /* 停止态：teardown 的 close 重试期间，把漏网的竞态帧归还驱动 */
            uvc_host_stream_hdl_t h = s_u.stream;
            while (xQueueReceive(s_u.work_q, &f, 0) == pdTRUE) {
                ret_frame(h, f);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (xQueueReceive(s_u.work_q, &f, pdMS_TO_TICKS(200)) != pdTRUE) continue;
        s_u.worker_busy = true;

        uint64_t t_arr = esp_timer_get_time();
        uvc_host_stream_hdl_t h = s_u.stream;

        /* 软件限帧 */
        if (s_u.fps_limit > 0) {
            if (s_u.next_due_us == 0) s_u.next_due_us = t_arr;
            if (t_arr < s_u.next_due_us) {
                ret_frame(h, f);
                s_u.worker_busy = false;
                continue;
            }
            s_u.next_due_us += 1000000ULL / s_u.fps_limit;
            if ((int64_t)(t_arr - s_u.next_due_us) > 500000) s_u.next_due_us = t_arr;
        }

        if (s_u.mode == USB_MODE_PASSTHROUGH) {
            /* 模式 A：MJPEG 直通 */
            uint64_t t0 = esp_timer_get_time();
            frame_meta_t meta = { .source = FRAME_SRC_USB, .scaled = 0,
                                  .ts_meaning = FRAME_TS_FRAME_ARRIVAL,
                                  .codec = FRAME_CODEC_JPEG };
            bool ok = h && frame_ring_publish_ex(src_if_ring(), f->data, f->data_len,
                                                 t_arr, t_arr, f->vs_format.h_res, f->vs_format.v_res,
                                                 0, &meta);
            uint64_t t1 = esp_timer_get_time();
            s_u.stats.proc_acc_us += t1 - t0;
            if (ok) s_u.stats.out_frames++; else s_u.stats.out_drops++;
            s_u.stats.out_bytes += f->data_len;
            ret_frame(h, f);
        } else {
            /* 模式 B/C：h264 / reencode。输入两类：
             * MJPEG 档（硬解码→UYVY）/ YUY2 档（直接字节序交换，回退路径） */
            const uint8_t *src = f->data;
            uint64_t t_pre = esp_timer_get_time();
            if (s_u.dec_src_mjpeg) {
                if (!s_u.dec_out) {
                    s_u.stats.cap_drops++;
                    ret_frame(h, f);
                    s_u.worker_busy = false;
                    continue;
                }
                jpeg_decode_cfg_t dc = { .output_format = JPEG_DECODE_OUT_FORMAT_YUV422 };
                uint32_t dec_len = 0;
                esp_err_t derr = jpeg_dec_share_process(&dc, f->data, f->data_len,
                                                        s_u.dec_out, s_u.dec_out_len, &dec_len);
                if (derr != ESP_OK) {
                    static uint32_t dec_err;
                    if (++dec_err % 30 == 1)
                        ESP_LOGW(TAG, "jpeg decode 失败 %s（len=%u）— 丢帧",
                                 esp_err_to_name(derr), (unsigned)f->data_len);
                    s_u.stats.cap_drops++;
                    ret_frame(h, f);
                    s_u.worker_busy = false;
                    continue;
                }
                src = s_u.dec_out;
            }

            int kx, ky, x0, y0;
            plan_virtual(s_u.w, s_u.h, s_u.out_w, s_u.out_h, &kx, &ky, &x0, &y0);

            if (s_u.mode == USB_MODE_H264) {
                /* H.264 管线：MJPEG→解码(YUV422/UYVY 序)→[OSD]→软件重排
                 * O_UYY_E_VYY→[PPA 裁剪/缩放(仅 out≠native)]→硬编码 */
                if (!s_u.h264 || !s_u.h264_in || !s_u.dec_out) {
                    s_u.stats.cap_drops++;   /* 编码链路未就绪（开流竞态）：丢弃 */
                    ret_frame(h, f);
                    s_u.worker_busy = false;
                    continue;
                }
                int nw = (s_u.w + 15) & ~15, nh = (s_u.h + 15) & ~15;
                int ew = (s_u.out_w + 15) & ~15, eh = (s_u.out_h + 15) & ~15;
                uint64_t t_dec_us = esp_timer_get_time() - t_pre;
                if (s_u.overlay) {
                    yuv_osd_draw_ms_counter(s_u.dec_out, (size_t)s_u.w * 2,
                                            s_u.w, s_u.h, OSD_FMT_UYVY, t_arr);
                }
                bool scaled = (s_u.out_w != s_u.w || s_u.out_h != s_u.h);
                uint64_t t_re0 = esp_timer_get_time();
                if (!scaled) {
                    /* 原生尺寸：直接重排进编码器输入缓冲 */
                    uyvy_to_ouev(s_u.dec_out, s_u.w, s_u.h, s_u.h264_in);
                } else {
                    /* 先重排为 native 420，再 PPA 裁剪/缩放（rev<3.0 PPA 不收 YUV422） */
                    uyvy_to_ouev(s_u.dec_out, s_u.w, s_u.h, s_u.conv_buf);
                    ppa_srm_oper_config_t op = {
                        .in = {
                            .buffer = s_u.conv_buf,
                            .pic_w = (uint32_t)nw, .pic_h = (uint32_t)nh,
                            .block_w = (uint32_t)s_u.out_w, .block_h = (uint32_t)s_u.out_h,
                            .block_offset_x = (uint32_t)x0,
                            .block_offset_y = (uint32_t)(y0 & ~1),
                            .srm_cm = PPA_SRM_COLOR_MODE_YUV420,
                        },
                        .out = {
                            .buffer = s_u.h264_in,
                            .buffer_size = s_u.h264_in_len,
                            .pic_w = (uint32_t)ew, .pic_h = (uint32_t)eh,
                            .block_offset_x = 0, .block_offset_y = 0,
                            .srm_cm = PPA_SRM_COLOR_MODE_YUV420,
                        },
                        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
                        .scale_x = 1.0f, .scale_y = 1.0f,
                        .mode = PPA_TRANS_MODE_BLOCKING,
                    };
                    esp_err_t perr = ppa_do_scale_rotate_mirror(s_u.ppa, &op);
                    if (perr != ESP_OK) {
                        static uint32_t ppa_err;
                        if (++ppa_err % 30 == 1)
                            ESP_LOGW(TAG, "PPA 缩放失败 %s", esp_err_to_name(perr));
                        s_u.stats.cap_drops++;
                        ret_frame(h, f);
                        s_u.worker_busy = false;
                        continue;
                    }
                }
                uint8_t *out_data;
                size_t out_len;
                bool is_idr;
                uint64_t t_enc0 = esp_timer_get_time();
                esp_err_t eerr = h264_pipeline_encode(s_u.h264, s_u.h264_in, &out_data, &out_len, &is_idr);
                uint64_t t_enc_done = esp_timer_get_time();
                ret_frame(h, f);
                /* 剖面：解码/重排(含PPA)/编码 三段耗时（每 15 帧打印均值） */
                {
                    static uint32_t pn;
                    static uint64_t acc_d, acc_r, acc_e;
                    acc_d += t_dec_us;
                    acc_r += t_enc0 - t_re0;
                    acc_e += t_enc_done - t_enc0;
                    if (++pn >= 15) {
                        ESP_LOGI(TAG, "剖面 ms：解码 %.1f 重排 %.1f H264编码 %.1f",
                                 acc_d / (float)pn / 1000, acc_r / (float)pn / 1000,
                                 acc_e / (float)pn / 1000);
                        pn = 0; acc_d = acc_r = acc_e = 0;
                    }
                }
                if (eerr == ESP_OK && out_len > 0) {
                    frame_meta_t meta = { .source = FRAME_SRC_USB,
                                          .scaled = scaled,
                                          .ts_meaning = FRAME_TS_FRAME_ARRIVAL,
                                          .codec = FRAME_CODEC_H264,
                                          .key = is_idr };
                    bool ok = frame_ring_publish_ex(src_if_ring(), out_data, out_len,
                                                    t_arr, t_enc_done, s_u.out_w, s_u.out_h, 0, &meta);
                    if (ok) s_u.stats.out_frames++; else s_u.stats.out_drops++;
                    s_u.stats.out_bytes += out_len;
                    s_u.stats.proc_acc_us += t_enc_done - t_arr;
                } else {
                    s_u.stats.out_drops++;
                }
                /* JPEG 预览流（与 H264 同源同刻）：无 WebCodecs 的浏览器（Safari/
                 * WKWebView/http 直连的 Chrome——WebCodecs 是 Secure-Context-Only）
                 * 靠它显示画面；有 WebCodecs 的页面按 magic 过滤不渲染。
                 * 统计走 pv_* 独立计数，不污染主指标（out_frames=H264 出帧）。
                 * ★ 自适应让路：相机输出帧率随枚举波动（全速下实测 15~24fps），
                 *   上一帧总处理超 40ms（>24fps 周期的安全余量）时跳过本帧预览，
                 *   保 H264 主链不堆积；空闲时预览全速。
                 * ★ 按需生产：LCD 屏预览关 && 无流客户端时整段跳过
                 *   （src_if_pv_wanted，省 fill+JPEG 编码 ~6-8ms/帧） */
                static uint64_t last_total_us;
                uint64_t total_us = t_enc_done - t_arr;
                if (s_u.jenc && s_u.jenc_in.start && s_u.jenc_out.start &&
                    last_total_us < 40000 && src_if_pv_wanted()) {
                    fill_encoder_input(s_u.jenc_in.start, s_u.dec_out, s_u.w, s_u.h,
                                       s_u.out_w, s_u.out_h, kx, ky, x0, y0,
                                       false, t_arr);   /* false=不交换字节序；OSD 已在 dec_out（重绘同值无害） */
                    jpeg_encode_cfg_t pec = {
                        .width = s_u.out_w,
                        .height = s_u.out_h,
                        .src_type = JPEG_ENCODE_IN_FORMAT_YUV422,
                        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
                        .image_quality = s_u.quality ? s_u.quality : 30,
                        .pixel_reverse = false,
                    };
                    uint32_t psize = 0;
                    esp_err_t perr = jpeg_encoder_process(s_u.jenc, &pec, s_u.jenc_in.start,
                                                          (uint32_t)s_u.out_w * s_u.out_h * 2,
                                                          s_u.jenc_out.start, s_u.jenc_out.length,
                                                          &psize);
                    if (perr == ESP_OK && psize > 0) {
                        frame_meta_t pm = { .source = FRAME_SRC_USB,
                                            .scaled = scaled,
                                            .ts_meaning = FRAME_TS_FRAME_ARRIVAL,
                                            .codec = FRAME_CODEC_JPEG,
                                            .key = 0 };
                        if (frame_ring_publish_ex(src_if_ring(), s_u.jenc_out.start, psize,
                                                  t_arr, esp_timer_get_time(),
                                                  s_u.out_w, s_u.out_h,
                                                  s_u.quality ? s_u.quality : 30, &pm)) {
                            s_u.stats.pv_frames++;
                            s_u.stats.pv_bytes += psize;
                        }
                    }
                    last_total_us = esp_timer_get_time() - t_arr;   /* 含预览的完整耗时 */
                } else {
                    last_total_us = total_us;   /* 跳过预览帧也记录（主链耗时） */
                }
            } else {
                /* reencode：P4 硬件 JPEG 编码器 */
                if (!s_u.jenc || !s_u.jenc_in.start) {
                    s_u.stats.cap_drops++;
                    ret_frame(h, f);
                    s_u.worker_busy = false;
                    continue;
                }
                fill_encoder_input(s_u.jenc_in.start, src, s_u.w, s_u.h, s_u.out_w, s_u.out_h,
                                   kx, ky, x0, y0, !s_u.dec_src_mjpeg, t_arr);
                jpeg_encode_cfg_t ec = {
                    .width = s_u.out_w,
                    .height = s_u.out_h,
                    .src_type = JPEG_ENCODE_IN_FORMAT_YUV422,
                    .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
                    .image_quality = s_u.quality,
                    .pixel_reverse = false,
                };
                uint32_t out_size = 0;
                esp_err_t eerr = jpeg_encoder_process(s_u.jenc, &ec, s_u.jenc_in.start,
                                                      s_u.out_w * s_u.out_h * 2,
                                                      s_u.jenc_out.start, s_u.jenc_out.length,
                                                      &out_size);
                ret_frame(h, f);
                if (eerr == ESP_OK && out_size > 0) {
                    uint64_t t_enc = esp_timer_get_time();
                    frame_meta_t meta = { .source = FRAME_SRC_USB,
                                          .scaled = (s_u.out_w != s_u.w || s_u.out_h != s_u.h),
                                          .ts_meaning = FRAME_TS_FRAME_ARRIVAL,
                                          .codec = FRAME_CODEC_JPEG };
                    bool ok = frame_ring_publish_ex(src_if_ring(), s_u.jenc_out.start, out_size,
                                                    t_arr, t_enc, s_u.out_w, s_u.out_h,
                                                    s_u.quality, &meta);
                    if (ok) s_u.stats.out_frames++; else s_u.stats.out_drops++;
                    s_u.stats.out_bytes += out_size;
                    s_u.stats.proc_acc_us += t_enc - t_arr;
                } else {
                    static uint32_t jenc_err;
                    if (++jenc_err % 30 == 1)
                        ESP_LOGW(TAG, "jpeg encode 失败 %s", esp_err_to_name(eerr));
                    s_u.stats.out_drops++;
                }
            }
        }
        s_u.worker_busy = false;
    }
}

/* lib 事件泵：必须独占常驻（S31 坑 #2：并进状态机任务会被 stream_open 阻塞冻结全总线） */
static void usb_lib_task(void *arg)
{
    uint32_t flags;
    while (1) {
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &flags);
        static uint64_t last_hb;
        uint64_t now = esp_timer_get_time();
        if (now - last_hb > 5000000) {
            ESP_LOGD(TAG, "usb_lib 心跳 err=%s", esp_err_to_name(err));
            last_hb = now;
        }
    }
}

/* ---------- monitor：client 事件派发 + 状态机 + 自动重连 ---------- */
static void monitor_task(void *arg)
{
    static int64_t open_fail_since;
    for (;;) {
        usb_host_client_handle_events(s_u.client, 0);
        static uint64_t last_hb;
        if (esp_timer_get_time() - last_hb > 5000000) {
            ESP_LOGI(TAG, "monitor 心跳：dev=%d stream=%d want=%d cap=%u",
                     (int)s_u.dev_present, (int)s_u.stream_up, (int)s_u.want_stream,
                     (unsigned)s_u.stats.cap_frames);
            last_hb = esp_timer_get_time();
        }

        /* ★ 帧停滞看门狗（S31 坑 #10） */
        {
            static uint32_t last_gen, last_frames;
            static int64_t  last_progress_us;
            static uint8_t  stall_rounds;
            uint32_t gen = s_u.stream_gen;
            if (gen != last_gen) {
                last_gen = gen;
                last_frames = s_u.stats.cap_frames;
                last_progress_us = esp_timer_get_time();
            } else if (s_u.stream_up && s_u.want_stream) {
                uint32_t frames = s_u.stats.cap_frames;
                int64_t now = esp_timer_get_time();
                if (frames != last_frames) {
                    last_frames = frames;
                    last_progress_us = now;
                    stall_rounds = 0;
                } else if (now - last_progress_us > STALL_TIMEOUT_US) {
                    stall_rounds++;
                    ESP_LOGE(TAG, "帧停滞看门狗：%lld s 无新帧 → teardown 重开（第 %u/%u 轮；"
                             "held=%d nohandle=%u open_err=%u disc=%u ovr=%u）",
                             (long long)((now - last_progress_us) / 1000000),
                             (unsigned)stall_rounds, (unsigned)STALL_MAX_ROUNDS,
                             s_u.frames_held, (unsigned)s_u.cb_nohandle,
                             (unsigned)s_u.open_errors, (unsigned)s_u.disconnects,
                             (unsigned)s_u.overflow_events);
                    last_progress_us = now;
                    if (stall_rounds >= STALL_MAX_ROUNDS) {
                        ESP_LOGE(TAG, "连续 %u 轮重开仍无帧（驱动/总线楔死），重启系统兜底",
                                 (unsigned)stall_rounds);
                        esp_restart();
                    }
                    s_u.teardown_req = true;
                }
            }
        }

        /* 清理请求 / 自愈 / close 重试：条件必须锁内重新求值（S31 坑 #9） */
        bool urgent = s_u.teardown_req || !s_u.want_stream;
        if (s_u.teardown_req || (!s_u.want_stream && s_u.stream) ||
            (s_u.stream && !s_u.stream_up)) {
            if (urgent || esp_timer_get_time() >= s_u.next_retry_us) {
                xSemaphoreTake(s_u.lock, portMAX_DELAY);
                bool need = s_u.teardown_req || (!s_u.want_stream && s_u.stream) ||
                            (s_u.stream && !s_u.stream_up);
                s_u.teardown_req = false;
                if (need) {
                    ESP_LOGW(TAG, "monitor：teardown（want=%d up=%d）",
                             (int)s_u.want_stream, (int)s_u.stream_up);
                    teardown_stream();
                }
                xSemaphoreGive(s_u.lock);
                s_u.next_retry_us = esp_timer_get_time() + RETRY_BACKOFF_MS * 1000;
            }
        }

        /* 自动（重）连接 */
        if (!s_u.dev_present || !s_u.want_stream || s_u.stream_up) open_fail_since = 0;
        if (s_u.want_stream && s_u.dev_present && !s_u.stream_up && !s_u.teardown_req && !s_u.stream) {
            int64_t now = esp_timer_get_time();
            if (now >= s_u.next_retry_us) {
                xSemaphoreTake(s_u.lock, portMAX_DELAY);
                esp_err_t err = open_stream_locked();
                xSemaphoreGive(s_u.lock);
                if (err != ESP_OK) {
                    s_u.open_errors++;
                    if (!open_fail_since) open_fail_since = now;
                    else if (now - open_fail_since > OPEN_FAIL_REBOOT_US) {
                        ESP_LOGE(TAG, "连续 %lld s 打开失败仍不愈（dev=1 want=1 held=%d），"
                                 "重启由 USB 重枚举自愈",
                                 (long long)((now - open_fail_since) / 1000000), s_u.frames_held);
                        esp_restart();
                    }
                    s_u.next_retry_us = now + RETRY_BACKOFF_MS * 1000;
                    ESP_LOGW(TAG, "UVC 打开失败（%s），%d s 后重试；若反复失败请检查 USB 供电",
                             esp_err_to_name(err), RETRY_BACKOFF_MS / 1000);
                } else {
                    open_fail_since = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------- 公共 API ---------- */
esp_err_t source_usb_init(void)
{
    memset(&s_u, 0, sizeof(s_u));
    s_u.mode = USB_MODE_H264;
#if defined(CONFIG_CAMTEST_USB_DEFAULT_MODE_PASSTHROUGH)
    s_u.mode = USB_MODE_PASSTHROUGH;
#elif defined(CONFIG_CAMTEST_USB_DEFAULT_MODE_REENCODE)
    s_u.mode = USB_MODE_REENCODE;
#endif
    s_u.quality = 20;
    s_u.h264_kbps = CONFIG_CAMTEST_H264_BITRATE_KBPS;
    s_u.h264_gop = CONFIG_CAMTEST_H264_GOP;
    s_u.fps_limit = 0;
    s_u.inherent_ms = -1.0f;
    s_u.lock = xSemaphoreCreateMutex();
    s_u.work_q = xQueueCreate(WORK_QUEUE_LEN, sizeof(uvc_host_frame_t *));
    ESP_RETURN_ON_FALSE(s_u.lock && s_u.work_q, ESP_ERR_NO_MEM, TAG, "sync");

    /* PPA SRM 客户端（h264 模式：UYVY→O_UYY_E_VYY＋缩放；worker 独占） */
    ppa_client_config_t pcfg = { .oper_type = PPA_OPERATION_SRM };
    ESP_RETURN_ON_ERROR(ppa_register_client(&pcfg, &s_u.ppa), TAG, "ppa_register");

    /* USB Host 库（Tab5 Type-A 直连 P4 OTG HS，内置 HS PHY） */
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
        .xCoreID = 1,
        .create_background_task = true,
        .event_cb = drv_event_cb,
        .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(uvc_host_install(&drv_config), TAG, "uvc_host_install");

    if (xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 3072, NULL, 6, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(worker_task, "usb_work", 6144, NULL, 13, NULL, 1) != pdPASS)
        return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(monitor_task, "usb_mon", 4096, NULL, 4, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;

    s_u.inited = true;

    /* 开机已插入的设备不会补发 NEW_DEV 事件，主动补扫（给枚举一点时间） */
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
        ESP_LOGI(TAG, "USB Host 就绪，未发现 UVC 设备（热插拔监听中）");
    }
    return ESP_OK;
}

void source_usb_set_active(bool active)
{
    s_u.want_stream = active;
    if (!active && s_u.stream_up) {
        s_u.teardown_req = true;
        int wait = 0;
        while (s_u.stream_up && wait++ < 100) vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (active) s_u.next_retry_us = 0;
}

esp_err_t source_usb_start(void)
{
    if (!s_u.inited) return ESP_ERR_INVALID_STATE;
    /* ★ 先置 want 再查设备：Tab5 以 USB 为主源，开机枚举竞态（src_if_init 补扫
     * 可能早于摄像头就绪）不能让 want 卡 0——置位后 monitor 每秒重试，
     * 设备到达/插入即自动开流（S31 语义：热插拔自动重连） */
    source_usb_set_active(true);
    if (!s_u.dev_present) {
        ESP_LOGW(TAG, "USB 摄像头未连接（want 已置位，插入后自动开流）");
        return ESP_ERR_NOT_FOUND;
    }
    for (int i = 0; i < 150 && !s_u.stream_up; i++) {
        if (!s_u.dev_present) return ESP_ERR_NOT_FOUND;
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
        } else if (s_u.mode == USB_MODE_H264) {
            ESP_LOGW(TAG, "h264 模式画质由码率决定（target_mbps/h264_kbps），quality=%u 忽略", quality);
        } else if (quality != s_u.quality) {
            s_u.quality = quality;
            /* JPEG 引擎 quality 在每次 process 的 cfg 里带（无需持久控件） */
        }
    }
    if (fps_limit >= 0) s_u.fps_limit = fps_limit;

    bool res_change = (w && h) && (w != s_u.out_w || h != s_u.out_h);
    if (res_change) {
        s_u.want_stream = false;
        s_u.teardown_req = false;
        xSemaphoreTake(s_u.lock, portMAX_DELAY);
        teardown_stream();
        s_u.req_w = w;
        s_u.req_h = h;
        esp_err_t err = ESP_OK;
        if (s_u.dev_present) {
            err = open_stream_locked();
        } else {
            err = ESP_ERR_NOT_FOUND;
        }
        s_u.want_stream = true;   /* ★ 锁内恢复，防 monitor 误杀新流 */
        xSemaphoreGive(s_u.lock);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t source_usb_set_quality(uint8_t q)
{
    if (s_u.mode == USB_MODE_PASSTHROUGH || s_u.mode == USB_MODE_H264)
        return ESP_ERR_NOT_SUPPORTED;
    s_u.quality = q;
    return ESP_OK;
}

void source_usb_set_fps_limit(int fps)
{
    s_u.fps_limit = fps > 0 ? fps : 0;
}

void source_usb_set_overlay(bool on)
{
    if (on && s_u.mode == USB_MODE_PASSTHROUGH) {
        ESP_LOGW(TAG, "passthrough 模式不解码，无法叠加毫秒计数器；已由配置层自动切 h264");
    }
    s_u.overlay = on;
}

bool source_usb_overlay(void) { return s_u.overlay; }

esp_err_t source_usb_h264_set_bitrate_kbps(uint32_t kbps)
{
    s_u.h264_kbps = kbps;
    if (s_u.h264) return h264_pipeline_set_bitrate(s_u.h264, kbps * 1000);
    return ESP_OK;
}

uint32_t source_usb_h264_kbps(void) { return s_u.h264_kbps; }

src_info_t *source_usb_info(void)
{
    static src_info_t info;
    memset(&info, 0, sizeof(info));
    info.source = VIDEO_SOURCE_USB;
    info.needs_encode = (s_u.mode != USB_MODE_PASSTHROUGH);
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
    strlcpy(info.pix_fmt_str,
            s_u.mode == USB_MODE_PASSTHROUGH ? "JPEG" :
            s_u.mode == USB_MODE_H264 ?
                (s_u.dec_src_mjpeg ? "MJPEG>H264" : "YUY2>H264") :
                (s_u.dec_src_mjpeg ? "MJPEG>JPEG" : "YUY2>JPEG"),
            sizeof(info.pix_fmt_str));
    snprintf(info.fmt_name, sizeof(info.fmt_name), "%s%s%ux%u",
             s_u.mode == USB_MODE_PASSTHROUGH ? "MJPEG直通" :
             s_u.mode == USB_MODE_H264 ? "H.264硬编码" : "JPEG重编码",
             info.scaled ? "缩放" : "", info.w, info.h);
    return &info;
}

src_stats_t *source_usb_stats(void) { return &s_u.stats; }

frame_ring_t *source_usb_ring(void) { return src_if_ring(); }

int source_usb_supported_res(char *out, size_t outlen)
{
    xSemaphoreTake(s_u.lock, portMAX_DELAY);
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
    if (!passthrough) {
        static const int tiers[][2] = { {320,240},{640,480},{1280,720} };
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
    }
    xSemaphoreGive(s_u.lock);
    return n;
}

bool source_usb_res_supported(int w, int h)
{
    xSemaphoreTake(s_u.lock, portMAX_DELAY);
    for (int i = 0; i < s_u.tier_n; i++) {
        usb_tier_t *t = &s_u.tiers[i];
        if (s_u.mode == USB_MODE_PASSTHROUGH) {
            if (t->mjpeg && t->w == w && t->h == h) {
                xSemaphoreGive(s_u.lock);
                return true;
            }
        } else {
            int kx, ky, x0, y0;
            if (t->w == w && t->h == h) {
                xSemaphoreGive(s_u.lock);
                return true;
            }
            if (plan_virtual(t->w, t->h, w, h, &kx, &ky, &x0, &y0)) {
                xSemaphoreGive(s_u.lock);
                return true;
            }
        }
    }
    xSemaphoreGive(s_u.lock);
    return false;
}

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
    if (m != USB_MODE_PASSTHROUGH && m != USB_MODE_REENCODE && m != USB_MODE_H264)
        return ESP_ERR_INVALID_ARG;
    if (m == s_u.mode) return ESP_OK;
    bool was_active = s_u.want_stream;
    usb_mode_t old = s_u.mode;
    if (was_active && s_u.dev_present) {
        s_u.want_stream = false;
        s_u.teardown_req = false;
        xSemaphoreTake(s_u.lock, portMAX_DELAY);
        teardown_stream();
        s_u.mode = m;   /* ★ 旧流完全停手后再切换 */
        esp_err_t err = open_stream_locked();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "切换到 %s 失败（%s），回滚 %s",
                     src_if_usb_mode_name(m), esp_err_to_name(err), src_if_usb_mode_name(old));
            s_u.mode = old;
            err = open_stream_locked();
        }
        s_u.want_stream = true;
        xSemaphoreGive(s_u.lock);
        return err;
    }
    s_u.mode = m;
    return ESP_OK;
}

uint32_t source_usb_disconnects(void) { return s_u.disconnects; }
const char *source_usb_device_name(void) { return s_u.dev_name; }

int source_usb_get_tiers(usb_tier_t *out, int max)
{
    xSemaphoreTake(s_u.lock, portMAX_DELAY);
    int n = s_u.tier_n < max ? s_u.tier_n : max;
    memcpy(out, s_u.tiers, n * sizeof(usb_tier_t));
    xSemaphoreGive(s_u.lock);
    return n;
}

float source_usb_inherent_ms(void) { return s_u.inherent_ms; }
void source_usb_set_inherent_ms(float ms) { s_u.inherent_ms = ms; }

#endif /* CONFIG_CAMTEST_ENABLE_USB */
