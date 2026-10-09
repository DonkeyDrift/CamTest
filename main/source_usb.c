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
#include "esp_system.h"     /* esp_restart：帧停滞看门狗的重启兜底 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"    /* VIDIOC_S_DQBUF_TIMEOUT（esp_video 自定义 ioctl） */
#include "driver/jpeg_decode.h" /* S31 硬件 JPEG 解码：MJPEG 档重编码路径（YUY2 档只有 10fps） */
#include "camera_pipeline.h"     /* cam_pipe_ring()：共享帧环；编码器设备号约定 */
#include "scan_ctrl.h"           /* scan_ctrl_state()：扫描期间抑制 esp_restart（毁整轮矩阵） */
#include "yuv_osd.h"
#include "sdkconfig.h"

#if CONFIG_CAMTEST_ENABLE_USB

static const char *TAG = "src_usb";

#define USB_DEV_ANY          0
#define FRAME_BUFFERS        4      /* uvc 驱动帧缓冲数（按协商 dwMaxVideoFrameSize 自分配 PSRAM；4 个平滑 60fps 突发） */
/* URB 在途配置降档链：真机 2026-10-09 DVP→USB 切换 NO_MEM 根因——LCD/WiFi 运行后
 * 内部 DMA 总空闲从开机 131KB 降到 ~56KB（最大块 31.7KB），72KB 总量的 8×9216 凑不齐，
 * uvc_transfers_allocate 报 NO_MEM 被迫回滚（开机首开正常、运行期重开必失败；
 * urb_alloc 每 URB 是一块独立 heap_caps_malloc(DMA|INTERNAL)，受总空闲和最大块双重约束）。
 * urb_size 会被驱动 usb_round_up_to_mps 向上取整到 MPS 整数倍（本机 MPS=3072），
 * 档位直接取 3072 倍数避免取整膨胀（如 4608 会被抬成 6144 反而多占）。
 * 在途覆盖 =（URB数−1）×每URB微帧数×125µs，须高于 1.5ms 失败点（4×9216 时仅
 * 1.5ms 即丢包，FID 错→整帧弃）。档 1/2 与档 0 同为 72KB 总量、只缩单块尺寸
 * （对纯碎片化场景兜底），档 3 缩总量到 48KB（对总空闲不足——真机实际命中的档）。
 * 每 URB 越小 resubmit 频率越高，但 60fps 实测每帧 ~10 个完成中断，负担可忽略 */
typedef struct { uint16_t size; uint8_t n; } urb_cfg_t;
static const urb_cfg_t URB_TIERS[] = {
    { 9216, 8 },    /* 3 微帧/URB：覆盖（8−1）×3×125µs=2.625ms，比 8×12288 省 24.6KB 内部 DMA
                         * ——LCD 面板链表/任务栈与 UVC 在途缓冲共存的预算关键（原单档） */
    { 6144, 12 },   /* 2 微帧/URB：覆盖（12−1）×2×125µs=2.75ms */
    { 3072, 24 },   /* 1 微帧/URB：覆盖（24−1)×1×125µs=2.875ms */
    { 3072, 16 },   /* 末档兜底：总量 48KB，覆盖 1.875ms（仍高于 1.5ms 失败点） */
};
#define URB_TIER_CNT   ((int)(sizeof(URB_TIERS) / sizeof(URB_TIERS[0])))
#define WORK_QUEUE_LEN       4      /* 与 FRAME_BUFFERS 对齐（深于驱动缓冲数无意义） */
#define STREAM_OPEN_TIMEOUT_MS 5000
#define RETRY_BACKOFF_MS     1000   /* ERROR 态重试间隔（供电不足场景别疯狂重试） */
/* 帧停滞看门狗：uvc 驱动存在静默停流路径（isoc 包 CANCELED/NO_DEVICE →
 * uvc_host_stream_pause 直接置 streaming=false：不回调任何事件、不打日志、不再
 * resubmit URB），停后流仍报 stream_up=1 而帧永久为零。真机 2026-10-08 实测：
 * 60fps 稳跑 1230s 后无任何错误日志归零，20+ 分钟不自愈，页面打开只见黑屏。
 * 监测：stream_up && want_stream 但 cap_frames 停滞 → 强制 teardown 走既有
 * teardown→reopen 自愈链路；连续 STALL_MAX_ROUNDS 轮重开仍无帧（驱动/总线
 * 楔死）→ esp_restart 兜底（与 main.c cam_watch 的 DVP 看门狗同语义） */
#define STALL_TIMEOUT_US     (5 * 1000000)
#define STALL_MAX_ROUNDS     6
/* 连续打开失败满 60 s → 重启：want+dev 恒真而 open 永久失败 = 设备/总线楔死
 * （真机实测：静默停流后 teardown 的 CTRL timeout 使驱动 close 簿记不一致 →
 * get_frame_list 永久 INVALID_ARG，1 s 重试永不愈），只有重启触发 USB 重枚举
 * （端口复位）才能自愈——实测复位后同一摄像头立即恢复推流 */
#define OPEN_FAIL_REBOOT_US  (60 * 1000000)
#define ENC_IN_BUFS          2
#define ENC_OUT_BUFS         3
#define ENC_DQBUF_TIMEOUT_MS 200

/* ---------- 私有状态（usb_state_t 已被 source_if.h 的对外状态枚举占用） ---------- */
struct usb_priv_s {
    /* USB 总线层 */
    usb_host_client_handle_t client;
    volatile bool dev_present;
    uint8_t  dev_addr;
    uint8_t  stream_index;                  /* 当前尝试的 UVC function */
    uint8_t  idx_candidates[8];             /* 部分摄像头暴露多个 UVC function
                                               （如 0bda:1376 双通道，仅其一响应 VS Probe） */
    int      idx_cand_n;
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
    volatile uint32_t stream_gen;                 /* 成功开流代数（帧停滞看门狗据此给首帧宽限期） */

    /* 编码器（仅 reencode；/dev/video10 与 DVP 同一台，两源互斥运行） */
    int enc_fd;
    struct { uint8_t *start; size_t length; } enc_in[ENC_IN_BUFS], enc_out[ENC_OUT_BUFS];
    QueueHandle_t enc_free_in;

    /* MJPEG 硬解码（reencode 且当前流为 MJPEG 档）：JPEG→UYVY 直写解码缓冲，
     * 解码/编码为独立引擎实例、worker 内串行使用。0bda:1376 的 YUY2 全档只有
     * 10fps（描述符上限），MJPEG 档才有 60fps——带计数器的重编码必须走此路径 */
    jpeg_decoder_handle_t dec;
    uint8_t *dec_out;                           /* 解码输出（native 尺寸 UYVY，jpeg_alloc_decoder_mem） */
    size_t dec_out_len;
    bool dec_src_mjpeg;                         /* 当前流是否 MJPEG 档（开流时确定） */

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
    volatile int frames_held;                   /* frame_cb 交出、尚未归还驱动的帧数（0 才能 close） */
    volatile uint32_t cb_nohandle;              /* frame_cb 无句柄防御路径触发次数（应恒为 0） */

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

/* → 编码器输入（UYVY）：整数抽取/中心裁剪 + 可选字节序交换 + 可选 OSD。
 * swap_bytes：YUY2 摄像头源需交换为 UYVY；MJPEG 硬解码输出已是 UYVY，传 false 免一次全帧交换 */
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

    /* 探测 UVC 功能：收集全部含档位表的流索引作候选（部分摄像头第一个 function
     * 是不响应 VS Probe 控制请求的哑通道，打开时逐个尝试） */
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
	case UVC_HOST_DEVICE_SUSPENDED:
		/* 驱动收到挂起已静默 pause（帧停、零日志）——若落进 default 忽略，
		 * stream_up 会卡 1 成死态；置 teardown 让 monitor 关流，RESUMED 后
		 * 按 want_stream 自动重开 */
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
 * （"Not all frames are returned" → 流对象泄漏占住接口 → 之后 open 永久
 * INVALID_ARG），所以统一从此走并记账，close 前 frames_held 必须归零 */
static inline void ret_frame(uvc_host_stream_hdl_t h, uvc_host_frame_t *f)
{
    if (h) {
        uvc_host_frame_return(h, f);
    } else {
        s_u.cb_nohandle++;   /* 无句柄可归还：禁止静默，计数暴露 */
    }
    if (s_u.frames_held > 0) s_u.frames_held--;
}

static bool frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    /* ★ U4 打点：完整一帧到达 ESP32 的时刻（含摄像头内部曝光/ISP/编码/USB 传输延迟） */
    s_u.stats.cap_frames++;
    uvc_host_stream_hdl_t h = s_u.stream;
    if (!h) {
        /* 防御：驱动只在流存在时回调。真发生则帧无法归还——计数暴露勿静默 */
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
    for (int i = 0; i < ENC_IN_BUFS; i++) {
        if (s_u.enc_in[i].start) {
            heap_caps_free(s_u.enc_in[i].start);   /* USERPTR：自有缓冲自行释放 */
            s_u.enc_in[i].start = NULL;
            s_u.enc_in[i].length = 0;
        }
    }
    if (s_u.dec) {
        jpeg_del_decoder_engine(s_u.dec);
        s_u.dec = NULL;
    }
    if (s_u.dec_out) {
        heap_caps_free(s_u.dec_out);               /* jpeg_alloc_decoder_mem 配对释放 */
        s_u.dec_out = NULL;
        s_u.dec_out_len = 0;
    }
}

static esp_err_t encoder_open(int w, int h, uint8_t quality)
{
    ESP_LOGI(TAG, "encoder_open: %dx%d q%u（/dev/video10%s）", w, h, quality,
             s_u.dec_src_mjpeg ? "，MJPEG 输入＋硬解码" : "");
    if (s_u.enc_fd >= 0) encoder_close();   /* 重入防护：上次 teardown 时 worker 忙未关 */
    if (s_u.dec_src_mjpeg) {
        /* 解码引擎先建：JPEG→UYVY 输出直写 SPIRAM（2D-DMA 支 PSRAM，驱动内部做 cache 维护）。
         * 输出尺寸按 JPEG 协议 16 对齐向上取整；与编码引擎为独立实例，worker 内串行无争用 */
        jpeg_decode_engine_cfg_t ecfg = { .intr_priority = 0, .timeout_ms = ENC_DQBUF_TIMEOUT_MS };
        esp_err_t derr = jpeg_new_decoder_engine(&ecfg, &s_u.dec);
        if (derr != ESP_OK) {
            ESP_LOGE(TAG, "jpeg 解码引擎创建失败 %s", esp_err_to_name(derr));
            return ESP_FAIL;
        }
        int nw = (s_u.w + 15) & ~15, nh = (s_u.h + 15) & ~15;   /* native 尺寸（开流时已置） */
        jpeg_decode_memory_alloc_cfg_t mcfg = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
        s_u.dec_out = jpeg_alloc_decoder_mem((size_t)nw * nh * 2, &mcfg, &s_u.dec_out_len);
        if (!s_u.dec_out) {
            ESP_LOGE(TAG, "解码输出缓冲分配失败（%dx%d UYVY）", nw, nh);
            encoder_close();
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "MJPEG 硬解码就绪：%dx%d→UYVY（缓冲 %u KB）", s_u.w, s_u.h,
                 (unsigned)(s_u.dec_out_len / 1024));
    }
    s_u.enc_fd = open(ESP_VIDEO_JPEG_DEVICE_NAME, O_RDWR);   /* /dev/video10 */
    ESP_RETURN_ON_FALSE(s_u.enc_fd >= 0, ESP_FAIL, TAG, "open %s", ESP_VIDEO_JPEG_DEVICE_NAME);
    struct timeval tv = { .tv_sec = 0, .tv_usec = ENC_DQBUF_TIMEOUT_MS * 1000 };
    xioctl(s_u.enc_fd, VIDIOC_S_DQBUF_TIMEOUT, &tv);

    struct v4l2_format fo = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
                              .fmt.pix.width = w, .fmt.pix.height = h,
                              .fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY };
    if (xioctl(s_u.enc_fd, VIDIOC_S_FMT, &fo) != 0) {
        ESP_LOGE(TAG, "enc S_FMT OUTPUT 失败 errno=%d", errno);
        goto fail;
    }
    struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                              .fmt.pix.width = w, .fmt.pix.height = h,
                              .fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG };
    if (xioctl(s_u.enc_fd, VIDIOC_S_FMT, &fc) != 0) {
        ESP_LOGE(TAG, "enc S_FMT CAPTURE 失败 errno=%d", errno);
        goto fail;
    }
    /* 不发 CHROMA_SUBSAMPLING 控件（esp_video 2.2.0 HAL 断言坑，见 API_NOTES.md） */
    struct v4l2_ext_control ctrl = { .id = V4L2_CID_JPEG_COMPRESSION_QUALITY, .value = quality };
    struct v4l2_ext_controls ctrls = { .count = 1, .controls = &ctrl };
    if (xioctl(s_u.enc_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
        ESP_LOGE(TAG, "enc quality 失败 errno=%d", errno);
        goto fail;
    }

    struct v4l2_requestbuffers rq = { .count = ENC_IN_BUFS, .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_USERPTR };
    if (xioctl(s_u.enc_fd, VIDIOC_REQBUFS, &rq) != 0) {
        ESP_LOGE(TAG, "enc REQBUFS OUT 失败 errno=%d", errno);
        goto fail;
    }
    struct v4l2_requestbuffers rc = { .count = ENC_OUT_BUFS, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    if (xioctl(s_u.enc_fd, VIDIOC_REQBUFS, &rc) != 0) {
        ESP_LOGE(TAG, "enc REQBUFS CAP 失败 errno=%d", errno);
        goto fail;
    }

    xQueueReset(s_u.enc_free_in);
    for (int i = 0; i < ENC_IN_BUFS; i++) {
        /* USERPTR：自有 PSRAM 缓冲（esp_video uvc 示例同款）。对齐 4096 + 尾部余量，
         * 覆盖 jpeg 引擎 info->align_size / info->size 的内部对齐要求 */
        s_u.enc_in[i].start = heap_caps_aligned_alloc(4096, (size_t)w * h * 2 + 4096,
                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_u.enc_in[i].start) {
            /* 常见于多客户端运行后碎片化：空闲总量够但最大连续块不够，与供电无关 */
            ESP_LOGE(TAG, "enc IN[%d] PSRAM 分配失败：重编码 %dx%d 需 %zu KB 连续块×%d，"
                          "请改用 ≤640x480 重编码或 passthrough",
                     i, w, h, ((size_t)w * h * 2 + 4096) / 1024, ENC_IN_BUFS);
            goto fail;
        }
        s_u.enc_in[i].length = (size_t)w * h * 2 + 4096;
        xQueueSend(s_u.enc_free_in, &i, 0);
    }
    for (int i = 0; i < ENC_OUT_BUFS; i++) {
        struct v4l2_buffer buf = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i };
        if (xioctl(s_u.enc_fd, VIDIOC_QUERYBUF, &buf) != 0 ||
            (s_u.enc_out[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                         s_u.enc_fd, buf.m.offset)) == MAP_FAILED) {
            ESP_LOGE(TAG, "enc OUT[%d] QUERYBUF/mmap 失败 errno=%d", i, errno);
            goto fail;
        }
        s_u.enc_out[i].length = buf.length;
        xioctl(s_u.enc_fd, VIDIOC_QBUF, &buf);
    }
    uint32_t t_out = V4L2_BUF_TYPE_VIDEO_OUTPUT, t_cap = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(s_u.enc_fd, VIDIOC_STREAMON, &t_out) != 0 ||
        xioctl(s_u.enc_fd, VIDIOC_STREAMON, &t_cap) != 0) {
        ESP_LOGE(TAG, "enc STREAMON 失败 errno=%d", errno);
        goto fail;
    }
    ESP_LOGI(TAG, "encoder_open OK");
    return ESP_OK;
fail:
    encoder_close();
    return ESP_FAIL;
}

/* ---------- 档位选择：返回 frame_info 下标；-1 = 不支持 ----------
 * want_mjpeg：只看 MJPEG 档；scalable_ok：允许"大原生档→缩放/裁剪到目标"（直通必须原生精确，
 * 重编码在解码后缩放，MJPEG/YUY2 档都允许）。reencode 优先 MJPEG 档——0bda:1376 的
 * YUY2 全档只有 10fps（描述符上限），MJPEG 档才有 60fps，硬解码后帧率不再受限 */
static int pick_native(bool want_mjpeg, int want_w, int want_h, bool scalable_ok)
{
    int fallback = -1;      /* 未指定分辨率时的默认档 */
    int fallback_640 = -1;  /* 未指定时优先 640x480（与 DVP 对比口径一致，PSRAM 友好） */
    int scalable = -1;      /* 可缩放达标的档 */
    for (size_t i = 0; i < s_u.frame_info_n; i++) {
        uvc_host_frame_info_t *fi = &s_u.frame_info[i];
        if (want_mjpeg && fi->format != UVC_VS_FORMAT_MJPEG) continue;
        if (!want_mjpeg && fi->format != UVC_VS_FORMAT_YUY2) continue;
        if (want_w <= 0 || want_h <= 0) {
            if (fallback < 0) fallback = i;
            if (fi->h_res == 640 && fi->v_res == 480) fallback_640 = i;
            continue;
        }
        if (fi->h_res == want_w && fi->v_res == want_h) return i;   /* 原生精确优先 */
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

static esp_err_t try_open_stream_on_index(uint8_t stream_idx);   /* 定义于 open_stream_locked 之后 */

/* ---------- 清理：顺序敏感（先暂停流 → 归还全部帧 → 关流 → 关编码器） ----------
 * ★ s_u.stream 必须在 close 成功之后才能置 NULL：worker 的停止态排水循环用
 *   s_u.stream 归还漏网帧（pause 前在途回调的竞态帧）。提前置 NULL 会让 worker
 *   丢弃该帧 → 驱动 "Not all frames are returned" → close 永久失败 → 流泄漏
 *   → 接口被占 → 之后所有 get_frame_list/open 永久失败（真机实测 2026-10-08）。 */
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

/* 安全关流：多轮 drain+close 直到成功（帧不归还时 close 报 INVALID_STATE 且不释放流对象；
 * 放弃重试 = 流永久泄漏占住接口 → 之后所有 get_frame_list/open 报 INVALID_ARG）。
 * 调用前须已 worker_run=false（worker 排水循环会归还漏网的竞态帧）。 */
static bool safe_close_stream(uvc_host_stream_hdl_t h)
{
    if (!h) return true;
    for (int i = 0; i < 20; i++) {
        int leaked = drain_work_queue(h);
        if (leaked) {
            ESP_LOGW(TAG, "safe_close：第 %d 轮追回 %d 个竞态帧", i, leaked);
            vTaskDelay(pdMS_TO_TICKS(30));   /* 给 worker 排水循环时间再捞一轮 */
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
        /* 先暂停（无新帧、无回调）。活设备走控制传输；死设备报错——两种情况都忽略返回值，
         * 断线场景 uvc_host 的 DEV_GONE 处理里已经 pause 过 */
        uvc_host_stream_stop(h);
        /* 等 worker 处理完手中帧（含编码，最长约 50 ms；编码 DQBUF 有 200 ms 超时兜底）。
         * 先等 busy 再 drain：worker 手中帧归还路径用的是它自己取的 h，与队列无关 */
        int wait = 0;
        while (s_u.worker_busy && wait++ < 100) vTaskDelay(pdMS_TO_TICKS(10));
        bool worker_idle = !s_u.worker_busy;

        s_u.stream_up = false;
        if (safe_close_stream(h)) {
            s_u.stream = NULL;
        } else {
            /* ★ close 失败时【保留句柄】：worker/迟到回调的漏网帧仍可归还，monitor 周期
             * 重试 close，凑齐即恢复。此前置 NULL 会把漏网帧锁死 → 永久楔死（真机实测） */
            ESP_LOGE(TAG, "close 未完成，保留流句柄（held=%d），monitor 将重试", s_u.frames_held);
        }
        if (worker_idle) {
            encoder_close();   /* 必须在 worker 完全停手之后（编码器只被 worker 访问） */
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

    /* 上次 close 未完成（漏网帧在途、句柄保留）：先补关再开——流对象仍占着接口，
     * 不补关则 get_frame_list/open 必报 INVALID_ARG。迟到帧凑齐即在此恢复 */
    if (s_u.stream) {
        s_u.worker_run = false;
        if (safe_close_stream(s_u.stream)) {
            s_u.stream = NULL;
        } else {
            return ESP_FAIL;
        }
    }

    /* 部分 UVC 设备有多个流索引（复合 function），仅其一可用：逐个尝试直到
     * open+start 成功（协商控制传输超时 = 该 function 是哑通道，换下一个） */
    for (int cand = 0; cand < s_u.idx_cand_n; cand++) {
        esp_err_t err = try_open_stream_on_index(s_u.idx_candidates[cand]);
        if (err == ESP_OK) return ESP_OK;
        if (err == ESP_ERR_NOT_SUPPORTED) return err;   /* 档位问题：换索引也无解 */
        ESP_LOGW(TAG, "流索引 %u 打开失败（%s），尝试下一个候选",
                 s_u.idx_candidates[cand], esp_err_to_name(err));
    }
    return ESP_FAIL;
}

/* 帧率选择：≤640x480 传 0（设备默认=最高档，本机 640@60 实测满帧）；更大档位
 * 选 ≤30fps 的最大档——720p@60 的持续码率（~12Mbps）会打满 20MHz Wi-Fi 链路，
 * 连 httpd 都被挤死（真机实测 2026-10-08）。
 * ★ 显式 fps 必须与驱动同源计算（10000000.0f/interval 的浮点值）：驱动的
 *   uvc_desc_format_is_equal 用 FLOAT_EQUAL(eps=1e-4) 与描述符换算值比较——
 *   传整数 60.0f 对 166667（=59.99988f）差 2.4e-4 会匹配失败；同源计算则逐位相等。 */
static float pick_tier_fps(const uvc_host_frame_info_t *fi)
{
    if ((int)fi->h_res * (int)fi->v_res <= 640 * 480) return 0.0f;   /* 默认即最高帧率 */
    float best = 0.0f;
    for (int j = 0; j < fi->interval_type && j < 8; j++) {
        float f = 10000000.0f / (float)fi->interval[j];   /* 与驱动 UVC_DESC_DWFRAMEINTERVAL_TO_FPS 同源 */
        if (f > 0.5f && f <= 30.0f + 0.5f && f > best) best = f;
    }
    return best;   /* 0 = 无 ≤30fps 档时退回设备默认 */
}

/* 在指定 UVC 流索引上完成 取档位表 → 选档 → open → start（→ 编码器） */
static esp_err_t try_open_stream_on_index(uint8_t stream_idx)
{
    bool passthrough = (s_u.mode == USB_MODE_PASSTHROUGH);

    /* 每个索引的档位表可能不同：先取该索引的 */
    size_t cap = sizeof(s_u.frame_info) / sizeof(s_u.frame_info[0]);
    s_u.frame_info_n = 0;
    ESP_RETURN_ON_ERROR(uvc_host_get_frame_list(s_u.dev_addr, stream_idx,
                                                (uvc_host_frame_info_t (*)[])s_u.frame_info, &cap),
                        TAG, "get_frame_list idx=%u", stream_idx);
    s_u.frame_info_n = cap;

    /* 重编码优先 MJPEG 档（硬解码后重编码，帧率不受 YUY2 档 10fps 描述符限制；
     * 摄像头无 MJPEG 档时回退 YUY2）。直通仅 MJPEG 原生精确 */
    int idx;
    bool src_mjpeg;
    if (passthrough) {
        idx = pick_native(true, s_u.req_w, s_u.req_h, false);
        src_mjpeg = true;
    } else if ((idx = pick_native(true, s_u.req_w, s_u.req_h, true)) >= 0) {
        src_mjpeg = true;
    } else {
        idx = pick_native(false, s_u.req_w, s_u.req_h, true);
        src_mjpeg = false;
    }
    if (idx < 0) {
        ESP_LOGE(TAG, "%s 模式下不支持 %dx%d（见档位列表）",
                 passthrough ? "passthrough" : "reencode", s_u.req_w, s_u.req_h);
        return ESP_ERR_NOT_SUPPORTED;
    }
    uvc_host_frame_info_t *fi = &s_u.frame_info[idx];

    /* 输出分辨率：直通=原生；重编码=请求值（默认原生），由 plan_virtual 保证可行 */
    int out_w = passthrough ? fi->h_res : (s_u.req_w ? s_u.req_w : fi->h_res);
    int out_h = passthrough ? fi->v_res : (s_u.req_h ? s_u.req_h : fi->v_res);
    /* native 尺寸先于 encoder_open 落位（解码输出缓冲按 native 分配） */
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
                       .fps = pick_tier_fps(fi),   /* 同源浮点帧率；0=设备默认 */
                       .format = fi->format },
        .advanced = {
            .number_of_frame_buffers = FRAME_BUFFERS,
            .frame_size = 0,        /* 0 = 用协商出的 dwMaxVideoFrameSize */
            .frame_heap_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
            .number_of_urbs = URB_TIERS[0].n,
            .urb_size = URB_TIERS[0].size,
            .user_frame_buffers = NULL,   /* 驱动自分配（重编码的大帧也放得下） */
        },
    };
    uvc_host_stream_hdl_t h = NULL;
    esp_err_t err = ESP_ERR_NO_MEM;
    /* 按当前堆状态预选起始档：跳过总量/单块注定放不下的高档（避免每次都从 8×9216
     * 交 3 次注定失败的尝试、刷 3 条误导性 NO_MEM ERROR）；预选不准时 NO_MEM 链兜底 */
    size_t dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    size_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    int t_start = 0;
    while (t_start < URB_TIER_CNT - 1 &&
           ((size_t)URB_TIERS[t_start].n * URB_TIERS[t_start].size > dma_free ||
            URB_TIERS[t_start].size > dma_largest)) {
        t_start++;
    }
    if (t_start > 0)
        ESP_LOGW(TAG, "内部 DMA 空闲 %u 最大块 %u：URB 首选 %d×%uB（高档预留不足）",
                 (unsigned)dma_free, (unsigned)dma_largest, URB_TIERS[t_start].n, URB_TIERS[t_start].size);
    for (int t = t_start; t < URB_TIER_CNT; t++) {
        cfg.advanced.number_of_urbs = URB_TIERS[t].n;
        cfg.advanced.urb_size = URB_TIERS[t].size;
        err = uvc_host_stream_open(&cfg, pdMS_TO_TICKS(STREAM_OPEN_TIMEOUT_MS), &h);
        if (err == ESP_OK) {
            if (t != t_start) ESP_LOGW(TAG, "URB 降档 %d×%uB 开流成功（前档 %d×%uB 分配失败）",
                                       URB_TIERS[t].n, URB_TIERS[t].size,
                                       URB_TIERS[t - 1].n, URB_TIERS[t - 1].size);
            break;
        }
        if (err != ESP_ERR_NO_MEM) break;   /* 非内存失败（协商/超时）换档无意义 */
        ESP_LOGW(TAG, "stream_open NO_MEM（URB %d×%uB）：内部 DMA 空闲 %u 最大块 %u，降档重试",
                 URB_TIERS[t].n, URB_TIERS[t].size,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    }
    ESP_RETURN_ON_ERROR(err, TAG, "stream_open (%s %ux%u)", fmt_enum_name(fi->format), fi->h_res, fi->v_res);
    s_u.stream = h;

    err = uvc_host_stream_start(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "stream_start failed (%s)", esp_err_to_name(err));
        s_u.worker_run = false;   /* 先停 worker，safe_close 依赖其排水归还竞态帧 */
        uvc_host_stream_stop(h);  /* start 失败也可能已 unpause 送帧：先停再关 */
        if (safe_close_stream(h)) s_u.stream = NULL;   /* 失败则保留句柄待迟到帧 */
        return err;
    }

    if (!passthrough) {
        err = encoder_open(out_w, out_h, s_u.quality);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "encoder_open failed");
            s_u.worker_run = false;
            uvc_host_stream_stop(h);
            if (safe_close_stream(h)) s_u.stream = NULL;   /* 失败则保留句柄待迟到帧 */
            return err;
        }
    }

    s_u.stream_index = stream_idx;
    s_u.w = fi->h_res;
    s_u.h = fi->v_res;
    s_u.out_w = out_w;
    s_u.out_h = out_h;
    s_u.next_due_us = 0;
    rebuild_tiers();   /* ★ 必须先于 stream_up=true：switch 返回后上层立即查档位表 */
    s_u.stream_up = true;
    s_u.stream_gen++;  /* 看门狗基线换代：给新流 5s 首帧宽限（旧流的停滞计时不带入） */
    /* ★ 开流收尾必须【归还】open 期间（stream_start 起即有帧流入，encoder_open/档位
     * 日志可达数百 ms）排队的帧，绝不能 xQueueReset 静默丢弃——丢弃=帧永不归还=
     * 下次 close 永久失败=流泄漏接口占死（真机楔死根因之一） */
    int stale = drain_work_queue(s_u.stream);
    if (stale) ESP_LOGW(TAG, "开流收尾归还 %d 个排队帧", stale);
    s_u.worker_run = true;
    bool scaled = (out_w != fi->h_res || out_h != fi->v_res);
    ESP_LOGI(TAG, "UVC 推流开始（流索引 %u）：%s %ux%u%s%s quality=%u fps_limit=%d",
             stream_idx,
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
            /* 停止态：teardown 的 close 重试期间，把漏网的竞态帧归还驱动
             * （s_u.stream 在 close 成功前保持有效——见 teardown_stream 注释） */
            uvc_host_stream_hdl_t h = s_u.stream;
            while (xQueueReceive(s_u.work_q, &f, 0) == pdTRUE) {
                ret_frame(h, f);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
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
                ret_frame(h, f);
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
            ret_frame(h, f);
        } else {
            /* ★ 模式 B：重编码（quality 可控，与 DVP 同档可比）。
             * 输入两类：MJPEG 档（硬件解码→UYVY，帧率不受 YUY2 档 10fps 描述符限制）
             * / YUY2 档（直接字节序交换，回退路径） */
            const uint8_t *src = f->data;
            if (s_u.dec_src_mjpeg) {
                if (!s_u.dec || !s_u.dec_out) {
                    s_u.stats.cap_drops++;   /* 解码器未就绪（开流竞态）：丢弃 */
                    ret_frame(h, f);
                    s_u.worker_busy = false;
                    continue;
                }
                jpeg_decode_cfg_t dc = { .output_format = JPEG_DECODE_OUT_FORMAT_YUV422 };
                uint32_t dec_len = 0;
                esp_err_t derr = jpeg_decoder_process(s_u.dec, &dc, f->data, f->data_len,
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
            int idx;
            if (!h || xQueueReceive(s_u.enc_free_in, &idx, 0) != pdTRUE) {
                s_u.stats.cap_drops++;   /* 编码器忙：丢帧保时延 */
                ret_frame(h, f);
                s_u.worker_busy = false;
                continue;
            }
            int kx, ky, x0, y0;
            plan_virtual(s_u.w, s_u.h, s_u.out_w, s_u.out_h, &kx, &ky, &x0, &y0);
            size_t out_bytes = MIN((size_t)s_u.out_w * s_u.out_h * 2, s_u.enc_in[idx].length);
            fill_encoder_input(s_u.enc_in[idx].start, src,
                               s_u.w, s_u.h, s_u.out_w, s_u.out_h, kx, ky, x0, y0,
                               !s_u.dec_src_mjpeg, t_arr);   /* YUY2 需交换；解码输出已是 UYVY */
            struct v4l2_buffer vb = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_USERPTR,
                                      .index = idx, .bytesused = out_bytes };
            vb.m.userptr = (unsigned long)s_u.enc_in[idx].start;
            vb.length = s_u.enc_in[idx].length;
            {
                static int dbg_q;
                if (++dbg_q <= 3)
                    ESP_LOGW(TAG, "QBUF(OUT) idx=%d userptr=%p(%%4k=%u) len=%u bytesused=%u",
                             idx, s_u.enc_in[idx].start,
                             (unsigned)((uintptr_t)s_u.enc_in[idx].start % 4096),
                             (unsigned)vb.length, (unsigned)vb.bytesused);
            }
            if (xioctl(s_u.enc_fd, VIDIOC_QBUF, &vb) != 0) {
                static uint32_t qbuf_err;
                if (++qbuf_err % 30 == 1)
                    ESP_LOGW(TAG, "enc QBUF(OUTPUT) 失败 errno=%d out_bytes=%u in_len=%u",
                             errno, (unsigned)out_bytes, (unsigned)s_u.enc_in[idx].length);
                xQueueSend(s_u.enc_free_in, &idx, 0);
                s_u.stats.cap_drops++;
                ret_frame(h, f);
                s_u.worker_busy = false;
                continue;
            }
            struct v4l2_buffer ob = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
            bool enc_ok = xioctl(s_u.enc_fd, VIDIOC_DQBUF, &ob) == 0;   /* M2M：DQBUF 内同步编码 */
            if (!enc_ok) {
                static uint32_t dqbuf_err;
                if (++dqbuf_err % 30 == 1)
                    ESP_LOGW(TAG, "enc DQBUF(CAPTURE) 失败 errno=%d（200ms 超时？）", errno);
            }
            /* ★ DQBUF(OUTPUT)：取回被消费的输入槽，元素回到 FREE 态。
             *   缺此步则元素永久 ALLOCATED，同槽第二次 QBUF 报 EINVAL
             *   （与 camera_pipeline 的 encode_task 中 DQBUF(OUTPUT) 同款必做步骤） */
            struct v4l2_buffer ob_in = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_USERPTR };
            if (xioctl(s_u.enc_fd, VIDIOC_DQBUF, &ob_in) != 0) {
                static uint32_t din_err;
                if (++din_err % 30 == 1)
                    ESP_LOGW(TAG, "enc DQBUF(OUTPUT) 失败 errno=%d", errno);
            }
            uint64_t t_enc = esp_timer_get_time();
            xQueueSend(s_u.enc_free_in, &idx, 0);
            ret_frame(h, f);

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

/* lib 事件泵：必须独占常驻（IDF v6 usbh 的传输完成事件经 lib 队列派发给各 client；
 * 若在状态机任务里附带泵送，任何一次阻塞（如 stream_open 等控制传输）都会冻结全总线
 * 的事件派发 —— 真机表现为 uvc_host_usb_ctrl CTRL timeout，官方示例同款独立任务） */
static void usb_lib_task(void *arg)
{
    uint32_t flags;
    uint32_t iters = 0;
    while (1) {
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &flags);
        iters++;
        if (flags) {
            ESP_LOGW(TAG, "lib evt=0x%x iter=%u", (unsigned)flags, (unsigned)iters);
        }
        static uint64_t last_hb;
        uint64_t now = esp_timer_get_time();
        if (now - last_hb > 5000000) {
            ESP_LOGI(TAG, "usb_lib 心跳：iter=%u err=%s", (unsigned)iters, esp_err_to_name(err));
            last_hb = now;
        }
    }
}

/* ---------- monitor：client 事件派发 + 状态机 + 自动重连（U5） ---------- */
static void monitor_task(void *arg)
{
    static int64_t open_fail_since;   /* 连续打开失败起始（0=未在失败中；dev 拔出/开流成功即清零） */
    for (;;) {
        usb_host_client_handle_events(s_u.client, 0);   /* 非阻塞派发 NEW_DEV/DEV_GONE */
        static uint64_t last_hb;
        if (esp_timer_get_time() - last_hb > 5000000) {
            ESP_LOGI(TAG, "monitor 心跳：dev=%d stream=%d want=%d cap=%u",
                     (int)s_u.dev_present, (int)s_u.stream_up, (int)s_u.want_stream,
                     (unsigned)s_u.stats.cap_frames);
            last_hb = esp_timer_get_time();
        }

        /* ★ 帧停滞看门狗（背景与阈值见 STALL_* 定义处注释）：驱动静默停流时
         * stream_up/want 恒为 1 而帧归零，旧状态机依赖 !stream_up 才重连，
         * 卡 1 即永不自愈——这里补上唯一缺失的触发条件 */
        {
            static uint32_t last_gen, last_frames;
            static int64_t  last_progress_us;
            static uint8_t  stall_rounds;
            uint32_t gen = s_u.stream_gen;
            if (gen != last_gen) {
                /* 新流刚开好：重置首帧宽限基线（停滞轮数只在帧真正流动后清零） */
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
                    last_progress_us = now;   /* 重开期间防重复触发（teardown 下轮即执行） */
                    if (stall_rounds >= STALL_MAX_ROUNDS) {
                        if (scan_ctrl_state() == SCAN_RUNNING) {
                            /* 扫描中重启会毁掉整轮矩阵（2026-10-09 真机：重编码档
                             * 楔死触发重启，72 行扫到 18 行即被清零）——改为继续
                             * teardown 重开轮询，每行 apply 的切流也会重建会话；
                             * 扫描结束后若仍楔死，下一轮计数自然触发重启 */
                            ESP_LOGW(TAG, "连续 %u 轮重开仍无帧，但扫描进行中，"
                                     "推迟重启并继续重开自愈", (unsigned)stall_rounds);
                            stall_rounds = 0;
                        } else {
                            ESP_LOGE(TAG, "连续 %u 轮重开仍无帧（驱动/总线楔死），重启系统兜底",
                                     (unsigned)stall_rounds);
                            esp_restart();
                        }
                    }
                    s_u.teardown_req = true;
                }
            }
        }

        /* 清理请求 / 自愈（want=0 但流悬挂）/ close 重试（close 未完成句柄保留中）：
         * 条件必须在【锁内】重新求值再执行——set_mode/apply 的同步 teardown+open 与
         * 本任务并发时，锁外读到的旧条件会误杀刚打开的新流（真机实测：每次切换后
         * 12~16 ms 新流即被"want=1"的误 teardown 杀掉再重开，快速连切时叠加成泄漏） */
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

        /* 自动（重）连接：USB 为当前源 && 有设备 && 未推流 && 无残留流句柄 → 周期尝试
         * （残留句柄=上次 close 未完成，接口仍被占，open 必失败，先等上面补关） */
        /* 计时窗口=「want+dev 恒真且推流未成」；拔出/切走/已开流都清零，防陈旧计时误重启 */
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
                        if (scan_ctrl_state() == SCAN_RUNNING) {
                            /* 扫描中重启会毁掉整轮矩阵——推迟 60 s 窗口继续 1 s 重试
                             * （scan 每行 apply 会 teardown+open，本身也是自愈路径）；
                             * 扫描结束后窗口重新累计，仍不愈才重启 */
                            ESP_LOGW(TAG, "连续 %lld s 打开失败，扫描进行中推迟重启，继续重试",
                                     (long long)((now - open_fail_since) / 1000000));
                            open_fail_since = now;
                        } else {
                            /* 真机实测（2026-10-08）：静默停流→teardown CTRL timeout→驱动
                             * close 簿记不一致 → open 永久失败，1 s 重试不愈；重启后同一
                             * 摄像头经 USB 重枚举立即恢复——见 OPEN_FAIL_REBOOT_US 注释 */
                            ESP_LOGE(TAG, "连续 %lld s 打开失败仍不愈（dev=1 want=1 held=%d），"
                                     "重启由 USB 重枚举自愈",
                                     (long long)((now - open_fail_since) / 1000000), s_u.frames_held);
                            esp_restart();
                        }
                    }
                    s_u.next_retry_us = now + RETRY_BACKOFF_MS * 1000;
                    ESP_LOGW(TAG, "UVC 打开失败（%s），%d s 后重试；若反复失败请检查 USB 供电"
                             "（Korvo-1 Type-A 经 TPS2051C 限流 500 mA，须 Type-C 口补供电）",
                             esp_err_to_name(err), RETRY_BACKOFF_MS / 1000);
                } else {
                    open_fail_since = 0;
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

    if (xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 3072, NULL, 6, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
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

    /* 分辨率变化需要重新协商流（直通必须原生精确；重编码经缩放）。
     * 同步执行：teardown + 设置参数 + 直开（不经 monitor 异步，消灭竞态） */
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
        s_u.want_stream = true;   /* ★ 锁内恢复（失败时 monitor 会按 want 重试），防 monitor 误杀新流 */
        xSemaphoreGive(s_u.lock);
        if (err != ESP_OK) return err;
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
        ESP_LOGW(TAG, "passthrough 模式不解码，无法叠加毫秒计数器；已由配置层自动切 reencode"
                     "（重编码走 MJPEG 档硬解码，帧率不受 YUY2 10fps 档限制）");
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
    strlcpy(info.pix_fmt_str,
            s_u.mode == USB_MODE_PASSTHROUGH ? "JPEG" :
            s_u.dec_src_mjpeg ? "MJPEG→解码→JPEG" : "YUY2→JPEG", sizeof(info.pix_fmt_str));
    snprintf(info.fmt_name, sizeof(info.fmt_name), "%s%s%ux%u",
             s_u.mode == USB_MODE_PASSTHROUGH ? "MJPEG直通" :
             s_u.dec_src_mjpeg ? "MJPEG解码重编码" : "YUY2重编码",
             info.scaled ? "缩放" : "", info.w, info.h);
    return &info;
}

src_stats_t *source_usb_stats(void) { return &s_u.stats; }

frame_ring_t *source_usb_ring(void) { return cam_pipe_ring(); }

int source_usb_supported_res(char *out, size_t outlen)
{
    /* 当前模式下的可用档：直通=MJPEG 原生精确档；重编码=YUY2 原生 + 可缩放的虚拟档。
     * ★ 取锁：rebuild_tiers() 在流打开路径清零重填档位表，无锁会读到中间态 */
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
        } else if (!t->mjpeg) {
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
    if (was_active && s_u.dev_present) {
        /* 同步重开（不经 monitor 异步）：新模式失败回滚旧模式 */
        s_u.want_stream = false;
        s_u.teardown_req = false;
        xSemaphoreTake(s_u.lock, portMAX_DELAY);
        teardown_stream();
        s_u.mode = m;   /* ★ 旧流完全停手后再切换：否则 worker 把旧格式帧走错分支 */
        esp_err_t err = open_stream_locked();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "切换到 %s 失败（%s），回滚 %s",
                     m == USB_MODE_PASSTHROUGH ? "passthrough" : "reencode",
                     esp_err_to_name(err),
                     old == USB_MODE_PASSTHROUGH ? "passthrough" : "reencode");
            s_u.mode = old;
            err = open_stream_locked();   /* 尽力回滚 */
        }
        s_u.want_stream = true;   /* ★ 锁内恢复：monitor 锁内求值不会误杀刚开的流 */
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
