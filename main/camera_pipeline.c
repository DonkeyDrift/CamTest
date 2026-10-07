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

#if CONFIG_CAMTEST_OV3660_HIFPS_L4
#define CAMTEST_OV3660_HIFPS_LEVEL 4
#elif CONFIG_CAMTEST_OV3660_HIFPS_L3
#define CAMTEST_OV3660_HIFPS_LEVEL 3
#elif CONFIG_CAMTEST_OV3660_HIFPS_L2
#define CAMTEST_OV3660_HIFPS_LEVEL 2
#elif CONFIG_CAMTEST_OV3660_HIFPS_L1
#define CAMTEST_OV3660_HIFPS_LEVEL 1
#else
#define CAMTEST_OV3660_HIFPS_LEVEL 0
#endif

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
    const esp_cam_sensor_format_t *cur_fmt;    /* 当前 sensor 格式（采集用原生档） */
    int src_w, src_h;                          /* 原生采集分辨率 */
    int tgt_w, tgt_h;                          /* 目标输出分辨率（虚拟档可小于原生） */
    struct { int kx, ky, x0, y0; } dsc;        /* 原生→目标映射：整数抽取(kx,ky) + 中心裁剪起点(x0,y0) */
    /* OV3660 高帧率实验档：克隆 240x240 YUYV 寄存器表并改写 VTS(0x380e/f)。
     * 帧率 ≈ PCLK/(HTS*VTS)，实测 25fps@VTS783 → 线性外推 VTS320≈61fps。
     * set_format 会保留格式指针，故必须静态存储。 */
    const esp_cam_sensor_format_t *small_yuv_fmt;   /* 240x240 YUYV 原生档（克隆母本） */
    esp_cam_sensor_format_t boost_fmt;              /* 合成高帧率档（静态） */
    bool boost_valid;
    int  vts_override;                              /* 0=用原生表；>0=VTS 改写值 */
    int  boost_level;                               /* 0=off, 1..4 档位 */

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

/*
 * 原生帧 → 编码器输入：整数抽取(kx,ky，保全视场) 或 中心裁剪(x0,y0)，同时完成
 * YUYV→UYVY / RGB565_BE→LE 的字节序转换（推导见文件头与 API_NOTES.md）。
 * YUV422 按宏像素（2 像素 4 字节，色度共用）处理，要求目标宽为偶数。
 */
static void fill_encoder_input(uint8_t *dst, const uint8_t *src, uint32_t fourcc, uint64_t now_us)
{
    const int sw = s_p.src_w, sh = s_p.src_h;
    const int tw = s_p.info.w, th = s_p.info.h;
    const int kx = s_p.dsc.kx, ky = s_p.dsc.ky, x0 = s_p.dsc.x0, y0 = s_p.dsc.y0;
    const size_t srow = (size_t)sw * 2, drow = (size_t)tw * 2;   /* 两种格式均 2B/px */

    const bool passthrough = (tw == sw && th == sh && kx == 1 && ky == 1 && x0 == 0 && y0 == 0);
    if (fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_UYVY) {
        const bool swap = (fourcc == V4L2_PIX_FMT_YUYV);   /* YUYV→UYVY 逐 16bit 交换 */
        if (passthrough && !swap) {
            memcpy(dst, src, drow * sh);
        } else if (passthrough) {
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
                    if (swap) { dp[0]=sp[1]; dp[1]=sp[0]; dp[2]=sp[3]; dp[3]=sp[2]; }
                    else      { dp[0]=sp[0]; dp[1]=sp[1]; dp[2]=sp[2]; dp[3]=sp[3]; }
                }
            }
        }
    } else {   /* RGB565：BE(X) 需交换，LE 直拷 */
        const bool swap = (fourcc == V4L2_PIX_FMT_RGB565X);
        for (int ty = 0; ty < th; ty++) {
            const uint16_t *sr = (const uint16_t *)(src + (size_t)(y0 + ty * ky) * srow) + x0;
            uint16_t *dr = (uint16_t *)(dst + (size_t)ty * drow);
            for (int tx = 0; tx < tw; tx++) {
                uint16_t v = sr[(size_t)tx * kx];
                dr[tx] = swap ? __builtin_bswap16(v) : v;
            }
        }
    }
    if (s_p.overlay) {
        uint32_t out_fourcc = (fourcc == V4L2_PIX_FMT_YUYV) ? V4L2_PIX_FMT_UYVY : fourcc;
        yuv_osd_draw_ms_counter(dst, drow, tw, th, fmt_to_osd(out_fourcc), now_us);
    }
}

/*
 * 规划 原生(sw,sh)→目标(tw,th) 的映射：
 *   1) 相等：恒等；
 *   2) 双轴整除：整数抽取（保全视场，优先）；
 *   3) 目标不超过源：中心裁剪（视野变小）；
 *   4) 需要放大：不支持。
 * YUV422 宏像素要求目标宽为偶数。
 */
static bool plan_virtual(int sw, int sh, int tw, int th, int *kx, int *ky, int *x0, int *y0)
{
    if ((tw & 1) || tw < 16 || th < 16) return false;
    if (tw == sw && th == sh) { *kx=*ky=1; *x0=*y0=0; return true; }
    if (sw % tw == 0 && sh % th == 0) { *kx=sw/tw; *ky=sh/th; *x0=*y0=0; return true; }
    if (tw <= sw && th <= sh) { *kx=*ky=1; *x0=(sw-tw)/2 & ~1; *y0=(sh-th)/2; return true; }
    return false;
}

static bool is_yuv(esp_cam_sensor_output_format_t f)
{
    return f == ESP_CAM_SENSOR_PIXFORMAT_YUV422_UYVY || f == ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV;
}

/* 与 ov3660_reginfo_t 二进制兼容（{u16 reg; u8 val;}，TAIL: reg==0） */
typedef struct { uint16_t reg; uint8_t val; } cam_reg_t;
static cam_reg_t s_boost_regs[256];

/*
 * 构建高帧率档：克隆 small_yuv_fmt(240x240 YUYV) 的寄存器序列并改写时序/窗口寄存器。
 *
 * 物理推导（实测标定）：该模式 V 窗口=全高 1548 行、2x binning → 读出 774 行 ≈ VTS783，
 * 即 VTS 已是读出下限，单减 VTS 会失步（0fps）。
 * 帧率 = PCLK/(HTS×VTS)，PCLK 实测 40MHz（DVP 安全上限内不宜再升）。
 * 提速唯一正道：中心裁剪读出窗口 → 行数变少 → VTS/HTS 同步缩小：
 *   读出行数 ≈ 774·cV，HTS ≈ (1568·cH)+280，VTS ≈ 774·cV+12
 *   fps ≈ 40MHz/(HTS×VTS)（c=1 时还原 25fps）
 * 代价：视场按 cH×cV 缩小（数字变焦）。输出仍 240x240（sensor ISP 缩放）。
 * bp 全 0 = 恢复原生表。
 */
typedef cam_boost_params_t boost_params_t;
/* 实验性时钟树补丁（值来自 esp_cam_sensor 2.4.x 官方 JPEG 30fps 档，数据驱动移植） */
typedef struct { int c303b, c303d, c3824; } boost_clk_t;
static boost_clk_t s_boost_clk;

static void patch_reg16(cam_reg_t *regs, size_t n, uint16_t reg_hi, int val)
{
    for (size_t i = 0; i < n; i++) {
        if (regs[i].reg == 0) break;
        if (regs[i].reg == reg_hi)     regs[i].val = (val >> 8) & 0xFF;
        if (regs[i].reg == reg_hi + 1) regs[i].val = val & 0xFF;
    }
}

static bool build_boost_fmt(const boost_params_t *bp)
{
    if (!s_p.small_yuv_fmt) return false;
    if (bp->vts == 0) { s_p.boost_valid = false; return false; }
    size_t n = s_p.small_yuv_fmt->regs_size;
    if (n == 0 || n > sizeof(s_boost_regs) / sizeof(s_boost_regs[0])) return false;
    memcpy(s_boost_regs, s_p.small_yuv_fmt->regs, n * sizeof(cam_reg_t));
    if (bp->vts)    patch_reg16(s_boost_regs, n, 0x380e, bp->vts);
    if (bp->hts)    patch_reg16(s_boost_regs, n, 0x380c, bp->hts);
    if (bp->vstart) patch_reg16(s_boost_regs, n, 0x3802, bp->vstart);
    if (bp->vend)   patch_reg16(s_boost_regs, n, 0x3806, bp->vend);
    if (bp->hstart) patch_reg16(s_boost_regs, n, 0x3800, bp->hstart);
    if (bp->hend)   patch_reg16(s_boost_regs, n, 0x3804, bp->hend);
    if (s_boost_clk.c303b >= 0) {
        for (size_t i = 0; i < n; i++) {
            if (s_boost_regs[i].reg == 0) break;
            if (s_boost_regs[i].reg == 0x303b) s_boost_regs[i].val = (uint8_t)s_boost_clk.c303b;
            if (s_boost_regs[i].reg == 0x303d) s_boost_regs[i].val = (uint8_t)s_boost_clk.c303d;
            if (s_boost_regs[i].reg == 0x3824) s_boost_regs[i].val = (uint8_t)s_boost_clk.c3824;
        }
    }

    s_p.boost_fmt = *s_p.small_yuv_fmt;
    s_p.boost_fmt.regs = s_boost_regs;
    s_p.boost_fmt.regs_size = n;
    int fps_est = (bp->hts && bp->vts) ? (int)(40000000LL / ((int64_t)bp->hts * bp->vts)) : 25;
    s_p.boost_fmt.fps = fps_est;
    static char boost_name[64];
    snprintf(boost_name, sizeof(boost_name), "240x240_YUYV 窗口裁剪 VTS=%d HTS=%d(~%dfps)",
             bp->vts, bp->hts, fps_est);
    s_p.boost_fmt.name = boost_name;
    s_p.boost_valid = true;
    return true;
}

/* 高帧率档位表：实测标定（2026-10-07）。时钟树来自官方 JPEG30fps 档。
 * 窗口垂直居中裁剪；VTS≈spanV/2+8。 */
typedef struct {
    int span_v;    /* 垂直读出跨度（1548=全高） */
    int fps_expect;
    const char *label;
} hifps_level_t;
static const hifps_level_t s_hifps_levels[] = {
    [0] = { 1548, 25, "OFF" },
    [1] = { 1548, 31, "L1 全视场" },
    [2] = {  800, 59, "L2 V52%" },
    [3] = {  640, 71, "L3 V41%" },
    [4] = {  512, 87, "L4 V33%" },
};

void cam_boost_apply_level(int level)
{
    if (!s_p.small_yuv_fmt || level <= 0 ||
        level >= (int)(sizeof(s_hifps_levels) / sizeof(s_hifps_levels[0]))) {
        s_p.boost_valid = false;
        return;
    }
    const hifps_level_t *lv = &s_hifps_levels[level];
    int span_v = lv->span_v & ~7;
    boost_params_t bp = {
        .vts = span_v / 2 + 8,
        .vstart = (1548 - span_v) / 2 & ~3,
        .vend = ((1548 - span_v) / 2 & ~3) + span_v,
        .hstart = 256, .hend = 1823,   /* 水平不动（H 侧改动会失步） */
    };
    s_boost_clk.c303b = 0x1e;   /* 官方 JPEG30fps 时钟树 */
    s_boost_clk.c303d = 0x30;
    s_boost_clk.c3824 = 0x0a;
    if (build_boost_fmt(&bp)) {
        static char name[64];
        snprintf(name, sizeof(name), "240x240_YUYV hifps-%s(~%dfps)", lv->label, lv->fps_expect);
        s_p.boost_fmt.name = name;
        s_p.boost_fmt.fps = lv->fps_expect;
        s_p.boost_level = level;
        ESP_LOGI(TAG, "hifps L%d: spanV=%d VTS=%d → 预期 ~%dfps",
                 level, span_v, bp.vts, lv->fps_expect);
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
    /* 队列/信号量/帧环均不销毁（任务或客户端可能仍阻塞/持有其中）——init 一次创建，
     * 重建时只做复位（见 build_pipeline_locked）。DQBUF 已设 200ms 超时，
     * 停靠握手能在超时窗口内达成，不会悬挂 apply。 */
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

    xQueueReset(s_p.free_in);
    while (xSemaphoreTake(s_p.enc_trigger, 0) == pdTRUE) {}   /* 清残留触发 */
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
    s_p.src_w = f->width; s_p.src_h = f->height;          /* 原生采集分辨率 */
    s_p.info.w = s_p.tgt_w; s_p.info.h = s_p.tgt_h;       /* 目标输出分辨率（虚拟档） */
    s_p.info.fps = f->fps; s_p.info.pclk_hz = f->xclk;   /* 格式表只有 xclk 输入时钟，无 PCLK 字段 */
    if (s_p.tgt_w == f->width && s_p.tgt_h == f->height) {
        strlcpy(s_p.info.fmt_name, f->name, sizeof(s_p.info.fmt_name));
    } else {
        snprintf(s_p.info.fmt_name, sizeof(s_p.info.fmt_name), "%s%s→%dx%d",
                 f->name,
                 (s_p.dsc.kx > 1 || s_p.dsc.ky > 1) ? " 抽取" : " 裁剪",
                 s_p.tgt_w, s_p.tgt_h);
    }
    uint32_t fourcc = sensor_fmt_to_v4l2(f->format);
    esp_cam_sensor_output_format_t of = f->format;
    memcpy(s_p.info.v4l2_fourcc, &fourcc, 4); s_p.info.v4l2_fourcc[4] = 0;

    s_p.cam_fd = open(CAM_DEV, O_RDWR);
    ESP_RETURN_ON_FALSE(s_p.cam_fd >= 0, ESP_FAIL, TAG, "open %s", CAM_DEV);
    {
        struct timeval tv = { .tv_sec = 0, .tv_usec = DQBUF_TIMEOUT_MS * 1000 };
        xioctl(s_p.cam_fd, VIDIOC_S_DQBUF_TIMEOUT, &tv);   /* 停靠握手依赖此超时 */
    }

    /* 1) 切 sensor 格式（分辨率切换的唯一途径）。
     * 注意：驱动会保留该指针（dev->cur_format），必须传静态存储的格式（表项或 boost_fmt）。 */
    ESP_RETURN_ON_ERROR(xioctl(s_p.cam_fd, VIDIOC_S_SENSOR_FMT, (void *)f) ? ESP_FAIL : ESP_OK,
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
    {
        struct timeval tv = { .tv_sec = 0, .tv_usec = DQBUF_TIMEOUT_MS * 1000 };
        xioctl(s_p.jpeg_fd, VIDIOC_S_DQBUF_TIMEOUT, &tv);
    }
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
            /* 编码器输入按目标分辨率填充（源帧可能更大，抽取/裁剪在 fill 内完成） */
            size_t out_bytes = (size_t)s_p.info.w * s_p.info.h * 2;
            out_bytes = MIN(out_bytes, s_p.j_in[idx].length);
            fill_encoder_input(s_p.j_in[idx].start, s_p.cam_buf[buf.index].start,
                               sensor_fmt_to_v4l2(s_p.cur_fmt->format), t_cap);
            s_p.j_in_tcap[idx] = t_cap;
            struct v4l2_buffer vb = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_MMAP,
                                      .index = idx, .bytesused = out_bytes };
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
    s_p.free_in = xQueueCreate(JPEG_IN_BUFS, sizeof(int));
    s_p.enc_trigger = xSemaphoreCreateCounting(JPEG_IN_BUFS, 0);
    ESP_RETURN_ON_FALSE(s_p.free_in && s_p.enc_trigger, ESP_ERR_NO_MEM, TAG, "sync primitives");

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
    s_p.tgt_w = s_p.cur_fmt->width;
    s_p.tgt_h = s_p.cur_fmt->height;
    s_p.dsc.kx = s_p.dsc.ky = 1;
    s_p.dsc.x0 = s_p.dsc.y0 = 0;
    /* OV3660 高帧率母本：最小的 YUV 方形档（240x240） */
    if (strcasecmp(s_p.info.sensor_name, "ov3660") == 0) {
        for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
            const esp_cam_sensor_format_t *f = &s_p.fmt_array.format_array[i];
            if (is_yuv(f->format) && f->width == 240 && f->height == 240) {
                s_p.small_yuv_fmt = f;
                break;
            }
        }
        cam_boost_apply_level(CAMTEST_OV3660_HIFPS_LEVEL);
    }
    s_p.inited = true;
    return ESP_OK;
}

void cam_pipe_start(void)
{
    xSemaphoreTake(s_p.api_lock, portMAX_DELAY);
    /* cam_pipe_stop() 后 ring 仍在但 running=false，同样需要重建链路 */
    if (!s_p.ring || !s_p.running) {
        esp_err_t err = build_pipeline_locked();
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    }
    if (!s_p.cap_task_h) {
        xTaskCreatePinnedToCore(capture_task, "cam_cap", 6144, NULL, 14, &s_p.cap_task_h, 1);
        xTaskCreatePinnedToCore(encode_task, "cam_enc", 6144, NULL, 12, &s_p.enc_task_h, 1);
    }
    xEventGroupSetBits(s_p.ev, EV_RUN);
    xSemaphoreGive(s_p.api_lock);
}

/* 完全停流（切换到 USB 源时调用）：任务停靠到安全点 → STREAMOFF + 关 fd。
 * 帧环/队列/任务句柄全部保留，cam_pipe_start() 可无损重启。 */
void cam_pipe_stop(void)
{
    xSemaphoreTake(s_p.api_lock, portMAX_DELAY);
    xEventGroupClearBits(s_p.ev, EV_RUN);
    xEventGroupWaitBits(s_p.ev, EV_CAP_PARK | EV_ENC_PARK, pdFALSE, pdTRUE,
                        pdMS_TO_TICKS(2000));
    vTaskDelay(pdMS_TO_TICKS(30));
    teardown_pipeline_locked();
    xSemaphoreGive(s_p.api_lock);
}

bool cam_pipe_scaled(int *native_w, int *native_h)
{
    bool scaled = s_p.running && (s_p.src_w != s_p.info.w || s_p.src_h != s_p.info.h);
    if (scaled && native_w) *native_w = s_p.src_w;
    if (scaled && native_h) *native_h = s_p.src_h;
    return scaled;
}

esp_err_t cam_pipe_apply_vts(int w, int h, uint8_t quality, int fps_limit, int vts)
{
    xSemaphoreTake(s_p.api_lock, portMAX_DELAY);
    /* 停任务（清 RUN，等两任务都停在安全点；最长 2 s） */
    xEventGroupClearBits(s_p.ev, EV_RUN);
    xEventGroupWaitBits(s_p.ev, EV_CAP_PARK | EV_ENC_PARK, pdFALSE, pdTRUE,
                        pdMS_TO_TICKS(2000));
    vTaskDelay(pdMS_TO_TICKS(30));   /* 双保险：等退出中的 ioctl 全部返回 */

    const esp_cam_sensor_format_t *want = NULL;
    int vkx = 1, vky = 1, vx0 = 0, vy0 = 0;
    (void)vts;   /* boost 由 apply_boost 预先构建；此入口不再单独改 VTS */
    if (w && h) {
        /* 候选 = 原生表 ∪ {boost}；评分 = 帧率为主，同分时 原生直通 > 整数抽取(保全视场) > 裁剪 */
        int best_score = -1;
        for (int pass = 0; pass < 2; pass++) {   /* 两轮都评分：原生表 ∪ boost 取最高分 */
            const esp_cam_sensor_format_t *tab = (pass == 0) ? s_p.fmt_array.format_array
                                                  : (s_p.boost_valid ? &s_p.boost_fmt : NULL);
            uint32_t count = (pass == 0) ? s_p.fmt_array.count : (s_p.boost_valid ? 1 : 0);
            for (uint32_t i = 0; i < count; i++) {
                const esp_cam_sensor_format_t *f = &tab[i];
                if (!fmt_hw_encodable(f->format) || !is_yuv(f->format)) continue;
                int kx, ky, x0, y0, score;
                if (f->width == w && f->height == h) {
                    kx = ky = 1; x0 = y0 = 0;
                    score = f->fps * 100 + 3;
                } else if (plan_virtual(f->width, f->height, w, h, &kx, &ky, &x0, &y0)) {
                    score = f->fps * 100 + ((kx > 1 || ky > 1) ? 2 : 1);
                } else {
                    continue;
                }
                if (score > best_score) {
                    best_score = score;
                    want = f; vkx = kx; vky = ky; vx0 = x0; vy0 = y0;
                }
            }
        }
        /* YUV 无解时允许 RGB 候选（兜底） */
        if (!want) {
            for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
                const esp_cam_sensor_format_t *f = &s_p.fmt_array.format_array[i];
                if (!fmt_hw_encodable(f->format)) continue;
                int kx, ky, x0, y0;
                if (f->width == w && f->height == h) { want = f; vkx=vky=1; vx0=vy0=0; break; }
                if (plan_virtual(f->width, f->height, w, h, &kx, &ky, &x0, &y0)) {
                    want = f; vkx=kx; vky=ky; vx0=x0; vy0=y0; break;
                }
            }
        }
        if (!want) { /* 恢复运行后报不支持 */
            xEventGroupSetBits(s_p.ev, EV_RUN);
            xSemaphoreGive(s_p.api_lock);
            return ESP_ERR_NOT_SUPPORTED;
        }
        s_p.cur_fmt = want;
        s_p.tgt_w = w; s_p.tgt_h = h;
        s_p.dsc.kx = vkx; s_p.dsc.ky = vky; s_p.dsc.x0 = vx0; s_p.dsc.y0 = vy0;
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

esp_err_t cam_pipe_apply(int w, int h, uint8_t quality, int fps_limit)
{
    return cam_pipe_apply_vts(w, h, quality, fps_limit, 0);
}

/* 带 boost 窗口参数的重建：bp 非空且 vts>0 时启用，否则清除 boost */
esp_err_t cam_pipe_apply_boost(int w, int h, uint8_t quality, int fps_limit,
                               const cam_boost_params_t *bp)
{
    if (bp && bp->vts > 0) {
        if (!build_boost_fmt(bp)) return ESP_ERR_INVALID_ARG;
    } else {
        s_p.boost_valid = false;
    }
    return cam_pipe_apply_vts(w, h, quality, fps_limit, 0);
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
    int kx, ky, x0, y0;
    for (uint32_t i = 0; i < s_p.fmt_array.count; i++) {
        const esp_cam_sensor_format_t *f = &s_p.fmt_array.format_array[i];
        if (!fmt_hw_encodable(f->format)) continue;
        if (f->width == w && f->height == h) return true;
        if (plan_virtual(f->width, f->height, w, h, &kx, &ky, &x0, &y0)) return true;
    }
    return false;
}

int cam_pipe_supported_res(char *out, size_t outlen)
{
    static const int tiers[][2] = {
        {160,120},{320,240},{480,320},{640,480},{800,600},{1280,720},
    };
    int n = 0;
    out[0] = 0;
    /* 先列标准档位（小→大，含虚拟档），再补 sensor 独有原生档 */
    for (size_t t = 0; t < sizeof(tiers) / sizeof(tiers[0]); t++) {
        if (!cam_pipe_res_supported(tiers[t][0], tiers[t][1])) continue;
        char item[16];
        snprintf(item, sizeof(item), "%dx%d", tiers[t][0], tiers[t][1]);
        if (strstr(out, item)) continue;
        if (n) strlcat(out, ",", outlen);
        strlcat(out, item, outlen);
        n++;
    }
    for (uint32_t i = 0; i < s_p.fmt_array.count && n < 12; i++) {
        const esp_cam_sensor_format_t *f = &s_p.fmt_array.format_array[i];
        if (!fmt_hw_encodable(f->format)) continue;
        char item[16];
        snprintf(item, sizeof(item), "%ux%u", f->width, f->height);
        if (strstr(out, item)) continue;
        if (n) strlcat(out, ",", outlen);
        strlcat(out, item, outlen);
        n++;
    }
    return n;
}

int cam_boost_level(void) { return s_p.boost_level; }

/* 实验台：在线改写时钟树补丁（-1 = 保持当前值） */
void cam_boost_clk_set(int c303b, int c303d, int c3824)
{
    if (c303b >= 0) s_boost_clk.c303b = c303b;
    if (c303d >= 0) s_boost_clk.c303d = c303d;
    if (c3824 >= 0) s_boost_clk.c3824 = c3824;
}
