/*
 * source_if.h — 统一采集源接口（移植自 S31 CamTest，Tab5 版）
 *
 * 上层（传输/度量/UI）只依赖本头文件。Tab5 当前后端：
 *   - source_usb.c：usb_host_uvc 原生 API；passthrough（MJPEG 直通）/
 *     h264（MJPEG→硬解→P4 硬件 H.264）/ reencode（MJPEG→硬解→P4 硬件 JPEG）三模式
 *   - DVP（Tab5 板载 SC202CS MIPI 相机）：后续接入（esp_video），本头先留枚举位
 *
 * 帧环由本层创建并终身持有（init 后永不销毁），跨源切换持续有效。
 *
 * 切换语义（src_if_switch）：先停当前源 → 启动目标源；失败返回错误码。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "frame_ring.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VIDEO_SOURCE_DVP = 0,
    VIDEO_SOURCE_USB = 1,
} video_source_t;

/* USB 摄像头工作模式 */
typedef enum {
    USB_MODE_NONE = 0,        /* 非 USB 源 */
    USB_MODE_PASSTHROUGH,     /* MJPEG 直通：不解码不编码，quality 不可控 */
    USB_MODE_REENCODE,        /* MJPEG→硬件解码→P4 硬件 JPEG：quality 可控（S31 同语义，OSD/标定） */
    USB_MODE_H264,            /* MJPEG→硬件解码→P4 硬件 H.264：码率可控（默认，最低时延带宽比） */
} usb_mode_t;

/* t_capture_us 的语义 */
typedef enum {
    TS_MEANING_SENSOR_OUT = 0,   /* DVP：DQBUF 返回即传感器输出时刻 */
    TS_MEANING_FRAME_ARRIVAL,    /* USB：完整一帧到达 ESP32 的时刻 */
} ts_meaning_t;

#define SRC_SENSOR_NAME_MAX 32
#define SRC_USB_NAME_MAX    24
#define SRC_FMT_NAME_MAX    64

typedef struct {
    video_source_t source;
    uint16_t w, h;                 /* 输出（进入传输层）分辨率 */
    bool     needs_encode;         /* true=经硬件编码后出帧；false=MJPEG 直通 */
    bool     scaled;               /* 输出帧是否由更大原生帧经 ESP32 缩放/裁剪得到 */
    uint16_t native_w, native_h;   /* scaled 时的原生分辨率 */
    uint8_t  quality;              /* reencode 时 1..100；直通/H264 恒 0 */
    usb_mode_t   usb_mode;
    ts_meaning_t ts_meaning;
    int      nominal_fps;          /* 描述符/格式表标称帧率 */
    int      fps_limit;            /* 软件帧率上限（0=不限） */
    char     pix_fmt_str[16];      /* 实际出帧格式（"JPEG"/"H264"/…） */
    char     sensor_name[SRC_SENSOR_NAME_MAX];
    char     usb_device_name[SRC_USB_NAME_MAX];
    char     fmt_name[SRC_FMT_NAME_MAX];
} src_info_t;

typedef struct {
    uint32_t cap_frames;           /* 采集到帧数 */
    uint32_t out_frames;           /* 发布到帧环帧数 */
    uint32_t cap_drops;            /* 采集侧丢帧 */
    uint32_t out_drops;            /* 输出侧丢帧（编码忙/环满） */
    uint64_t out_bytes;            /* 输出字节累计 */
    uint64_t proc_acc_us;          /* 输出处理耗时累计（重编码=解码+编码耗时） */
} src_stats_t;

/* ---------- 生命周期 ---------- */
esp_err_t src_if_init(void);                    /* app_main 早期一次 */
esp_err_t src_if_switch(video_source_t want);   /* 运行时热切换 */
video_source_t src_if_current(void);

/* ---------- 当前源操作 ---------- */
esp_err_t src_if_apply(int w, int h, uint8_t quality, int fps_limit);
esp_err_t src_if_set_quality(uint8_t q);
void      src_if_set_fps_limit(int fps);
void      src_if_set_overlay(bool on);
bool      src_if_overlay(void);
src_info_t  *src_if_info(void);
src_stats_t *src_if_stats(void);
frame_ring_t *src_if_ring(void);
int      src_if_supported_res(char *out, size_t outlen);
bool     src_if_res_supported(int w, int h);

/* ---------- USB 专用查询/控制 ---------- */
#define USB_TIER_MAX 16
#define USB_TIER_FPS_MAX 6

typedef struct {
    char     fmt[8];           /* "MJPEG" / "YUY2" */
    uint16_t w, h;
    int      fps[USB_TIER_FPS_MAX];
    int      fps_n;
    bool     mjpeg;
} usb_tier_t;

typedef enum {
    USB_STATE_DISABLED = 0,
    USB_STATE_NO_DEVICE,
    USB_STATE_DEVICE_READY,
    USB_STATE_STREAMING,
    USB_STATE_ERROR,
} usb_state_t;

usb_state_t   src_if_usb_state(void);
usb_mode_t    src_if_usb_mode(void);
esp_err_t     src_if_usb_set_mode(usb_mode_t m);
uint32_t      src_if_usb_disconnects(void);
const char   *src_if_usb_device_name(void);
int           src_if_usb_get_tiers(usb_tier_t *out, int max);
float         src_if_usb_inherent_ms(void);
void          src_if_usb_set_inherent_ms(float ms);
esp_err_t     src_if_h264_set_bitrate_kbps(uint32_t kbps);  /* H.264 码率在线调整 */
uint32_t      src_if_h264_kbps(void);                       /* 当前 H.264 目标码率 */

const char *src_if_source_name(video_source_t s);
const char *src_if_usb_mode_name(usb_mode_t m);
const char *src_if_ts_meaning_name(ts_meaning_t m);

#ifdef __cplusplus
}
#endif
