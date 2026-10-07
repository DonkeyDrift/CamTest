/*
 * source_if.h — 统一采集源接口（U1）
 *
 * 上层（编码/传输/度量/扫描/UI）只依赖本头文件，不感知底层是 DVP 还是 USB。
 *   - source_dvp.c：薄适配层，包装既有 camera_pipeline.c（零改动复用）
 *   - source_usb.c：usb_host_uvc 2.5.2 原生 API 后端（为何不走 esp_video 的
 *     /dev/video40 V4L2 封装见 README「可行性结论」——其断线去初始化路径存在
 *     资源泄漏与状态卡死缺陷，不满足拔插 3 次不卡死的验收要求）
 *
 * 切换语义（src_if_switch）：先停当前源 → 启动目标源；失败则回滚重启原源，
 * 返回错误码，绝不处于"两源皆停"的不可观测状态。
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

/* USB 摄像头工作模式（U2） */
typedef enum {
    USB_MODE_NONE = 0,        /* 非 USB 源（DVP），CSV 填 n/a */
    USB_MODE_PASSTHROUGH,     /* MJPEG 直通：不解码不编码，quality 不可控 */
    USB_MODE_REENCODE,        /* YUY2 → S31 硬件 JPEG：quality 可控 */
} usb_mode_t;

/* t_capture_us 的语义（U4） */
typedef enum {
    TS_MEANING_SENSOR_OUT = 0,   /* DVP：DQBUF 返回即传感器输出时刻，链路可控 */
    TS_MEANING_FRAME_ARRIVAL,    /* USB：完整一帧到达 ESP32 的时刻（含摄像头内部
                                    曝光/ISP/编码/USB 传输的不可见延迟） */
} ts_meaning_t;

#define SRC_SENSOR_NAME_MAX 32
#define SRC_USB_NAME_MAX    24
#define SRC_FMT_NAME_MAX    64

typedef struct {
    video_source_t source;
    uint16_t w, h;                 /* 输出（进入传输层）分辨率 */
    bool     needs_encode;         /* true=经硬件 JPEG 编码后出帧；false=MJPEG 直通 */
    bool     scaled;               /* 输出帧是否由更大原生帧经 ESP32 缩放/裁剪得到 */
    uint16_t native_w, native_h;   /* scaled 时的原生分辨率 */
    uint8_t  quality;              /* needs_encode 时 1..100；直通模式恒 0（不可控，勿伪造） */
    usb_mode_t   usb_mode;
    ts_meaning_t ts_meaning;
    int      nominal_fps;          /* 描述符/格式表标称帧率 */
    int      fps_limit;            /* 软件帧率上限（0=不限） */
    char     pix_fmt_str[8];       /* 实际出帧像素格式（"YUYV"/"JPEG"/"RGB565"…；直通=摄像头内部 JPEG） */
    char     sensor_name[SRC_SENSOR_NAME_MAX];  /* "OV3660"/"SC101IOT"/USB "vid:pid" */
    char     usb_device_name[SRC_USB_NAME_MAX]; /* 非 USB 源为 "" */
    char     fmt_name[SRC_FMT_NAME_MAX];
} src_info_t;

typedef struct {
    uint32_t cap_frames;           /* 采集到帧数 */
    uint32_t out_frames;           /* 发布到帧环帧数（编码完成或直通转发） */
    uint32_t cap_drops;            /* 采集侧丢帧 */
    uint32_t out_drops;            /* 输出侧丢帧（编码忙/环满） */
    uint64_t out_bytes;            /* 输出字节累计（码率口径，含直通） */
    uint64_t proc_acc_us;          /* 输出处理耗时累计：DVP/重编码=编码耗时，直通=发布拷贝耗时 */
} src_stats_t;

/* ---------- 生命周期 ---------- */
esp_err_t src_if_init(void);                    /* app_main 早期一次；注册两个后端 */
esp_err_t src_if_switch(video_source_t want);   /* 运行时热切换（含回滚） */
video_source_t src_if_current(void);

/* ---------- 当前源操作（对应原 cam_pipe_* 接口，供上层统一调用） ---------- */
esp_err_t src_if_apply(int w, int h, uint8_t quality, int fps_limit);
esp_err_t src_if_set_quality(uint8_t q);
void      src_if_set_fps_limit(int fps);
void      src_if_set_overlay(bool on);
bool      src_if_overlay(void);
src_info_t  *src_if_info(void);
src_stats_t *src_if_stats(void);
frame_ring_t *src_if_ring(void);
int      src_if_supported_res(char *out, size_t outlen);   /* 当前源支持档，逗号分隔 "WxH" */
bool     src_if_res_supported(int w, int h);

/* ---------- USB 专用查询/控制（UI 与 CSV 用） ---------- */
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
    USB_STATE_DISABLED = 0,   /* 编译期未启用 */
    USB_STATE_NO_DEVICE,      /* 未插入/被拔出 */
    USB_STATE_DEVICE_READY,   /* 已枚举（可列出档位），未推流 */
    USB_STATE_STREAMING,      /* 推流中 */
    USB_STATE_ERROR,          /* 打开/协商失败，重试中（典型诱因：供电不足） */
} usb_state_t;

usb_state_t   src_if_usb_state(void);
usb_mode_t    src_if_usb_mode(void);               /* 当前配置的 USB 模式 */
esp_err_t     src_if_usb_set_mode(usb_mode_t m);   /* USB 激活时即时重协商，否则仅生效于下次启动 */
uint32_t      src_if_usb_disconnects(void);        /* 掉线计数（U5） */
const char   *src_if_usb_device_name(void);        /* "vid:pid"，未插入为 "" */
int           src_if_usb_get_tiers(usb_tier_t *out, int max);  /* 描述符枚举的原生档位 */
float         src_if_usb_inherent_ms(void);        /* 摄像头内部固有延迟估算；未标定 = -1 */
void          src_if_usb_set_inherent_ms(float ms);/* 光学闭环标定后写入（/api/config） */

const char *src_if_source_name(video_source_t s);  /* "dvp" / "usb" */
const char *src_if_usb_mode_name(usb_mode_t m);    /* "passthrough"/"reencode"/"n/a" */
const char *src_if_ts_meaning_name(ts_meaning_t m);/* "sensor_out"/"frame_arrival" */

#ifdef __cplusplus
}
#endif
