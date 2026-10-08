/*
 * app_config.h — NVS 持久化配置（Tab5 版；重启/看门狗复位后恢复源、档位与标定值）
 *
 * 持久化语义（按"字段出现过"，valid 位掩码）：
 *   - 仅 /api/config 全部应用成功后才合并落盘（app_config_persist）；
 *   - 运行时临时改动不落盘，重启以本配置为准。
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CFG_MAGIC   0x43544d32   /* "CTM2"：Tab5 布局（与 S31 的 CTM1 区分） */
#define APP_CFG_VERSION 1

/* valid 位 */
#define APP_CFG_SRC       (1u << 0)   /* source：dvp/usb */
#define APP_CFG_USB_MODE  (1u << 1)   /* usb_mode：passthrough/reencode/h264 */
#define APP_CFG_RES       (1u << 2)   /* res WxH */
#define APP_CFG_QUALITY   (1u << 3)   /* JPEG 质量（reencode） */
#define APP_CFG_FPS       (1u << 4)   /* fps_limit（>0 才可设置） */
#define APP_CFG_OVERLAY   (1u << 5)   /* 毫秒计数叠加开关 */
#define APP_CFG_MBPS      (1u << 6)   /* 码率软上限 */
#define APP_CFG_INHERENT  (1u << 7)   /* 光学标定 ①（usb_inherent_ms） */
#define APP_CFG_H264_KBPS (1u << 8)   /* H.264 目标码率 kbps */

/* NVS blob（namespace "camtest"，key "cfg"） */
typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  _rsvd0[3];
    uint32_t valid;
    uint8_t  source;        /* video_source_t */
    uint8_t  usb_mode;      /* usb_mode_t */
    uint8_t  quality;
    uint8_t  overlay;
    uint16_t w, h;
    int16_t  fps_limit;
    uint16_t _rsvd1;
    float    target_mbps;
    float    inherent_ms;   /* -1=未标定 */
    uint32_t h264_kbps;
} app_config_t;

typedef struct {
    bool ok;
    bool restarted;
    char error[64];
} app_cfg_result_t;

void app_config_init(void);
const app_config_t *app_config_get(void);
bool app_config_apply(const cJSON *j, app_cfg_result_t *r);
void app_config_persist(const cJSON *j);
cJSON *app_config_boot_json(void);

#ifdef __cplusplus
}
#endif
