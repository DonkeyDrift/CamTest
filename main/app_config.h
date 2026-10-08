/*
 * app_config.h — NVS 持久化配置（重启/看门狗复位后恢复采集源、档位与光学标定值）
 *
 * 持久化语义（按"字段出现过"，valid 位掩码）：
 *   - 仅 /api/config 全部应用成功后才合并落盘（app_config_persist）；
 *   - 扫描等运行时临时改动不落盘，重启以本配置为准；
 *   - vts/boost 窗口为诊断参数，刻意不持久化。
 *
 * 启动恢复（main.c）：app_config_init 读入 → USB 模式/固有延迟先登记（源未激活
 * 时仅"待生效"）→ app_config_boot_json 构造完整 JSON → app_config_apply 统一应用，
 * 与 POST /api/config 走同一条代码路径，语义完全一致。
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CFG_MAGIC   0x43544d31   /* "CTM1"：结构布局变更时必须改 VERSION */
#define APP_CFG_VERSION 1

/* valid 位 */
#define APP_CFG_SRC      (1u << 0)    /* source：dvp/usb */
#define APP_CFG_USB_MODE (1u << 1)    /* usb_mode：passthrough/reencode */
#define APP_CFG_RES      (1u << 2)    /* res WxH */
#define APP_CFG_QUALITY  (1u << 3)    /* JPEG 质量 */
#define APP_CFG_FPS      (1u << 4)    /* fps_limit（>0 才可设置） */
#define APP_CFG_OVERLAY  (1u << 5)    /* 毫秒计数叠加开关 */
#define APP_CFG_MBPS     (1u << 6)    /* 码率软上限 */
#define APP_CFG_INHERENT (1u << 7)    /* 光学标定 ①（usb_inherent_ms） */
#define APP_CFG_HIFPS    (1u << 8)    /* DVP 高帧率档位 0..4 */

/* NVS blob（namespace "camtest"，key "cfg"）；布局变更 → bump APP_CFG_VERSION */
typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  _rsvd0[3];
    uint32_t valid;
    uint8_t  source;        /* video_source_t：0=dvp 1=usb */
    uint8_t  usb_mode;      /* usb_mode_t */
    uint8_t  quality;
    uint8_t  overlay;
    uint16_t w, h;          /* APP_CFG_RES 时有效 */
    int16_t  fps_limit;     /* APP_CFG_FPS 时有效 */
    int16_t  hifps;         /* APP_CFG_HIFPS 时有效：0=关 1..4 档 */
    uint16_t _rsvd1;
    float    target_mbps;
    float    inherent_ms;   /* APP_CFG_INHERENT 时有效；-1=清除标定 */
} app_config_t;

typedef struct {
    bool ok;         /* 全部步骤成功（含重建/重协商） */
    bool restarted;  /* 触发了重建/重协商 */
    char error[64];
} app_cfg_result_t;

/* app_main：nvs_flash_init 之后调用一次（坏记录/无记录 → 全默认） */
void app_config_init(void);
const app_config_t *app_config_get(void);

/* 应用一组配置字段（部分更新语义，与 POST /api/config 同一实现） */
bool app_config_apply(const cJSON *j, app_cfg_result_t *r);

/* 仅在 app_config_apply 成功后调用：按 POST 中出现的字段合并落盘 */
void app_config_persist(const cJSON *j);

/* 启动恢复用 JSON：source 恒发（无记录时默认 dvp，等价旧固件行为），其余按 valid 位 */
cJSON *app_config_boot_json(void);

#ifdef __cplusplus
}
#endif
