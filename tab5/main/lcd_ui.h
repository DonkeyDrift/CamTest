/*
 * lcd_ui.h — Tab5 LCD 实时预览 + 运行参数面板 + 触屏调参（竖屏 720x1280 MIPI-DSI）
 *
 * 移植自 S31 lcd_ui（c0c104c），平台差异：
 *   - 显示：S31 RGB 800x480 → Tab5 MIPI-DSI 720x1280（原生竖屏），三代面板
 *     （ILI9881C+GT911 / ST7123 / ST7121）运行时按触摸 IC I2C 探测分版本
 *   - 预览解码：P4 JPEG 解码器与编码器是独立外设，解码经 jpeg_dec_share
 *     与 source_usb 互斥共享（S31 是编码空闲窗抢 codec_mutex）
 *   - 触屏动作与网页 POST /api/config 同路径（app_config_apply + persist）
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool  active;         /* LCD UI 已启动 */
    bool  preview_on;     /* 预览解码任务在跑 */
    float preview_fps;    /* 最近 1 s 实际解码帧率 */
    float dec_ms;         /* 单帧硬解耗时 EMA（ms） */
    uint32_t flush_cnt;   /* 面板 draw_bitmap 调用计数 */
    uint32_t flush_err;   /* 其中失败次数 */
} lcd_ui_stats_t;

esp_err_t lcd_ui_start(void);
lcd_ui_stats_t lcd_ui_stats(void);
bool lcd_ui_active(void);

#ifdef __cplusplus
}
#endif
