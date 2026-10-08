/*
 * lcd_ui.h — LCD 实时预览 + 运行参数面板 + 触屏调参（4.3" 800x480 RGB + GT1151）
 *
 * 启动（app_main 末尾）：探测触摸芯片 → 初始化 BSP 显示/LVGL → 建 UI →
 * 起"预览任务"（frame_ring 订阅 + 硬件 JPEG 解码，限速、低优先级、只丢不排）
 * 与"配置任务"（与 POST /api/config 完全同路径：apply 成功才 persist）。
 *
 * 未接 LCD 子板时 lcd_ui_start() 探测失败并返回 NOT_FOUND，主流程不受影响；
 * CONFIG_CAMTEST_ENABLE_LCD=n 时全部编译为空操作。
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool  active;         /* LCD UI 已启动 */
    bool  preview_on;     /* 预览解码任务在跑（CAMTEST_LCD_PREVIEW_FPS>0 且缓冲分配成功） */
    float preview_fps;    /* 最近 1 s 实际解码帧率 */
    float dec_ms;         /* 单帧硬解耗时 EMA（ms） */
} lcd_ui_stats_t;

/* app_main 末尾调用一次；错误码仅用于日志（NOT_FOUND=未接 LCD 属正常） */
esp_err_t lcd_ui_start(void);

lcd_ui_stats_t lcd_ui_stats(void);
bool lcd_ui_active(void);   /* http status / 扫描 CSV 的 lcd_on 列 */

#ifdef __cplusplus
}
#endif
