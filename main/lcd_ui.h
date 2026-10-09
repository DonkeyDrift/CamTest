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
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool  active;         /* LCD UI 已启动 */
    bool  preview_on;     /* 预览解码任务在跑（CAMTEST_LCD_PREVIEW_FPS>0 且缓冲分配成功） */
    bool  pv_on;          /* 屏上预览开关（关=任务深睡，解码/缩放/翻页全停省 CPU） */
    float preview_fps;    /* 最近 1 s 实际解码帧率 */
    float dec_ms;         /* 单帧硬解耗时 EMA（ms） */
    uint32_t flush_cnt;   /* 面板 draw_bitmap 调用计数（LVGL flush + 测试图案） */
    uint32_t flush_err;   /* 其中失败次数（>0 = 帧缓冲写入有问题） */
} lcd_ui_stats_t;

/* app_main 末尾调用一次；错误码仅用于日志（NOT_FOUND=未接 LCD 属正常） */
esp_err_t lcd_ui_start(void);

/* 直写测试图案（绕过 LVGL）停留 hold_ms 后自动恢复 UI；
 * 黑屏诊断用：彩条可见 = 面板/时序/背光正常，问题在 LVGL 侧 */
void lcd_ui_test_pattern(int hold_ms);

/* 预览开关（屏上 PV 按钮 /api/lcdpv 共用）：关=预览任务注销帧环深度休眠，
 * 解码/缩放/翻页全停省 CPU；LCD 本地开关不动流水线，不落 NVS */
void lcd_ui_pv_set(bool on);

lcd_ui_stats_t lcd_ui_stats(void);
bool lcd_ui_active(void);   /* http status / 扫描 CSV 的 lcd_on 列 */

#ifdef __cplusplus
}
#endif
