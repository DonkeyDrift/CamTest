/*
 * board_power.c/h — Tab5 板级电源使能（最小复刻，不引入完整 BSP）
 *
 * Tab5 的外设电源不直连 P4 GPIO，而是挂两颗 PI4IOE5V6408 IO 扩展器
 * （I2C0：SDA=GPIO31 SCL=GPIO32，内部上拉）：
 *   扩展器0（7bit 地址 LOW=0x43）：LCD_EN(4) TOUCH_EN(5) CAMERA_EN(6) SPEAKER_EN(1)
 *   扩展器1（7bit 地址 HIGH=0x44）：WIFI_EN(0，C6 电源) USB_EN(3，Type-A VBUS)
 * 依据：esp-bsp m5stack_tab5 1.3.x bsp_feature_en.c / m5stack_tab5.h。
 *
 * 注意：C6 上电到 SDIO 可通信需要一点时间（hosted 侧还有复位脚 GPIO15），
 * wifi 使能后由 wifi_net 自行重试。
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t board_power_init(void);            /* I2C + 两颗扩展器；幂等 */
esp_err_t board_usb_vbus(bool on);           /* Type-A VBUS 供电（UVC 摄像头） */
esp_err_t board_wifi_enable(bool on);        /* C6 协处理器电源 */
esp_err_t board_camera_enable(bool on);      /* MIPI 相机电源（DVP 源用） */
esp_err_t board_lcd_enable(bool on);         /* LCD 电源（exp0.4，开漏上拉特殊序列） */
esp_err_t board_touch_enable(bool on);       /* 触摸电源（exp0.5） */
struct i2c_master_bus_ctx;
void *board_i2c_handle(void);                /* I2C0 总线句柄（相机 SCCB 用；i2c_master_bus_handle_t） */

#ifdef __cplusplus
}
#endif
