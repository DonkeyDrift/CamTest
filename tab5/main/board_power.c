#include "board_power.h"
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "esp_io_expander.h"
#include "esp_io_expander_pi4ioe5v6408.h"

static const char *TAG = "board_pwr";

/* 引脚定义（与 esp-bsp m5stack_tab5 1.3.2 一致） */
#define BSP_I2C_SCL        (GPIO_NUM_32)
#define BSP_I2C_SDA        (GPIO_NUM_31)
#define BSP_WIFI_EN        (IO_EXPANDER_PIN_NUM_0)   /* 扩展器1 */
#define BSP_USB_EN         (IO_EXPANDER_PIN_NUM_3)   /* 扩展器1 */
#define BSP_CAMERA_EN      (IO_EXPANDER_PIN_NUM_6)   /* 扩展器0 */

static i2c_master_bus_handle_t s_i2c;
static esp_io_expander_handle_t s_exp0, s_exp1;   /* 0=LOW 地址，1=HIGH 地址 */

static esp_err_t pin_out(esp_io_expander_handle_t h, uint8_t pin, bool level)
{
    esp_err_t err = esp_io_expander_set_dir(h, pin, IO_EXPANDER_OUTPUT);
    if (err == ESP_OK) err = esp_io_expander_set_output_mode(h, pin, IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
    if (err == ESP_OK) err = esp_io_expander_set_level(h, pin, level);
    return err;
}

esp_err_t board_power_init(void)
{
    if (s_i2c) return ESP_OK;
    const i2c_master_bus_config_t cfg = {
        .i2c_port = 0,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &s_i2c), TAG, "i2c master");
    ESP_RETURN_ON_ERROR(esp_io_expander_new_i2c_pi4ioe5v6408(
        s_i2c, ESP_IO_EXPANDER_I2C_PI4IOE5V6408_ADDRESS_LOW, &s_exp0), TAG, "exp0");
    ESP_RETURN_ON_ERROR(esp_io_expander_new_i2c_pi4ioe5v6408(
        s_i2c, ESP_IO_EXPANDER_I2C_PI4IOE5V6408_ADDRESS_HIGH, &s_exp1), TAG, "exp1");
    ESP_LOGI(TAG, "I2C0 + 双 PI4IOE5V6408 扩展器就绪");
    return ESP_OK;
}

esp_err_t board_usb_vbus(bool on)
{
    ESP_RETURN_ON_ERROR(board_power_init(), TAG, "pwr init");
    esp_err_t err = pin_out(s_exp1, BSP_USB_EN, on);
    ESP_LOGI(TAG, "Type-A VBUS %s", on ? "ON" : "OFF");
    return err;
}

esp_err_t board_wifi_enable(bool on)
{
    ESP_RETURN_ON_ERROR(board_power_init(), TAG, "pwr init");
    esp_err_t err = pin_out(s_exp1, BSP_WIFI_EN, on);
    ESP_LOGI(TAG, "C6 Wi-Fi 电源 %s", on ? "ON（等其就绪后 hosted 才能通信）" : "OFF");
    return err;
}

esp_err_t board_camera_enable(bool on)
{
    ESP_RETURN_ON_ERROR(board_power_init(), TAG, "pwr init");
    esp_err_t err = pin_out(s_exp0, BSP_CAMERA_EN, on);
    ESP_LOGI(TAG, "MIPI 相机电源 %s", on ? "ON" : "OFF");
    return err;
}
