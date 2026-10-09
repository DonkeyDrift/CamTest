#include "jpeg_dec_share.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "jdec_sh";

static jpeg_decoder_handle_t s_dec;
static SemaphoreHandle_t s_lock;

esp_err_t jpeg_dec_share_process(const jpeg_decode_cfg_t *cfg,
                                 const uint8_t *in, uint32_t in_len,
                                 uint8_t *out, uint32_t out_cap,
                                 uint32_t *out_size)
{
    if (!s_lock) {
        SemaphoreHandle_t m = xSemaphoreCreateMutex();
        if (!m) return ESP_ERR_NO_MEM;
        if (!s_lock) s_lock = m;   /* 双检查（首个使用者并发） */
    }
    if (!s_dec) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (!s_dec) {
            jpeg_decode_engine_cfg_t ecfg = { .intr_priority = 0, .timeout_ms = 500 };
            esp_err_t err = jpeg_new_decoder_engine(&ecfg, &s_dec);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "解码引擎创建失败 %s", esp_err_to_name(err));
                xSemaphoreGive(s_lock);
                return err;
            }
            ESP_LOGI(TAG, "共享 JPEG 解码引擎就绪");
        }
        xSemaphoreGive(s_lock);
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = jpeg_decoder_process(s_dec, cfg, in, in_len, out, out_cap, out_size);
    xSemaphoreGive(s_lock);
    return err;
}
