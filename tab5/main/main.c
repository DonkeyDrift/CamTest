/*
 * main.c — CamTest Tab5 入口：初始化顺序 + 看护逻辑（移植自 S31）
 *
 * 初始化顺序（失败即打印并重启，保证可观测）：
 *   NVS(+持久化配置载入) → 板级电源(USB VBUS/C6) → Wi-Fi(C6 协处理器，STA 失败回退
 *   SoftAP) → mDNS → 采集源（USB-UVC，按持久化配置恢复模式/档位/标定值）→
 *   流服务(:81 /stream /ws) → 管理 API(:80) → 指标/看护任务
 *
 * 看护（cam_watch）：每 5 s 检查
 *   - Wi-Fi 断连超过 60 s → 重启
 *   - USB 源 state 卡 STREAMING 但 20 s 帧冻结 → 重启（source_usb monitor 内
 *     另有 5s 帧停滞看门狗负责 teardown→重开，此为 monitor 自身卡死的兜底）
 *   - PSRAM 最低水位 < 512 kB → 打印告警
 */
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "source_if.h"
#include "wifi_net.h"
#include "stream_server.h"
#include "http_server.h"
#include "metrics.h"
#include "app_config.h"
#include "board_power.h"
#include "lcd_ui.h"
#include "cJSON.h"
#include "sdkconfig.h"

static const char *TAG = "main";

static void cam_watch_task(void *arg)
{
    uint32_t last_cap_frames = 0;
    int usb_stall_ticks = 0;
    int64_t wifi_down_since = 0;
    usb_state_t last_usb_state = USB_STATE_NO_DEVICE;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        src_stats_t *ps = src_if_stats();
        wifi_info_t *w = wifi_net_info();

        if (w->mode == WIFI_MODE_STA_M && !w->connected) {
            if (!wifi_down_since) wifi_down_since = esp_timer_get_time();
            if (esp_timer_get_time() - wifi_down_since > 60LL * 1000000) {
                ESP_LOGE(TAG, "Wi-Fi down >60s, reboot");
                esp_restart();
            }
        } else {
            wifi_down_since = 0;
        }

        /* USB 源状态播报 + 二重防护（S31 坑 #10：monitor 卡死时的兜底重启） */
        usb_state_t us = src_if_usb_state();
        if (us != last_usb_state) {
            static const char *names[] = { "disabled", "no-device", "ready", "streaming", "error" };
            ESP_LOGW(TAG, "USB 源状态：%s", names[us]);
            last_usb_state = us;
        }
        if (us == USB_STATE_STREAMING && ps->cap_frames == last_cap_frames) {
            if (++usb_stall_ticks >= 4) {
                ESP_LOGE(TAG, "USB state=streaming but cap frames frozen 20s (cap=%u), reboot",
                         (unsigned)ps->cap_frames);
                esp_restart();
            }
        } else {
            usb_stall_ticks = 0;
        }
        last_cap_frames = ps->cap_frames;

        if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) < 512 * 1024) {
            ESP_LOGW(TAG, "PSRAM low: %u kB", (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===== CamTest · M5Stack Tab5 (ESP32-P4+C6) 硬件 H.264 图传 =====");
    ESP_LOGI(TAG, "IDF: %s  |  build: %s %s", esp_get_idf_version(), __DATE__, __TIME__);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    app_config_init();

    /* 板级电源：Type-A VBUS（UVC 摄像头）先上电，给设备留枚举时间 */
    ESP_ERROR_CHECK(board_power_init());
    ESP_ERROR_CHECK(board_usb_vbus(true));

    ESP_ERROR_CHECK(wifi_net_start());

    /* 采集源抽象层：USB 后端（usb_host 库安装 + 热插拔监听） */
    ESP_ERROR_CHECK(src_if_init());

    /* 恢复 NVS 持久化配置（与 POST /api/config 同一 apply 实现）：
     * USB 模式/标定值/码率先登记（源未激活时仅"待生效"），首次开流即用 */
    const app_config_t *cfg = app_config_get();
    if (cfg->valid & APP_CFG_USB_MODE)
        src_if_usb_set_mode((usb_mode_t)cfg->usb_mode);
    if (cfg->valid & APP_CFG_INHERENT)
        src_if_usb_set_inherent_ms(cfg->inherent_ms);
    if (cfg->valid & APP_CFG_H264_KBPS)
        src_if_h264_set_bitrate_kbps(cfg->h264_kbps);

    cJSON *bootj = app_config_boot_json();
    if (bootj) {
        app_cfg_result_t r;
        if (!app_config_apply(bootj, &r)) {
            ESP_LOGW(TAG, "恢复持久化配置未完全成功：%s（当前源=%s）",
                     r.error[0] ? r.error : "unknown", src_if_source_name(src_if_current()));
        }
        cJSON_Delete(bootj);
    } else {
        src_if_switch(VIDEO_SOURCE_USB);
    }

    /* 网络服务 */
    wifi_net_mdns_start();
    ESP_ERROR_CHECK(stream_server_start(CONFIG_CAMTEST_STREAM_PORT));
    ESP_ERROR_CHECK(http_server_start());

    metrics_start();
    xTaskCreatePinnedToCore(cam_watch_task, "cam_watch", 3072, NULL, 3, NULL, 0);

    /* LCD 触屏控制台（探测失败自动跳过，不影响主流程） */
    esp_err_t lerr = lcd_ui_start();
    if (lerr != ESP_OK)
        ESP_LOGW(TAG, "LCD UI 未启用（%s）", esp_err_to_name(lerr));

    ESP_LOGI(TAG, "ready: http://%s.local  (IP %s)", CONFIG_CAMTEST_MDNS_HOSTNAME, wifi_net_info()->ip);
}
