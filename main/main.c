/*
 * main.c — CamTest 入口：初始化顺序 + 看护逻辑
 *
 * 初始化顺序（失败即打印并重启，保证可观测）：
 *   NVS → Wi-Fi(STA，失败回退 SoftAP) → mDNS → 摄像头管线(侦测+采集+编码) →
 *   流服务(:81 /stream /ws) → 管理 API(:80) → UDP 分片(可选) → 指标/串口CSV 任务
 *
 * 看护（cam_watch）：每 5 s 检查
 *   - Wi-Fi 断连超过 60 s → 重启（STA 模式下 IP 全无时图传无意义）
 *   - 采集有帧但 15 s 无编码输出 → 自动重建一次链路；连续 3 次失败 → 重启
 *   - PSRAM 最低水位 < 512 kB → 打印告警（不重启，仅 UI 可见）
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
#include "scan_ctrl.h"
#include "sdkconfig.h"

static const char *TAG = "main";

static void cam_watch_task(void *arg)
{
    uint32_t last_out_frames = 0;
    uint32_t last_cap_frames = 0;
    int out_stall = 0;
    int64_t wifi_down_since = 0;
    usb_state_t last_usb_state = USB_STATE_NO_DEVICE;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        src_stats_t *ps = src_if_stats();
        src_info_t *ci = src_if_info();
        wifi_info_t *w = wifi_net_info();

        /* Wi-Fi 看护 */
        if (w->mode == WIFI_MODE_STA_M && !w->connected) {
            if (!wifi_down_since) wifi_down_since = esp_timer_get_time();
            if (esp_timer_get_time() - wifi_down_since > 60LL * 1000000) {
                ESP_LOGE(TAG, "Wi-Fi down >60s, reboot");
                esp_restart();
            }
        } else {
            wifi_down_since = 0;
        }

        if (ci->source == VIDEO_SOURCE_DVP) {
            /* DVP 编码停滞看护：采集在走但编码卡死 → 重建链路（USB 源有自己的 monitor） */
            if (ps->cap_frames > last_cap_frames && ps->out_frames == last_out_frames) {
                out_stall++;
                if (out_stall == 3) {   /* ~15 s 无输出 */
                    ESP_LOGE(TAG, "encode stalled (cap=%u out=%u) → rebuild pipeline",
                             (unsigned)ps->cap_frames, (unsigned)ps->out_frames);
                    esp_err_t err = src_if_apply(0, 0, 0, 0);
                    out_stall = (err == ESP_OK) ? 0 : out_stall;
                    if (err != ESP_OK && ++out_stall >= 6) {
                        ESP_LOGE(TAG, "rebuild failed repeatedly, reboot");
                        esp_restart();
                    }
                }
            } else {
                out_stall = 0;
            }
        } else {
            /* USB 源状态播报（自动重连由 source_usb monitor 负责） */
            usb_state_t us = src_if_usb_state();
            if (us != last_usb_state) {
                static const char *names[] = { "disabled", "no-device", "ready", "streaming", "error" };
                ESP_LOGW(TAG, "USB 源状态：%s%s", names[us],
                         us == USB_STATE_ERROR ? "（反复打开失败？检查供电：Type-C 口补电）" : "");
                last_usb_state = us;
            }
        }
        last_out_frames = ps->out_frames;
        last_cap_frames = ps->cap_frames;

        if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) < 512 * 1024) {
            ESP_LOGW(TAG, "PSRAM low: %u kB", (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===== CamTest · ESP32-S31-Korvo-1 图传性能闭环 =====");
    ESP_LOGI(TAG, "IDF: %s  |  build: %s %s", esp_get_idf_version(), __DATE__, __TIME__);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Wi-Fi（STA 默认，失败回退 SoftAP） */
    ESP_ERROR_CHECK(wifi_net_start());

    /* 采集源抽象层：DVP 后端初始化（BSP 上电 + sensor 侦测）+ USB 后端（热插拔监听），
     * 默认源 DVP（与原行为一致）；USB 切换经 /api/config 或 UI 下拉框 */
    ESP_ERROR_CHECK(src_if_init());
    src_if_switch(VIDEO_SOURCE_DVP);   /* 启动 DVP 采集/编码任务 */
    ESP_ERROR_CHECK(scan_ctrl_init());

    /* 网络服务 */
    wifi_net_mdns_start();
    ESP_ERROR_CHECK(stream_server_start(CONFIG_CAMTEST_STREAM_PORT));
    ESP_ERROR_CHECK(http_server_start());
#if CONFIG_CAMTEST_ENABLE_UDP
    ESP_ERROR_CHECK(udp_push_start(CONFIG_CAMTEST_UDP_PORT));
#endif

    metrics_start();
    xTaskCreatePinnedToCore(cam_watch_task, "cam_watch", 3072, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "ready: http://%s.local  (IP %s)", CONFIG_CAMTEST_MDNS_HOSTNAME, wifi_net_info()->ip);
}
