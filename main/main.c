/*
 * main.c — CamTest 入口：初始化顺序 + 看护逻辑
 *
 * 初始化顺序（失败即打印并重启，保证可观测）：
 *   NVS(+持久化配置载入) → Wi-Fi(STA，失败回退 SoftAP) → mDNS → 摄像头管线(按持久化
 *   配置恢复源/档位/标定值，无记录默认 DVP) → 流服务(:81 /stream /ws) → 管理 API(:80)
 *   → UDP 分片(可选) → 指标/串口CSV 任务
 *
 * 看护（cam_watch）：每 5 s 检查
 *   - Wi-Fi 断连超过 60 s → 重启（STA 模式下 IP 全无时图传无意义）
 *   - 采集有帧但 15 s 无编码输出 → 自动重建一次链路；连续 3 次失败 → 重启
 *   - USB 源 state 卡 STREAMING 但 20 s 帧冻结 → 重启（source_usb monitor 内
 *     另有 5s 帧停滞看门狗负责 teardown→重开，此为 monitor 自身卡死的兜底）
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
#include "app_config.h"
#include "lcd_ui.h"
#include "cJSON.h"
#include "sdkconfig.h"

static const char *TAG = "main";

static void cam_watch_task(void *arg)
{
    uint32_t last_out_frames = 0;
    uint32_t last_cap_frames = 0;
    int out_stall = 0;
    int usb_stall_ticks = 0;   /* USB 源：STREAMING 态帧冻结拍数（cam_watch 每 5s 一拍） */
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
                if (scan_ctrl_state() == SCAN_RUNNING) {
                    /* 扫描中重启会毁掉整轮矩阵：推迟 60 s 窗口，STA 自动重连通常
                     * 在此期间恢复；扫描结束后仍断才重启 */
                    ESP_LOGW(TAG, "Wi-Fi down >60s but scan running, postpone reboot");
                    wifi_down_since = esp_timer_get_time();
                } else {
                    ESP_LOGE(TAG, "Wi-Fi down >60s, reboot");
                    esp_restart();
                }
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
                        if (scan_ctrl_state() == SCAN_RUNNING) {
                            /* 扫描中不重启（毁整轮矩阵）：清零重新累计宽限，重建仍每轮尝试 */
                            ESP_LOGW(TAG, "rebuild failed repeatedly but scan running, postpone reboot");
                            out_stall = 0;
                        } else {
                            ESP_LOGE(TAG, "rebuild failed repeatedly, reboot");
                            esp_restart();
                        }
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
            /* 二重防护：source_usb monitor 自带帧停滞看门狗（5s 无帧 teardown 重开、
             * 6 轮不愈重启），但 monitor 任务自身若卡死则无人触发——state 卡
             * STREAMING 且 cap_frames 连续 4 拍（20s）零增长 → 重启兜底 */
            if (us == USB_STATE_STREAMING && ps->cap_frames == last_cap_frames) {
                if (++usb_stall_ticks >= 4) {
                    if (scan_ctrl_state() == SCAN_RUNNING) {
                        /* 扫描中不重启（毁整轮矩阵）：monitor 的 5s 停滞看门狗仍负责
                         * teardown→重开自愈，此兜底推迟并重新累计 20 s 宽限 */
                        ESP_LOGW(TAG, "cap frames frozen 20s but scan running, postpone reboot");
                        usb_stall_ticks = 0;
                    } else {
                        ESP_LOGE(TAG, "USB state=streaming but cap frames frozen 20s (cap=%u), reboot",
                                 (unsigned)ps->cap_frames);
                        esp_restart();
                    }
                }
            } else {
                usb_stall_ticks = 0;
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

    /* 内部 DMA 池水位逐阶段打点（RGB 面板 DMA 链表节点必须在内部 DMA RAM） */
#define DMA_MARK(tag) ESP_LOGI(TAG, "[%s] dma_free=%u 最大块=%u internal=%u", tag, \
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA), \
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA), \
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT))
    DMA_MARK("boot");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    app_config_init();   /* 载入持久化配置（采集源/档位/光学标定值），坏记录自动回默认 */

    /* Wi-Fi（STA 默认，失败回退 SoftAP） */
    ESP_ERROR_CHECK(wifi_net_start());
    DMA_MARK("wifi");

    /* 采集源抽象层：DVP 后端初始化（BSP 上电 + sensor 侦测）+ USB 后端（热插拔监听） */
    ESP_ERROR_CHECK(src_if_init());
    DMA_MARK("src_if_init");

    /* LCD 必须在摄像头拉流前初始化：RGB 面板的帧缓冲 DMA 链表节点（约 8KB）
     * 必须在内部 DMA RAM 分配，而 camera_start 一个阶段就要吃掉 ~100KB，
     * 放到启动末尾会因 DMA 池耗尽触发 BSP abort（CONFIG_BSP_ERROR_CHECK=y）。
     * 此点位 src_if 已就绪、Wi-Fi 已连（面板可显示 IP），DMA 池尚有 128KB；
     * 后续阶段的普通 internal 分配会自动溢出到非 DMA 的 32KB 内部区。 */
    lcd_ui_start();   /* 未接 LCD 子板时探测失败返回 NOT_FOUND，仅日志告警 */
    DMA_MARK("before_uvc");   /* = src_if_init − LCD 阶段1 消耗；UVC 开流前水位 */

    /* 恢复 NVS 持久化配置（与 POST /api/config 同一 apply 实现）：
     * USB 模式/固有延迟先登记——源未激活时仅"待生效"，让首次开流就用上标定值，
     * 免去"先默认开流、再同步重协商"的一次抖动。无记录时 boot_json 仅含
     * source=dvp，行为与旧固件的 src_if_switch(VIDEO_SOURCE_DVP) 完全一致。 */
    const app_config_t *cfg = app_config_get();
    if (cfg->valid & APP_CFG_USB_MODE)
        src_if_usb_set_mode((usb_mode_t)cfg->usb_mode);
    if (cfg->valid & APP_CFG_INHERENT)
        src_if_usb_set_inherent_ms(cfg->inherent_ms);

    cJSON *bootj = app_config_boot_json();
    if (bootj) {
        app_cfg_result_t r;
        if (!app_config_apply(bootj, &r)) {
            /* 源切换失败时 src_if_switch 内部已回滚（USB 未插入 → 回落 DVP）；
             * 其余字段失败仅部分恢复，照常启动可观测 */
            ESP_LOGW(TAG, "恢复持久化配置未完全成功：%s（当前源=%s）",
                     r.error[0] ? r.error : "unknown", src_if_source_name(src_if_current()));
        }
        cJSON_Delete(bootj);
    } else {
        src_if_switch(VIDEO_SOURCE_DVP);
    }
    DMA_MARK("after_uvc");    /* before_uvc − 本值 = UVC 开流实际 DMA 成本 */
    ESP_ERROR_CHECK(scan_ctrl_init());
    DMA_MARK("camera_start");

    /* 网络服务 */
    wifi_net_mdns_start();
    ESP_ERROR_CHECK(stream_server_start(CONFIG_CAMTEST_STREAM_PORT));
    ESP_ERROR_CHECK(http_server_start());
    DMA_MARK("net_svc");
#if CONFIG_CAMTEST_ENABLE_UDP
    ESP_ERROR_CHECK(udp_push_start(CONFIG_CAMTEST_UDP_PORT));
#endif

    metrics_start();
    xTaskCreatePinnedToCore(cam_watch_task, "cam_watch", 3072, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "ready: http://%s.local  (IP %s)", CONFIG_CAMTEST_MDNS_HOSTNAME, wifi_net_info()->ip);
}
