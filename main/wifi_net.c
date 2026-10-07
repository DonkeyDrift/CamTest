/* wifi_net.c — 见 wifi_net.h */
#include "wifi_net.h"
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "lwip/ip4_addr.h"
#include <stdio.h>
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

static const char *TAG = "wifi_net";

#define CONNECT_TIMEOUT_MS 15000

static EventGroupHandle_t s_ev;
#define EV_STA_GOT_IP (1 << 0)
#define EV_AP_STARTED (1 << 1)
static wifi_info_t s_info;
static volatile wifi_op_mode_t s_desired;

static void ip_event_cb(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base != IP_EVENT) return;
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_info.ip, sizeof(s_info.ip), IPSTR, IP2STR(&e->ip_info.ip));
        s_info.connected = true;
        xEventGroupSetBits(s_ev, EV_STA_GOT_IP);
        ESP_LOGI(TAG, "STA got IP: %s", s_info.ip);
    } else if (id == IP_EVENT_ASSIGNED_IP_TO_CLIENT) {
        s_info.connected = true;
    }
}

static void wifi_event_cb(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base != WIFI_EVENT) return;
    switch (id) {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        s_info.connected = false;
        if (s_desired == WIFI_MODE_STA_M) {
            ESP_LOGW(TAG, "STA disconnected, retrying...");
            esp_wifi_connect();   /* 自动重连；长时间失败由 main 的看护逻辑处理 */
        }
        break;
    case WIFI_EVENT_AP_START:
        xEventGroupSetBits(s_ev, EV_AP_STARTED);
        s_info.connected = true;
        break;
    case WIFI_EVENT_AP_STACONNECTED:
        s_info.connected = true;
        break;
    default:
        break;
    }
}

static esp_err_t start_one(wifi_op_mode_t mode)
{
    xEventGroupClearBits(s_ev, EV_STA_GOT_IP | EV_AP_STARTED);
    memset(&s_info, 0, sizeof(s_info));
    s_info.mode = mode;
    wifi_config_t wc = { 0 };
    esp_err_t err;

    if (mode == WIFI_MODE_STA_M) {
        strlcpy((char *)wc.sta.ssid, CONFIG_CAMTEST_WIFI_STA_SSID, sizeof(wc.sta.ssid));
        strlcpy((char *)wc.sta.password, CONFIG_CAMTEST_WIFI_STA_PASSWORD, sizeof(wc.sta.password));
        wc.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
        err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (err) return err;
        err = esp_wifi_set_config(WIFI_IF_STA, &wc);
        if (err) return err;
        err = esp_wifi_start();
        if (err) return err;
        esp_wifi_set_ps(WIFI_PS_NONE);   /* start 后再设一次，确保关联后省电保持关闭 */
        EventBits_t bits = xEventGroupWaitBits(s_ev, EV_STA_GOT_IP, pdFALSE, pdFALSE,
                                               pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));
        if (!(bits & EV_STA_GOT_IP)) {
            ESP_LOGE(TAG, "STA connect timeout (ssid=%s)", CONFIG_CAMTEST_WIFI_STA_SSID);
            return ESP_ERR_TIMEOUT;
        }
        strlcpy(s_info.ssid, CONFIG_CAMTEST_WIFI_STA_SSID, sizeof(s_info.ssid));
    } else {
        strlcpy((char *)wc.ap.ssid, CONFIG_CAMTEST_WIFI_AP_SSID, sizeof(wc.ap.ssid));
        wc.ap.ssid_len = strlen(CONFIG_CAMTEST_WIFI_AP_SSID);
        wc.ap.max_connection = 4;
        if (strlen(CONFIG_CAMTEST_WIFI_AP_PASSWORD) >= 8) {
            strlcpy((char *)wc.ap.password, CONFIG_CAMTEST_WIFI_AP_PASSWORD, sizeof(wc.ap.password));
            wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
        } else {
            wc.ap.authmode = WIFI_AUTH_OPEN;
        }
        esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);   /* 20MHz-only（S31 非 AP 模式 11ax 限制） */
        err = esp_wifi_set_mode(WIFI_MODE_AP);
        if (err) return err;
        err = esp_wifi_set_config(WIFI_IF_AP, &wc);
        if (err) return err;
        err = esp_wifi_start();
        if (err) return err;
        esp_wifi_set_ps(WIFI_PS_NONE);
        if (!(xEventGroupWaitBits(s_ev, EV_AP_STARTED, pdFALSE, pdFALSE,
                                  pdMS_TO_TICKS(CONNECT_TIMEOUT_MS)) & EV_AP_STARTED))
            return ESP_ERR_TIMEOUT;
        strlcpy(s_info.ip, "192.168.4.1", sizeof(s_info.ip));
        strlcpy(s_info.ssid, CONFIG_CAMTEST_WIFI_AP_SSID, sizeof(s_info.ssid));
    }
    s_desired = mode;
    return ESP_OK;
}

esp_err_t wifi_net_start(void)
{
    s_ev = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_cb, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_cb, NULL));

    wifi_op_mode_t mode =
#if CONFIG_CAMTEST_WIFI_MODE_AP
        WIFI_MODE_AP_M;
#else
        WIFI_MODE_STA_M;
#endif
    /* 关闭 modem 省电：默认省电模式使局域网 RTT 恶化到 ~100ms（DonkeyCar 低延迟场景必须关） */
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_err_t err = start_one(mode);
#if !CONFIG_CAMTEST_WIFI_MODE_AP
    if (err == ESP_ERR_TIMEOUT) {
        /* STA 连不上：回退 SoftAP，保证浏览器总能连上（UI 会显示降级） */
        ESP_LOGE(TAG, "STA failed → fallback SoftAP");
        err = start_one(WIFI_MODE_AP_M);
    }
#endif
    return err;
}

esp_err_t wifi_net_switch(wifi_op_mode_t mode)
{
    if (mode == s_info.mode) return ESP_OK;
    ESP_LOGW(TAG, "switching wifi mode %d → %d", s_info.mode, mode);
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(300));
    return start_one(mode);
}

wifi_info_t *wifi_net_info(void)
{
    static int cnt;
    if ((cnt++ & 0x3) == 0) {   /* 降频刷新，避免每次调用都进 esp_wifi */
        if (s_info.mode == WIFI_MODE_STA_M) {
            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                s_info.rssi = ap.rssi;
                s_info.channel = ap.primary;
                s_info.band_mhz = (ap.second != WIFI_SECOND_CHAN_NONE) ? 40 : 20;
                /* phy 判定 */
                if (ap.phy_11ax) strlcpy(s_info.phy, "11ax", sizeof(s_info.phy));
                else if (ap.phy_11n) strlcpy(s_info.phy, "11n", sizeof(s_info.phy));
                else if (ap.phy_11g) strlcpy(s_info.phy, "11g", sizeof(s_info.phy));
                else if (ap.phy_11b) strlcpy(s_info.phy, "11b", sizeof(s_info.phy));
                else strlcpy(s_info.phy, "?", sizeof(s_info.phy));
            }
        } else {
            uint8_t ch = 0;
            wifi_second_chan_t sc;
            if (esp_wifi_get_channel(&ch, &sc) == ESP_OK) s_info.channel = ch;
            s_info.band_mhz = (sc == WIFI_SECOND_CHAN_NONE) ? 20 : 40;
            wifi_mode_t m;
            if (esp_wifi_get_mode(&m) == ESP_OK)
                strlcpy(s_info.phy, m == WIFI_MODE_AP ? "AP" : "?", sizeof(s_info.phy));
            s_info.rssi = 0;
        }
    }
    return &s_info;
}

void wifi_net_mdns_start(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(CONFIG_CAMTEST_MDNS_HOSTNAME));
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    mdns_service_add(NULL, "_camtest", "_tcp", 81, NULL, 0);
    ESP_LOGI(TAG, "mDNS: http://%s.local", CONFIG_CAMTEST_MDNS_HOSTNAME);
}

esp_err_t wifi_net_sntp_start(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_init();
    return ESP_OK;
}

bool wifi_net_time_synced(void)
{
    return esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED;
}
