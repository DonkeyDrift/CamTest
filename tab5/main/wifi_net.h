/* wifi_net.h — Wi-Fi STA/SoftAP + SNTP + mDNS（P4 上 esp_wifi 走 C6 协处理器） */
#pragma once
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_MODE_STA_M = 0,
    WIFI_MODE_AP_M,
} wifi_op_mode_t;

typedef struct {
    wifi_op_mode_t mode;
    bool connected;
    char ssid[33];
    char ip[16];
    char phy[8];
    int  channel;
    int  band_mhz;
    int  rssi;
} wifi_info_t;

esp_err_t wifi_net_start(void);                    /* app_main 一次（STA 失败回退 SoftAP） */
esp_err_t wifi_net_switch(wifi_op_mode_t mode);    /* 运行时切换 */
wifi_info_t *wifi_net_info(void);
void wifi_net_mdns_start(void);
esp_err_t wifi_net_sntp_start(void);
bool wifi_net_time_synced(void);

#ifdef __cplusplus
}
#endif
