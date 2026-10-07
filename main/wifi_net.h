/*
 * wifi_net.h — Wi-Fi STA/SoftAP、RSSI/PHY 信息、mDNS、SNTP（可选）
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum { WIFI_MODE_NONE = 0, WIFI_MODE_STA_M, WIFI_MODE_AP_M } wifi_op_mode_t;

typedef struct {
    wifi_op_mode_t mode;
    char phy[12];        /* 11ax / 11n / 11g */
    int  channel;        /* 主信道 */
    int  band_mhz;       /* 20 / 40 */
    int  rssi;           /* STA 模式有效；AP 模式填 0 */
    char ip[16];
    char ssid[33];
    bool connected;
} wifi_info_t;

esp_err_t wifi_net_start(void);                          /* 按 Kconfig 默认模式启动 */
esp_err_t wifi_net_switch(wifi_op_mode_t mode);          /* 扫描用：STA↔AP 切换（阻塞至连接/起网或超时） */
wifi_info_t *wifi_net_info(void);                        /* 实时信息（内部刷新） */
void wifi_net_mdns_start(void);
esp_err_t wifi_net_sntp_start(void);                     /* 可选：SNTP 对时（ wall clock 交叉验证用） */
bool wifi_net_time_synced(void);
