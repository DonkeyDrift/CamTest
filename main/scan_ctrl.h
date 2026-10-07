/*
 * scan_ctrl.h — 自动参数扫描状态机（闭环核心）
 *
 * 矩阵 = 分辨率档位（固定 6 档 ∩ sensor 实际支持；不支持者记 unsupported 行）
 *        × JPEG 质量 {10,20,30,40}（V4L2 语义：越大越清晰）
 *        × 网络模式 {当前模式, [可选 SoftAP]}
 * 每组：warmup 2 s（丢弃）+ 采样 10 s（每秒取 metrics 快照累计）。
 * 浏览器在扫描期间每秒 POST /api/scan/report 回传"到达帧率/时延统计"，
 * 由设备挂到当前组 —— 采集/编码/码率等设备侧指标始终由设备自己测。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum { SCAN_IDLE = 0, SCAN_RUNNING = 1, SCAN_DONE = 2 } scan_state_t;

typedef struct {
    char     timestamp[20];       /* 采样完成时刻 */
    char     sensor[16];
    char     resolution[16];      /* "640x480" 或 "unsupported" */
    int      quality;
    char     wifi_mode[4];        /* STA / AP */
    int      bandwidth_mhz;
    int      rssi;
    float    capture_fps, encode_fps, arrival_fps;
    float    bitrate_mbps;
    uint32_t jpeg_avg_bytes;
    float    latency_mean_ms, latency_p95_ms, latency_max_ms, latency_std_ms;
    float    encode_ms;
    float    cpu0_pct, cpu1_pct;
    uint32_t free_heap, free_psram, drop_frames;
    bool     unsupported;
    bool     has_client_data;     /* 浏览器回传是否拿到 */
} scan_row_t;

esp_err_t scan_ctrl_init(void);                      /* 创建锁（app_main 早期调用） */
esp_err_t scan_ctrl_start(bool include_ap);          /* modes=当前模式[,AP] */
void      scan_ctrl_stop(void);
scan_state_t scan_ctrl_state(void);

/* 扫描期间浏览器回传（http_server 调用）：挂在当前组 */
void scan_ctrl_report(float arrival_fps, float bitrate_mbps,
                      float lat_mean, float lat_p95, float lat_max, float lat_std);

/* 结果读取（拷贝行数组），返回行数 */
int scan_ctrl_rows(scan_row_t *out, int max);

/* CSV 导出（表头与任务书完全一致）；返回写入长度 */
int scan_ctrl_csv(char *buf, size_t buflen);
