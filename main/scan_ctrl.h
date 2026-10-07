/*
 * scan_ctrl.h — 自动参数扫描状态机（闭环核心，U6/U8 扩展版）
 *
 * 矩阵按采集源分组（两源的可控维度不同）：
 *   DVP:            {160x120, 320x240, 640x480} × 质量{10,20,30} × 协议{http,ws,udp} = 27 组
 *   USB(直通):       摄像头 MJPEG 原生档（动态枚举） × 质量=passthrough × 协议{http,ws,udp}
 *   USB(重编码):     摄像头 YUY2 原生档 × 质量{10,20,30} × 协议{http,ws,udp}
 * 每组：warmup 2 s（丢弃）+ 采样 10 s（每秒取 metrics 快照累计）。
 * 浏览器在扫描期间每秒 POST /api/scan/report 回传"到达帧率/时延统计"（http/ws 模式），
 * udp 模式由 tools/udp_receiver.py 回传（可含 udp_loss_rate/udp_incomplete_rate）。
 * 协议切换由浏览器根据当前行自动完成；扫描范围可选 dvp / usb / both（默认 both）。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum { SCAN_IDLE = 0, SCAN_RUNNING = 1, SCAN_DONE = 2 } scan_state_t;

typedef struct {
    char     timestamp[20];           /* 采样完成时刻 */
    char     video_source[8];         /* dvp / usb */
    char     sensor[24];              /* DVP sensor 名 / USB "vid:pid" */
    char     usb_device_name[24];     /* 非 USB 行为空 */
    char     resolution[16];          /* "640x480" 或 "unsupported" */
    int      scaled;                  /* 0/1：输出是否经 ESP32 缩放 */
    char     quality[16];             /* "10"/"20"/"30" 或 "passthrough" */
    int      quality_num;             /* 0=passthrough（apply 用） */
    char     usb_mode[12];            /* passthrough / reencode / n/a */
    char     protocol[8];             /* http / ws / udp */
    char     wifi_mode[4];            /* STA / AP */
    int      bandwidth_mhz;
    int      rssi;
    char     capture_ts_meaning[16];  /* sensor_out / frame_arrival */
    float    capture_fps, encode_fps, arrival_fps;
    uint32_t jpeg_avg_bytes;
    float    bitrate_mbps;
    float    latency_mean_ms, latency_p95_ms, latency_max_ms, latency_std_ms;
    float    encode_ms;               /* 设备侧处理耗时均值（DVP/重编码=编码；直通=发布拷贝） */
    char     usb_inherent[16];        /* 摄像头内部固有延迟估算 ms；未标定为空 */
    uint32_t usb_disconnect_count;
    float    cpu0_pct, cpu1_pct;
    uint32_t free_heap, free_psram, drop_frames;
    char     udp_loss_rate[16];       /* 仅 UDP 行且订阅端回传时有值 */
    char     udp_incomplete_rate[16];
    bool     unsupported;
    bool     has_client_data;         /* 浏览器/工具回传是否拿到 */
} scan_row_t;

typedef struct {
    scan_state_t state;
    int idx, total;
    char cur_source[8];        /* 两级进度（U8）：当前源 */
    char cur_protocol[8];      /* 当前组协议 */
    char cur_res[16];
    char cur_quality[16];
    bool usb_skipped;          /* USB 不在线被跳过 */
} scan_prog_t;

esp_err_t scan_ctrl_init(void);                       /* 创建锁与行存储（app_main 早期调用） */
esp_err_t scan_ctrl_start(bool include_ap, const char *scope);  /* scope: "dvp"/"usb"/"both"(默认) */
void      scan_ctrl_stop(void);
scan_state_t scan_ctrl_state(void);
void      scan_ctrl_progress(scan_prog_t *out);

/* 扫描期间浏览器/工具回传（http_server 调用）：挂在当前组 */
void scan_ctrl_report(float arrival_fps, float bitrate_mbps,
                      float lat_mean, float lat_p95, float lat_max, float lat_std);
void scan_ctrl_report_udp(float loss_rate, float incomplete_rate);

/* 结果读取（拷贝行数组），返回行数 */
int scan_ctrl_rows(scan_row_t *out, int max);

/* CSV 导出（表头=任务书完整表头，含 6 个新字段）；返回写入长度 */
int scan_ctrl_csv(char *buf, size_t buflen);
