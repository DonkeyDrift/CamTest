/*
 * stream_server.h — 原生 socket 流服务（:81）
 *
 * 为什么不用 esp_http_server 提供 /stream：
 *   esp_http_server 单 task 分发请求，长连接 handler 会阻塞同实例上的其他会话，
 *   无法满足"2~3 个并发观看端"的硬需求。这里直接用 lwip socket：
 *   accept 一客户端 → 派发独立 task 发送，天然并发、可统计各自到达帧率。
 * 端点：
 *   GET /stream  — multipart/x-mixed-replace MJPEG，part 头带 X-Frame-Id 等元数据
 *   GET /ws      — WebSocket 二进制帧推送（36B 小端头 + JPEG），用于对比协议开销
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    char type[8];     /* "mjpeg" / "ws" */
    char ip[16];
    float fps;        /* 该连接实际发送帧率 */
    float mbps;       /* 该连接实际码率 */
    uint32_t frames;
    bool active;
} stream_client_stat_t;

esp_err_t stream_server_start(uint16_t port);
int  stream_server_get_clients(stream_client_stat_t *out, int max);
uint32_t stream_server_frames_sent(void);   /* 累计发送帧数（所有客户端） */
uint32_t stream_server_bytes_sent(void);

/* UDP 分片推送（Kconfig 可关）：仅 1 个订阅者，见 tools/udp_receiver.py */
esp_err_t udp_push_start(uint16_t port);
