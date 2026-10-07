/*
 * metrics.h — 设备侧指标聚合（每秒）：FPS/码率/CPU/内存/丢帧/栈水位 + 串口 CSV 行
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    float cap_fps, enc_fps, send_fps;
    float bitrate_mbps;
    uint32_t jpeg_avg_bytes;
    float cpu0, cpu1;
    uint32_t free_heap, min_heap, free_psram, min_psram;
    uint32_t drop_capture, drop_encode;
    int stack_cap, stack_enc;          /* 任务栈最低水位（字节） */
    uint32_t uptime_s;
    /* 码率自适应降质（P1） */
    float target_mbps;                 /* 0=关闭 */
    uint32_t gov_events;
    char gov_last[96];                 /* 最近一次降质事件描述 */
} metrics_t;

void metrics_start(void);
metrics_t *metrics_get(void);

/* 码率自适应目标（0=关闭）；governor 在 metrics 任务里每秒评估 */
void metrics_set_target_mbps(float v);
void metrics_register_tasks(TaskHandle_t cap, TaskHandle_t enc);
