/* metrics.h — 1Hz 聚合指标 + 串口 CSV + 码率 governor */
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float cap_fps, enc_fps, send_fps;
    float bitrate_mbps;
    float pv_fps, pv_mbps;          /* JPEG 预览流（h264 模式并行发布给无 WebCodecs 浏览器） */
    uint32_t jpeg_avg_bytes;
    float cpu0, cpu1;
    uint32_t free_heap, min_heap, free_psram, min_psram;
    uint32_t drop_capture, drop_encode;
    int stack_cap, stack_enc;
    uint32_t uptime_s;
    float target_mbps;              /* >0 时 governor 生效 */
    char gov_last[96];
    uint32_t gov_events;
} metrics_t;

metrics_t *metrics_get(void);
void metrics_register_tasks(TaskHandle_t cap, TaskHandle_t enc);
void metrics_set_target_mbps(float v);
void metrics_start(void);

#ifdef __cplusplus
}
#endif
