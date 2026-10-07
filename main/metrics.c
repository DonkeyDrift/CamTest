/* metrics.c — 见 metrics.h */
#include "metrics.h"
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/task.h"
#include "camera_pipeline.h"
#include "frame_ring.h"
#include "stream_server.h"
#include "wifi_net.h"

static metrics_t s_m;
static TaskHandle_t s_cap_h, s_enc_h;
static uint8_t s_gov_overshoot;

metrics_t *metrics_get(void) { return &s_m; }

void metrics_register_tasks(TaskHandle_t cap, TaskHandle_t enc)
{
    s_cap_h = cap; s_enc_h = enc;
}

void metrics_set_target_mbps(float v)
{
    s_m.target_mbps = v;
    s_gov_overshoot = 0;
}

/*
 * CPU 占用率：vTaskGetRunTimeStats 输出格式为 "<name> <abs_ticks> <pct>%\r\n"，
 * 取 IDLE0/IDLE1 行的最后一个百分数字段 → CPU0 = 100 − idle0%。
 */
static void update_cpu(void)
{
#if CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
    static char buf[1024];
    vTaskGetRunTimeStats(buf);
    float idle[2] = { 0, 0 };
    char *line = buf;
    while (line && *line) {
        char name[20] = {0};
        char pcts[24] = {0};
        /* 行内按空白切：第 1 段任务名，最后一段是 "12.3%" 形式 */
        if (sscanf(line, "%19s %*s %24s", name, pcts) >= 2) {
            char *pct_end;
            float v = strtof(pcts, &pct_end);   /* 自动停在 '%' */
            if (strncmp(name, "IDLE0", 5) == 0) idle[0] = v;
            if (strncmp(name, "IDLE1", 5) == 0) idle[1] = v;
        }
        char *nl = strchr(line, '\n');
        line = nl ? nl + 1 : NULL;
    }
    s_m.cpu0 = 100.0f - idle[0];
    s_m.cpu1 = 100.0f - idle[1];
#endif
}

/*
 * 码率自适应（P1，DonkeyCar 场景友好）：
 * 5s 窗口码率连续 3 次超目标 ×1.15 → 调低 JPEG 质量 5 档（最低 5）。
 * 方向语义：V4L2_CID_JPEG_COMPRESSION_QUALITY 越大越清晰、码率越大。
 * 只降不升（避免振荡）；每次事件写入 gov_last 供 UI 展示。
 */
static void governor_tick(void)
{
    if (s_m.target_mbps <= 0) return;
    if (s_m.bitrate_mbps > s_m.target_mbps * 1.15f) {
        if (++s_gov_overshoot >= 3) {
            s_gov_overshoot = 0;
            uint8_t q = cam_pipe_info()->quality;
            if (q > 5) {
                uint8_t nq = q - 5;
                if (cam_pipe_apply(0, 0, nq, 0) == ESP_OK) {
                    s_m.gov_events++;
                    snprintf(s_m.gov_last, sizeof(s_m.gov_last),
                             "5s码率 %.2f Mbps > 目标 %.2f → 质量 %u→%u",
                             s_m.bitrate_mbps, s_m.target_mbps, q, nq);
                }
            }
        }
    } else {
        s_gov_overshoot = 0;
    }
}

static void metrics_task(void *arg)
{
    uint32_t last_cap = 0, last_enc = 0, last_bytes = 0;
    uint64_t last_enc_bytes = 0;
    uint64_t last_us = esp_timer_get_time();
    int cpu_tick = 0;
    uint32_t last_total_sent_frames = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        cam_pipe_stats_t *ps = cam_pipe_stats();
        uint64_t now = esp_timer_get_time();
        float sec = (now - last_us) / 1e6f;
        if (sec <= 0) continue;

        uint32_t cap_now = ps->cap_frames, enc_now = ps->enc_frames;
        uint32_t sent_frames = stream_server_frames_sent();
        uint32_t bytes_now = stream_server_bytes_sent();
        uint64_t enc_bytes_now = ps->enc_bytes;
        uint32_t d_cap = cap_now - last_cap;
        uint32_t d_enc = enc_now - last_enc;
        uint32_t d_bytes = bytes_now - last_bytes;
        uint64_t d_enc_bytes = enc_bytes_now - last_enc_bytes;
        uint32_t d_sent = sent_frames - last_total_sent_frames;

        s_m.cap_fps = d_cap / sec;
        s_m.enc_fps = d_enc / sec;
        s_m.send_fps = d_sent / sec;
        s_m.bitrate_mbps = d_enc_bytes * 8 / sec / 1e6;         /* 设备侧（编码输出）码率 */
        s_m.jpeg_avg_bytes = d_enc ? (uint32_t)(d_enc_bytes / d_enc) : 0;
        last_cap = cap_now; last_enc = enc_now; last_bytes = bytes_now;
        last_enc_bytes = enc_bytes_now;
        last_total_sent_frames = sent_frames;
        last_us = now;

        s_m.drop_capture = ps->capture_drops;
        s_m.drop_encode = ps->encode_drops;
        s_m.free_heap = esp_get_free_heap_size();
        s_m.min_heap = esp_get_minimum_free_heap_size();
        s_m.free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        static uint32_t min_ps;
        if (s_m.free_psram < min_ps || !min_ps) min_ps = s_m.free_psram;
        s_m.min_psram = min_ps;
        s_m.stack_cap = s_cap_h ? uxTaskGetStackHighWaterMark(s_cap_h) * sizeof(StackType_t) : 0;
        s_m.stack_enc = s_enc_h ? uxTaskGetStackHighWaterMark(s_enc_h) * sizeof(StackType_t) : 0;
        s_m.uptime_s = (uint32_t)(now / 1000000ULL);

        if ((cpu_tick++ & 1) == 0) update_cpu();

        /* 串口 CSV（每秒一行，供脚本抓取）：
           CSV,<t_us>,cap_fps,enc_fps,send_fps,mbps,jpeg_avg,drop_cap,drop_enc,heap,psram,mode,rssi,res */
        wifi_info_t *w = wifi_net_info();
        cam_pipe_info_t *ci = cam_pipe_info();
        printf("CSV,%lld,%.2f,%.2f,%.2f,%.3f,%u,%u,%u,%u,%u,%s,%d,%ux%u\n",
               (long long)now, s_m.cap_fps, s_m.enc_fps, s_m.send_fps,
               s_m.bitrate_mbps, (unsigned)s_m.jpeg_avg_bytes,
               (unsigned)s_m.drop_capture, (unsigned)s_m.drop_encode,
               (unsigned)s_m.free_heap, (unsigned)s_m.free_psram,
               w->mode == WIFI_MODE_STA_M ? "STA" : "AP",
               w->rssi, ci->w, ci->h);
        governor_tick();
    }
}

void metrics_start(void)
{
    xTaskCreatePinnedToCore(metrics_task, "metrics", 4096, NULL, 4, NULL, 0);
}
