/* metrics.c — 见 metrics.h */
#include "metrics.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "source_if.h"
#include "frame_ring.h"
#include "stream_server.h"
#include "wifi_net.h"

static const char *TAG = "metrics";

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
 * ★ 缓冲必须装下全表：Tab5 任务数 ~40（esp_hosted/lwip/USB/LVGL…），
 *   1024B 会把按占用排序后靠后的 IDLE 行截掉 → 占用率虚高（实机踩过：
 *   15fps reencode 显示 68/76% 纹丝不动，4KB 后回落到真实值）。
 *   同时每分钟输出一次 top-5 任务占用，便于定位真实负载源。
 */
static void update_cpu(void)
{
#if CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
    static char buf[4096];
    vTaskGetRunTimeStats(buf);
    float idle[2] = { 0, 0 };
    char tn[5][16] = {{0}};
    float tp[5] = {0};
    char *line = buf;
    while (line && *line) {
        char *nl = strchr(line, '\n');
        if (nl) *nl = 0;   /* 原地截断行：strlen/解析只在行内 */
        char name[20] = {0};
        if (sscanf(line, "%19s", name) == 1) {
            /* 百分比 = 行尾 token（vTaskGetRunTimeStats 行格式
             * "name<pad> <ticks>  <pct>%"；任务名可含空格如 "Tmr Svc"，
             * 按空格切段会把 ticks 当百分比——必须取行尾） */
            const char *p = line + strlen(line);
            while (p > line && isspace((unsigned char)p[-1])) p--;
            while (p > line && !isspace((unsigned char)p[-1])) p--;
            float v = strtof(p, NULL);   /* 自动停在 '%'；"<1%" → 0 */
            if (strncmp(name, "IDLE0", 5) == 0) idle[0] = v;
            if (strncmp(name, "IDLE1", 5) == 0) idle[1] = v;
            if (v > tp[4]) {   /* top-5 插入（升序尾部替换后冒泡归位） */
                tp[4] = v;
                strlcpy(tn[4], name, sizeof(tn[4]));
                for (int i = 4; i > 0 && tp[i] > tp[i - 1]; i--) {
                    float tf = tp[i]; tp[i] = tp[i - 1]; tp[i - 1] = tf;
                    char tmpn[16];
                    strlcpy(tmpn, tn[i], sizeof(tmpn));
                    strlcpy(tn[i], tn[i - 1], sizeof(tn[i]));
                    strlcpy(tn[i - 1], tmpn, sizeof(tn[i - 1]));
                }
            }
        }
        line = nl ? nl + 1 : NULL;
    }
    s_m.cpu0 = 100.0f - idle[0];
    s_m.cpu1 = 100.0f - idle[1];
    static int topn;
    if (++topn >= 6) {
        topn = 0;
        ESP_LOGI(TAG, "CPU top5: %s %.1f%% | %s %.1f%% | %s %.1f%% | %s %.1f%% | %s %.1f%%",
                 tn[0], tp[0], tn[1], tp[1], tn[2], tp[2], tn[3], tp[3], tn[4], tp[4]);
    }
#endif
}

/*
 * 码率自适应（P1，DonkeyCar 场景友好）：
 * 5s 窗口码率连续 3 次超目标 ×1.15 → h264 模式降码率 15%（最低 300kbps），
 * reencode 模式调低 JPEG 质量 5 档（最低 5）。
 * 只降不升（避免振荡）；每次事件写入 gov_last 供 UI 展示。
 */
static void governor_tick(void)
{
    if (s_m.target_mbps <= 0) return;
    if (s_m.bitrate_mbps > s_m.target_mbps * 1.15f) {
        if (++s_gov_overshoot >= 3) {
            s_gov_overshoot = 0;
            if (src_if_info()->usb_mode == USB_MODE_H264) {
                uint32_t kbps = src_if_h264_kbps();
                if (kbps > 300) {
                    uint32_t nkbps = kbps * 85 / 100;
                    if (nkbps < 300) nkbps = 300;
                    if (src_if_h264_set_bitrate_kbps(nkbps) == ESP_OK) {
                        s_m.gov_events++;
                        snprintf(s_m.gov_last, sizeof(s_m.gov_last),
                                 "5s码率 %.2f Mbps > 目标 %.2f → H264 %lu→%lu kbps",
                                 s_m.bitrate_mbps, s_m.target_mbps,
                                 (unsigned long)kbps, (unsigned long)nkbps);
                    }
                }
            } else {
                uint8_t q = src_if_info()->quality;
                if (src_if_info()->usb_mode == USB_MODE_PASSTHROUGH) {
                    snprintf(s_m.gov_last, sizeof(s_m.gov_last),
                             "passthrough 模式无法降质（quality 不可控），仅告警");
                } else if (q > 5) {
                    uint8_t nq = q - 5;
                    if (src_if_apply(0, 0, nq, 0) == ESP_OK) {
                        s_m.gov_events++;
                        snprintf(s_m.gov_last, sizeof(s_m.gov_last),
                                 "5s码率 %.2f Mbps > 目标 %.2f → 质量 %u→%u",
                                 s_m.bitrate_mbps, s_m.target_mbps, q, nq);
                    }
                }
            }
        }
    } else {
        s_gov_overshoot = 0;
    }
}

static void metrics_task(void *arg)
{
    uint32_t last_cap = 0, last_enc = 0, last_bytes = 0, last_pv = 0;
    uint64_t last_enc_bytes = 0, last_pv_bytes = 0;
    uint64_t last_us = esp_timer_get_time();
    int cpu_tick = 0;
    uint32_t last_total_sent_frames = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        src_stats_t *ps = src_if_stats();
        uint64_t now = esp_timer_get_time();
        float sec = (now - last_us) / 1e6f;
        if (sec <= 0) continue;

        uint32_t cap_now = ps->cap_frames, enc_now = ps->out_frames;
        uint32_t sent_frames = stream_server_frames_sent();
        uint32_t bytes_now = stream_server_bytes_sent();
        uint64_t enc_bytes_now = ps->out_bytes;
        uint32_t pv_now = ps->pv_frames;
        uint64_t pv_bytes_now = ps->pv_bytes;
        uint32_t d_cap = cap_now - last_cap;
        uint32_t d_enc = enc_now - last_enc;
        uint32_t d_bytes = bytes_now - last_bytes;
        uint64_t d_enc_bytes = enc_bytes_now - last_enc_bytes;
        uint32_t d_pv = pv_now - last_pv;
        uint64_t d_pv_bytes = pv_bytes_now - last_pv_bytes;
        uint32_t d_sent = sent_frames - last_total_sent_frames;

        s_m.cap_fps = d_cap / sec;
        s_m.enc_fps = d_enc / sec;
        s_m.send_fps = d_sent / sec;
        s_m.bitrate_mbps = d_enc_bytes * 8 / sec / 1e6;         /* 设备侧（编码输出）码率 */
        s_m.pv_fps = d_pv / sec;
        s_m.pv_mbps = d_pv_bytes * 8 / sec / 1e6;
        s_m.jpeg_avg_bytes = d_enc ? (uint32_t)(d_enc_bytes / d_enc) : 0;
        last_cap = cap_now; last_enc = enc_now; last_bytes = bytes_now;
        last_enc_bytes = enc_bytes_now;
        last_pv = pv_now; last_pv_bytes = pv_bytes_now;
        last_total_sent_frames = sent_frames;
        last_us = now;

        s_m.drop_capture = ps->cap_drops;
        s_m.drop_encode = ps->out_drops;
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

        /* 串口 CSV（每秒一行，供脚本抓取；末尾字段只增不改，旧脚本可继续按序解析）：
           CSV,<t_us>,cap_fps,out_fps,send_fps,mbps,jpeg_avg,drop_cap,drop_out,heap,psram,mode,rssi,res,source,usb_mode */
        wifi_info_t *w = wifi_net_info();
        src_info_t *ci = src_if_info();
        printf("CSV,%lld,%.2f,%.2f,%.2f,%.3f,%u,%u,%u,%u,%u,%s,%d,%ux%u,%s,%s\n",
               (long long)now, s_m.cap_fps, s_m.enc_fps, s_m.send_fps,
               s_m.bitrate_mbps, (unsigned)s_m.jpeg_avg_bytes,
               (unsigned)s_m.drop_capture, (unsigned)s_m.drop_encode,
               (unsigned)s_m.free_heap, (unsigned)s_m.free_psram,
               w->mode == WIFI_MODE_STA_M ? "STA" : "AP",
               w->rssi, ci->w, ci->h,
               src_if_source_name(ci->source), src_if_usb_mode_name(ci->usb_mode));
        governor_tick();
    }
}

void metrics_start(void)
{
    xTaskCreatePinnedToCore(metrics_task, "metrics", 4096, NULL, 4, NULL, 0);
}
