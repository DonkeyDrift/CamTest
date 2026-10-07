/* scan_ctrl.c — 见 scan_ctrl.h */
#include "scan_ctrl.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "camera_pipeline.h"
#include "metrics.h"
#include "wifi_net.h"

static const char *TAG = "scan";

static const int TIER_RES[][2] = {
    {160,120},{320,240},{480,320},{640,480},{800,600},{1280,720},
};
#define TIER_N (sizeof(TIER_RES) / sizeof(TIER_RES[0]))
static const int TIER_QUAL[] = { 10, 20, 30, 40 };
#define TIER_QN (sizeof(TIER_QUAL) / sizeof(TIER_QUAL[0]))

static struct {
    scan_state_t state;
    scan_row_t rows[64];
    int n_rows;
    int idx;                       /* 当前组下标 */
    bool want_stop;
    /* 浏览器回传（最近一次） */
    struct {
        bool valid;
        int64_t us;
        float arrival_fps, bitrate_mbps, lat_mean, lat_p95, lat_max, lat_std;
    } report;
    SemaphoreHandle_t lock;
} s_scan;

scan_state_t scan_ctrl_state(void) { return s_scan.state; }

esp_err_t scan_ctrl_init(void)
{
    if (!s_scan.lock) s_scan.lock = xSemaphoreCreateMutex();
    return s_scan.lock ? ESP_OK : ESP_ERR_NO_MEM;
}

void scan_ctrl_report(float arrival_fps, float bitrate_mbps,
                      float lat_mean, float lat_p95, float lat_max, float lat_std)
{
    if (s_scan.state != SCAN_RUNNING) return;
    s_scan.report.valid = true;
    s_scan.report.us = esp_timer_get_time();
    s_scan.report.arrival_fps = arrival_fps;
    s_scan.report.bitrate_mbps = bitrate_mbps;
    s_scan.report.lat_mean = lat_mean;
    s_scan.report.lat_p95 = lat_p95;
    s_scan.report.lat_max = lat_max;
    s_scan.report.lat_std = lat_std;
}

static void accumulate(scan_row_t *row, int seconds)
{
    /* 每秒调用：把 metrics 快照累加为均值（P95/最大仅对客户端时延有意义） */
    metrics_t *m = metrics_get();
    wifi_info_t *w = wifi_net_info();
    cam_pipe_stats_t *ps = cam_pipe_stats();
    row->capture_fps += m->cap_fps / seconds;
    row->encode_fps += m->enc_fps / seconds;
    row->bitrate_mbps += m->bitrate_mbps / seconds;
    row->jpeg_avg_bytes += (uint32_t)(m->jpeg_avg_bytes / seconds);
    row->cpu0_pct += m->cpu0 / seconds;
    row->cpu1_pct += m->cpu1 / seconds;
    row->bandwidth_mhz = w->band_mhz;
    row->rssi = w->rssi;
    if (m->jpeg_avg_bytes) {
        /* 编码均值耗时：由 pipeline 累计器换算 */
        uint64_t acc = ps->enc_time_acc_us;
        uint32_t frames = ps->enc_frames;
        static uint64_t last_acc; static uint32_t last_frames;
        uint64_t d_acc = acc - last_acc; uint32_t d_f = frames - last_frames;
        last_acc = acc; last_frames = frames;
        if (d_f) row->encode_ms += (float)(d_acc / 1000) / d_f / seconds;
    }
    row->free_heap = m->free_heap;
    row->free_psram = m->free_psram;
    row->drop_frames = m->drop_capture + m->drop_encode;
}

static void finish_row(scan_row_t *row, const cam_pipe_info_t *info)
{
    strlcpy(row->sensor, info->sensor_name, sizeof(row->sensor));
    time_t t = (time_t)(esp_timer_get_time() / 1000000ULL);
    struct tm tm;
    gmtime_r(&t, &tm);
    strftime(row->timestamp, sizeof(row->timestamp), "%Y-%m-%dT%H:%M:%S", &tm);

    if (s_scan.report.valid &&
        esp_timer_get_time() - s_scan.report.us < 3000000LL) {
        row->arrival_fps = s_scan.report.arrival_fps;
        row->latency_mean_ms = s_scan.report.lat_mean;
        row->latency_p95_ms = s_scan.report.lat_p95;
        row->latency_max_ms = s_scan.report.lat_max;
        row->latency_std_ms = s_scan.report.lat_std;
        row->has_client_data = true;
        s_scan.report.valid = false;
    }
    /* 串口同步输出（无浏览器场景可用） */
    printf("SCAN,%s,%s,q%d,%s,%.1f,%.1f,%u,%.2f,%.1f,%.1f,%s\n",
           row->resolution, row->sensor, row->quality, row->wifi_mode,
           row->capture_fps, row->encode_fps, (unsigned)row->jpeg_avg_bytes,
           row->bitrate_mbps, row->encode_ms,
           row->latency_mean_ms, row->has_client_data ? "client-ok" : "no-client");
}

static void scan_task(void *arg)
{
    cam_pipe_info_t *info = cam_pipe_info();
    for (int i = 0; i < s_scan.n_rows; i++) {
        scan_row_t *row = &s_scan.rows[i];
        if (s_scan.want_stop) break;
        s_scan.idx = i;
        if (row->unsupported) {   /* 跳过但不缺失：结果中带 unsupported 标志 */
            continue;
        }
        int w, h, q;
        char mode;
        sscanf(row->resolution, "%dx%d", &w, &h);
        q = row->quality;
        mode = row->wifi_mode[0];

        /* 网络模式切换（如需要）：切换后等稳定，浏览器会自动重连并恢复回传 */
        wifi_info_t *wif = wifi_net_info();
        if ((mode == 'A' && wif->mode != WIFI_MODE_AP_M) ||
            (mode == 'S' && wif->mode != WIFI_MODE_STA_M)) {
            esp_err_t err = wifi_net_switch(mode == 'A' ? WIFI_MODE_AP_M : WIFI_MODE_STA_M);
            if (err != ESP_OK) { ESP_LOGE(TAG, "wifi switch failed"); continue; }
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        /* 重建链路 */
        if (cam_pipe_apply(w, h, q, 0) != ESP_OK) {
            ESP_LOGE(TAG, "apply %ux%u q%d failed", w, h, q);
            row->unsupported = true;   /* 运行时失败同样按不支持记录 */
            continue;
        }
        /* warmup：丢数据 */
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CAMTEST_SCAN_WARMUP_MS));
        /* 采样：逐秒累计 */
        int sample_s = CONFIG_CAMTEST_SCAN_SAMPLE_MS / 1000;
        for (int sec = 0; sec < sample_s && !s_scan.want_stop; sec++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            accumulate(row, sample_s);
        }
        finish_row(row, info);
    }
    /* 扫完回到原模式 */
    wifi_info_t *wif = wifi_net_info();
    (void)wif;
    ESP_LOGI(TAG, "scan done: %d rows", s_scan.n_rows);
    s_scan.state = SCAN_DONE;
    vTaskDelete(NULL);
}

esp_err_t scan_ctrl_start(bool include_ap)
{
    if (!s_scan.lock) s_scan.lock = xSemaphoreCreateMutex();
    if (s_scan.state == SCAN_RUNNING) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_scan.lock, portMAX_DELAY);
    memset(s_scan.rows, 0, sizeof(s_scan.rows));
    s_scan.n_rows = 0;
    s_scan.want_stop = false;
    s_scan.report.valid = false;

    bool ap_now = wifi_net_info()->mode == WIFI_MODE_AP_M;
    wifi_op_mode_t modes[2] = { ap_now ? WIFI_MODE_AP_M : WIFI_MODE_STA_M };
    int n_modes = 1;
    if (include_ap) { modes[1] = ap_now ? WIFI_MODE_STA_M : WIFI_MODE_AP_M; n_modes = 2; }

    cam_pipe_info_t *info = cam_pipe_info();
    for (int mi = 0; mi < n_modes; mi++) {
        char mode_str[4];
        strlcpy(mode_str, modes[mi] == WIFI_MODE_AP_M ? "AP" : "STA", sizeof(mode_str));
        for (size_t r = 0; r < TIER_N; r++) {
            for (size_t q = 0; q < TIER_QN; q++) {
                scan_row_t *row = &s_scan.rows[s_scan.n_rows];
                memset(row, 0, sizeof(*row));
                snprintf(row->resolution, sizeof(row->resolution), "%dx%d",
                         TIER_RES[r][0], TIER_RES[r][1]);
                row->quality = TIER_QUAL[q];
                strlcpy(row->wifi_mode, mode_str, sizeof(row->wifi_mode));
                strlcpy(row->sensor, info->sensor_name, sizeof(row->sensor));
                row->unsupported = !cam_pipe_res_supported(TIER_RES[r][0], TIER_RES[r][1]);
                if (++s_scan.n_rows >= (int)(sizeof(s_scan.rows) / sizeof(s_scan.rows[0])))
                    goto full;
            }
        }
    }
full:
    s_scan.state = SCAN_RUNNING;
    xSemaphoreGive(s_scan.lock);
    if (xTaskCreatePinnedToCore(scan_task, "scan_ctrl", 6144, NULL, 3, NULL, 0) != pdPASS) {
        s_scan.state = SCAN_IDLE;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "scan started: %d combos", s_scan.n_rows);
    return ESP_OK;
}

void scan_ctrl_stop(void)
{
    s_scan.want_stop = true;
}

int scan_ctrl_rows(scan_row_t *out, int max)
{
    xSemaphoreTake(s_scan.lock, portMAX_DELAY);
    int n = s_scan.n_rows < max ? s_scan.n_rows : max;
    memcpy(out, s_scan.rows, n * sizeof(scan_row_t));
    xSemaphoreGive(s_scan.lock);
    return n;
}

/* CSV 表头与任务书一致 */
static const char *CSV_HEADER =
    "timestamp,sensor,resolution,quality,wifi_mode,bandwidth_mhz,rssi,"
    "capture_fps,encode_fps,arrival_fps,jpeg_avg_bytes,bitrate_mbps,"
    "latency_mean_ms,latency_p95_ms,latency_max_ms,latency_std_ms,encode_ms,"
    "cpu0_pct,cpu1_pct,free_heap,free_psram,drop_frames\n";

int scan_ctrl_csv(char *buf, size_t buflen)
{
    size_t off = 0;
    static scan_row_t rows[64];   /* httpd 任务栈小，用静态缓冲（有 lock 保护） */
    int n = scan_ctrl_rows(rows, 64);
    off += strlcpy(buf + off, CSV_HEADER, buflen - off);
    for (int i = 0; i < n; i++) {
        scan_row_t *r = &rows[i];
        char line[512];
        if (r->unsupported) {
            snprintf(line, sizeof(line), "%s,%s,%s,%d,%s,,,,,,,,,,,,,,,,,,\n",
                     r->timestamp, r->sensor, r->resolution, r->quality, r->wifi_mode);
        } else {
            snprintf(line, sizeof(line),
                     "%s,%s,%s,%d,%s,%d,%d,%.2f,%.2f,%.2f,%u,%.3f,"
                     "%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u,%u\n",
                     r->timestamp, r->sensor, r->resolution, r->quality, r->wifi_mode,
                     r->bandwidth_mhz, r->rssi,
                     r->capture_fps, r->encode_fps, r->arrival_fps,
                     (unsigned)r->jpeg_avg_bytes, r->bitrate_mbps,
                     r->latency_mean_ms, r->latency_p95_ms, r->latency_max_ms,
                     r->latency_std_ms, r->encode_ms,
                     r->cpu0_pct, r->cpu1_pct,
                     (unsigned)r->free_heap, (unsigned)r->free_psram,
                     (unsigned)r->drop_frames);
        }
        off += strlcpy(buf + off, line, buflen - off);
        if (off >= buflen - 1) break;
    }
    return off;
}
