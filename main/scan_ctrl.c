/* scan_ctrl.c — 见 scan_ctrl.h（U6/U8：按源分组矩阵 + 6 新 CSV 字段 + 范围选择） */
#include "scan_ctrl.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "source_if.h"
#include "metrics.h"
#include "wifi_net.h"
#include "lcd_ui.h"
#include "sdkconfig.h"

static const char *TAG = "scan";

/* U8 矩阵定义：DVP 3 分辨率 × 3 质量 × 3 协议 = 27 组 */
static const int DVP_RES[][2] = { {160,120},{320,240},{640,480} };
#define DVP_RES_N (sizeof(DVP_RES) / sizeof(DVP_RES[0]))
static const int DVP_QUAL[] = { 10, 20, 30 };
#define DVP_QUAL_N (sizeof(DVP_QUAL) / sizeof(DVP_QUAL[0]))
static const char *PROTO[] = { "http", "ws", "udp" };
#define PROTO_N (sizeof(PROTO) / sizeof(PROTO[0]))

#define SCAN_ROWS_MAX 128

static struct {
    scan_state_t state;
    scan_row_t *rows;              /* PSRAM（128 行 × ~200B） */
    int n_rows;
    int idx;                       /* 当前组下标 */
    bool want_stop;
    bool usb_skipped;
    video_source_t restore_source; /* 扫描结束恢复的源 */
    /* 浏览器/工具回传（最近一次） */
    struct {
        bool valid;
        int64_t us;
        float arrival_fps, bitrate_mbps, lat_mean, lat_p95, lat_max, lat_std;
    } report;
    struct {
        bool valid;
        int64_t us;
        float loss, incomplete;
    } udp_report;
    SemaphoreHandle_t lock;
} s_scan;

scan_state_t scan_ctrl_state(void) { return s_scan.state; }

esp_err_t scan_ctrl_init(void)
{
    if (!s_scan.lock) s_scan.lock = xSemaphoreCreateMutex();
    if (!s_scan.rows) {
        s_scan.rows = heap_caps_calloc(SCAN_ROWS_MAX, sizeof(scan_row_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    return (s_scan.lock && s_scan.rows) ? ESP_OK : ESP_ERR_NO_MEM;
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

void scan_ctrl_report_udp(float loss_rate, float incomplete_rate)
{
    if (s_scan.state != SCAN_RUNNING) return;
    s_scan.udp_report.valid = true;
    s_scan.udp_report.us = esp_timer_get_time();
    s_scan.udp_report.loss = loss_rate;
    s_scan.udp_report.incomplete = incomplete_rate;
}

static void accumulate(scan_row_t *row, int seconds)
{
    /* 每秒调用：把 metrics 快照累加为均值（P95/最大仅对客户端时延有意义） */
    metrics_t *m = metrics_get();
    wifi_info_t *w = wifi_net_info();
    src_stats_t *ps = src_if_stats();
    row->capture_fps += m->cap_fps / seconds;
    row->encode_fps += m->enc_fps / seconds;
    row->bitrate_mbps += m->bitrate_mbps / seconds;
    row->jpeg_avg_bytes += (uint32_t)(m->jpeg_avg_bytes / seconds);
    row->cpu0_pct += m->cpu0 / seconds;
    row->cpu1_pct += m->cpu1 / seconds;
    row->bandwidth_mhz = w->band_mhz;
    row->rssi = w->rssi;
    if (m->jpeg_avg_bytes) {
        /* 输出处理均值耗时：由源后端累计器换算（DVP/重编码=编码；直通=发布拷贝） */
        uint64_t acc = ps->proc_acc_us;
        uint32_t frames = ps->out_frames;
        static uint64_t last_acc; static uint32_t last_frames;
        uint64_t d_acc = acc - last_acc; uint32_t d_f = frames - last_frames;
        last_acc = acc; last_frames = frames;
        if (d_f) row->encode_ms += (float)(d_acc / 1000) / d_f / seconds;
    }
    row->free_heap = m->free_heap;
    row->free_psram = m->free_psram;
    row->drop_frames = m->drop_capture + m->drop_encode;
}

static void finish_row(scan_row_t *row)
{
    src_info_t *info = src_if_info();
    time_t t = (time_t)(esp_timer_get_time() / 1000000ULL);
    struct tm tm;
    gmtime_r(&t, &tm);
    strftime(row->timestamp, sizeof(row->timestamp), "%Y-%m-%dT%H:%M:%S", &tm);

    /* U6：源级元数据落行（以采样末刻的源信息为准） */
    strlcpy(row->video_source, src_if_source_name(info->source), sizeof(row->video_source));
    strlcpy(row->sensor, info->sensor_name, sizeof(row->sensor));
    strlcpy(row->usb_device_name,
            info->source == VIDEO_SOURCE_USB ? info->usb_device_name : "",
            sizeof(row->usb_device_name));
    row->scaled = info->scaled ? 1 : 0;
    strlcpy(row->usb_mode, src_if_usb_mode_name(info->usb_mode), sizeof(row->usb_mode));
    strlcpy(row->capture_ts_meaning, src_if_ts_meaning_name(info->ts_meaning),
            sizeof(row->capture_ts_meaning));
    float inherent = src_if_usb_inherent_ms();
    if (info->source == VIDEO_SOURCE_USB && inherent >= 0) {
        snprintf(row->usb_inherent, sizeof(row->usb_inherent), "%.1f", inherent);
    }   /* 未标定/DVP：留空，绝不伪造 */
    row->usb_disconnect_count = info->source == VIDEO_SOURCE_USB ? src_if_usb_disconnects() : 0;

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
    if (row->protocol[0] == 'u' && s_scan.udp_report.valid &&
        esp_timer_get_time() - s_scan.udp_report.us < 3000000LL) {
        snprintf(row->udp_loss_rate, sizeof(row->udp_loss_rate), "%.2f", s_scan.udp_report.loss);
        snprintf(row->udp_incomplete_rate, sizeof(row->udp_incomplete_rate), "%.2f", s_scan.udp_report.incomplete);
        s_scan.udp_report.valid = false;
    }
    /* 串口同步输出（无浏览器场景可用） */
    printf("SCAN,%s,%s,%s,q%s,%s,%s,%.1f,%.1f,%u,%.2f,%.1f,%.1f,%s\n",
           row->video_source, row->resolution, row->usb_mode, row->quality, row->protocol,
           row->wifi_mode, row->capture_fps, row->encode_fps, (unsigned)row->jpeg_avg_bytes,
           row->bitrate_mbps, row->encode_ms, row->latency_mean_ms,
           row->has_client_data ? "client-ok" : "no-client");
}

/* ---------- 矩阵构建 ---------- */
static void add_row(const char *source, const char *sensor, const char *res,
                    const char *qstr, int qnum, const char *umode, const char *proto,
                    const char *wifi, bool unsupported)
{
    if (s_scan.n_rows >= SCAN_ROWS_MAX) return;
    scan_row_t *row = &s_scan.rows[s_scan.n_rows++];
    memset(row, 0, sizeof(*row));
    strlcpy(row->video_source, source, sizeof(row->video_source));
    strlcpy(row->sensor, sensor, sizeof(row->sensor));
    strlcpy(row->resolution, res, sizeof(row->resolution));
    strlcpy(row->quality, qstr, sizeof(row->quality));
    row->quality_num = qnum;
    strlcpy(row->usb_mode, umode, sizeof(row->usb_mode));
    strlcpy(row->protocol, proto, sizeof(row->protocol));
    strlcpy(row->wifi_mode, wifi, sizeof(row->wifi_mode));
    row->unsupported = unsupported;
    strlcpy(row->capture_ts_meaning,
            strcmp(source, "usb") == 0 ? "frame_arrival" : "sensor_out",
            sizeof(row->capture_ts_meaning));
}

static void build_dvp_group(const char *wifi)
{
    src_info_t *info = src_if_info();
    const char *sensor = info->sensor_name;
    for (size_t r = 0; r < DVP_RES_N; r++) {
        char res[16];
        snprintf(res, sizeof(res), "%dx%d", DVP_RES[r][0], DVP_RES[r][1]);
        bool sup = src_if_res_supported(DVP_RES[r][0], DVP_RES[r][1]);
        for (size_t q = 0; q < DVP_QUAL_N; q++) {
            char qstr[16];
            snprintf(qstr, sizeof(qstr), "%d", DVP_QUAL[q]);
            for (size_t p = 0; p < PROTO_N; p++) {
                add_row("dvp", sensor, res, qstr, DVP_QUAL[q], "n/a", PROTO[p], wifi, !sup);
            }
        }
    }
}

static void build_usb_groups(const char *wifi)
{
    usb_tier_t tiers[USB_TIER_MAX];
    int n = src_if_usb_get_tiers(tiers, USB_TIER_MAX);
    char src_sensor[24];
    strlcpy(src_sensor, src_if_usb_device_name()[0] ? src_if_usb_device_name() : "USB-UVC",
            sizeof(src_sensor));

    /* MJPEG 原生档 × passthrough × 协议 */
    for (int i = 0; i < n; i++) {
        if (!tiers[i].mjpeg) continue;
        char res[16];
        snprintf(res, sizeof(res), "%dx%d", tiers[i].w, tiers[i].h);
        for (size_t p = 0; p < PROTO_N; p++) {
            add_row("usb", src_sensor, res, "passthrough", 0, "passthrough", PROTO[p], wifi, false);
        }
    }
    /* YUY2 原生档 × 质量{10,20,30} × 协议（重编码） */
    for (int i = 0; i < n; i++) {
        if (tiers[i].mjpeg) continue;
        char res[16];
        snprintf(res, sizeof(res), "%dx%d", tiers[i].w, tiers[i].h);
        for (size_t q = 0; q < DVP_QUAL_N; q++) {
            char qstr[16];
            snprintf(qstr, sizeof(qstr), "%d", DVP_QUAL[q]);
            for (size_t p = 0; p < PROTO_N; p++) {
                add_row("usb", src_sensor, res, qstr, DVP_QUAL[q], "reencode", PROTO[p], wifi, false);
            }
        }
    }
}

/* Wi-Fi 模式切换（行组声明模式与当前不符时调用） */
static bool ensure_wifi_mode(const char *mode)
{
    wifi_info_t *wif = wifi_net_info();
    if ((mode[0] == 'A' && wif->mode != WIFI_MODE_AP_M) ||
        (mode[0] == 'S' && wif->mode != WIFI_MODE_STA_M)) {
        esp_err_t err = wifi_net_switch(mode[0] == 'A' ? WIFI_MODE_AP_M : WIFI_MODE_STA_M);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "wifi switch failed");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));   /* 浏览器断流重连 + 回传恢复 */
    }
    return true;
}

static void scan_task(void *arg)
{
    video_source_t cur_source = VIDEO_SOURCE_DVP;   /* 与 restore 分离：执行时跟踪 */
    usb_mode_t cur_mode = USB_MODE_NONE;
    for (int i = 0; i < s_scan.n_rows; i++) {
        scan_row_t *row = &s_scan.rows[i];
        if (s_scan.want_stop) break;
        s_scan.idx = i;
        if (row->unsupported) {   /* 跳过但不缺失：结果中带 unsupported 标志 */
            continue;
        }

        /* Wi-Fi 模式（AP 双模式扫描时按组切换） */
        if (row->wifi_mode[0] && !ensure_wifi_mode(row->wifi_mode)) {
            row->unsupported = true;
            continue;
        }

        /* ---- 源/模式切换（按行声明，变化时才切换） ---- */
        bool want_usb = strcmp(row->video_source, "usb") == 0;
        video_source_t want_src = want_usb ? VIDEO_SOURCE_USB : VIDEO_SOURCE_DVP;
        usb_mode_t want_mode = want_usb
            ? (strcmp(row->usb_mode, "passthrough") == 0 ? USB_MODE_PASSTHROUGH : USB_MODE_REENCODE)
            : USB_MODE_NONE;

        if (want_src != cur_source) {
            esp_err_t err = src_if_switch(want_src);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "切换到 %s 失败：%s", row->video_source, esp_err_to_name(err));
                row->unsupported = true;   /* 运行时失败按不支持记录（如 USB 中途拔出） */
                cur_source = src_if_current();
                continue;
            }
            cur_source = want_src;
            cur_mode = USB_MODE_NONE;
        }
        if (want_usb && want_mode != cur_mode) {
            esp_err_t err = src_if_usb_set_mode(want_mode);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "USB 切换 %s 失败", row->usb_mode);
                row->unsupported = true;
                cur_mode = src_if_usb_mode();
                continue;
            }
            cur_mode = want_mode;
        }

        /* ---- 参数应用 ---- */
        int w = 0, h = 0;
        sscanf(row->resolution, "%dx%d", &w, &h);
        esp_err_t err = src_if_apply(w, h, row->quality_num, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "apply %s %s q%s failed", row->video_source, row->resolution, row->quality);
            row->unsupported = true;
            continue;
        }
        /* warmup：丢数据（协议切换由浏览器在 warmup 窗口内完成） */
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CAMTEST_SCAN_WARMUP_MS));
        /* 采样：逐秒累计 */
        int sample_s = CONFIG_CAMTEST_SCAN_SAMPLE_MS / 1000;
        for (int sec = 0; sec < sample_s && !s_scan.want_stop; sec++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            accumulate(row, sample_s);
        }
        finish_row(row);
    }

    /* 恢复扫描前的源（避免停在 USB 档位上） */
    if (src_if_current() != s_scan.restore_source) {
        if (src_if_switch(s_scan.restore_source) != ESP_OK) {
            ESP_LOGW(TAG, "恢复源 %s 失败", src_if_source_name(s_scan.restore_source));
        }
    }
    ESP_LOGI(TAG, "scan done: %d rows", s_scan.n_rows);
    s_scan.state = SCAN_DONE;
    vTaskDelete(NULL);
}

esp_err_t scan_ctrl_start(bool include_ap, const char *scope)
{
    if (!s_scan.rows && scan_ctrl_init() != ESP_OK) return ESP_ERR_NO_MEM;
    if (s_scan.state == SCAN_RUNNING) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_scan.lock, portMAX_DELAY);
    memset(s_scan.rows, 0, SCAN_ROWS_MAX * sizeof(scan_row_t));
    s_scan.n_rows = 0;
    s_scan.want_stop = false;
    s_scan.usb_skipped = false;
    s_scan.report.valid = false;
    s_scan.udp_report.valid = false;
    s_scan.restore_source = src_if_current();

    bool do_dvp = !scope || strcmp(scope, "both") == 0 || strcmp(scope, "dvp") == 0;
    bool do_usb = !scope || strcmp(scope, "both") == 0 || strcmp(scope, "usb") == 0;

    bool ap_now = wifi_net_info()->mode == WIFI_MODE_AP_M;
    const char *modes[2] = { ap_now ? "AP" : "STA", ap_now ? "STA" : "AP" };
    int n_modes = 1;
    if (include_ap) n_modes = 2;

    /* USB 组前置校验：设备不在线则整组跳过（UI 显示 usb_skipped） */
    bool usb_ok = do_usb;
#if CONFIG_CAMTEST_ENABLE_USB
    if (do_usb) {
        usb_state_t us = src_if_usb_state();
        if (us != USB_STATE_DEVICE_READY && us != USB_STATE_STREAMING) {
            usb_ok = false;
            s_scan.usb_skipped = true;
            ESP_LOGW(TAG, "USB 摄像头不在线（state=%d），USB 组跳过", us);
        }
    }
#else
    if (do_usb) { usb_ok = false; s_scan.usb_skipped = true; }
#endif

    for (int mi = 0; mi < n_modes; mi++) {
        if (do_dvp) build_dvp_group(modes[mi]);
        if (usb_ok) {
            /* USB 组在当前 Wi-Fi 模式下需要先切到该模式才能构建（原逻辑同样按模式切换执行） */
            build_usb_groups(modes[mi]);
        }
    }
    if (s_scan.n_rows == 0) {
        xSemaphoreGive(s_scan.lock);
        return ESP_ERR_INVALID_STATE;
    }

    s_scan.state = SCAN_RUNNING;
    xSemaphoreGive(s_scan.lock);
    if (xTaskCreatePinnedToCore(scan_task, "scan_ctrl", 6144, NULL, 3, NULL, 0) != pdPASS) {
        s_scan.state = SCAN_IDLE;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "scan started: %d combos (scope=%s%s)", s_scan.n_rows, scope ? scope : "both",
             s_scan.usb_skipped ? "，USB 组已跳过" : "");
    return ESP_OK;
}

void scan_ctrl_stop(void)
{
    s_scan.want_stop = true;
}

void scan_ctrl_progress(scan_prog_t *out)
{
    memset(out, 0, sizeof(*out));
    out->state = s_scan.state;
    out->idx = s_scan.idx;
    out->total = s_scan.n_rows;
    out->usb_skipped = s_scan.usb_skipped;
    if (s_scan.state == SCAN_RUNNING && s_scan.idx < s_scan.n_rows) {
        scan_row_t *r = &s_scan.rows[s_scan.idx];
        strlcpy(out->cur_source, r->video_source, sizeof(out->cur_source));
        strlcpy(out->cur_protocol, r->protocol, sizeof(out->cur_protocol));
        strlcpy(out->cur_res, r->resolution, sizeof(out->cur_res));
        strlcpy(out->cur_quality, r->quality, sizeof(out->cur_quality));
    }
}

int scan_ctrl_rows(scan_row_t *out, int max)
{
    xSemaphoreTake(s_scan.lock, portMAX_DELAY);
    int n = s_scan.n_rows < max ? s_scan.n_rows : max;
    memcpy(out, s_scan.rows, n * sizeof(scan_row_t));
    xSemaphoreGive(s_scan.lock);
    return n;
}

/* ---------- CSV：表头与任务书 U6 完全一致（旧列全部保留、位置对齐新表头） ---------- */
static const char *CSV_HEADER =
    "timestamp,video_source,sensor,usb_device_name,resolution,scaled,quality,usb_mode,protocol,"
    "tcp_nodelay,lcd_on,wifi_mode,bandwidth_mhz,rssi,capture_ts_meaning,"
    "capture_fps,encode_fps,arrival_fps,jpeg_avg_bytes,bitrate_mbps,"
    "latency_mean_ms,latency_p95_ms,latency_max_ms,latency_std_ms,encode_ms,"
    "usb_cam_inherent_latency_ms,usb_disconnect_count,"
    "cpu0_pct,cpu1_pct,free_heap,free_psram,drop_frames,udp_loss_rate,udp_incomplete_rate\n";

int scan_ctrl_csv(char *buf, size_t buflen)
{
    size_t off = 0;
    static scan_row_t *rows;        /* httpd 任务栈小：PSRAM 缓冲（有 lock 保护，单实例） */
    if (!rows) rows = heap_caps_calloc(SCAN_ROWS_MAX, sizeof(scan_row_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rows) return 0;
    int n = scan_ctrl_rows(rows, SCAN_ROWS_MAX);
    off += strlcpy(buf + off, CSV_HEADER, buflen - off);
    for (int i = 0; i < n; i++) {
        scan_row_t *r = &rows[i];
        char line[640];
        if (r->unsupported) {
            /* 不支持行：标识列 + tcp/lcd/tsm 常量列，数值列全部留空（16..34 列） */
            int used = snprintf(line, sizeof(line),
                     "%s,%s,%s,%s,%s,,%s,%s,%s,1,%d,%s,,,%s",
                     r->timestamp, r->video_source, r->sensor, r->usb_device_name,
                     r->resolution, r->quality, r->usb_mode, r->protocol,
                     lcd_ui_active() ? 1 : 0,
                     r->wifi_mode, r->capture_ts_meaning);
            for (int c = 16; c <= 34; c++) line[used++] = ',';   /* 列 16..34 空 */
            line[used++] = '\n';
            line[used] = 0;
        } else {
            snprintf(line, sizeof(line),
                     "%s,%s,%s,%s,%s,%d,%s,%s,%s,1,%d,%s,%d,%d,%s,"
                     "%.2f,%.2f,%.2f,%u,%.3f,"
                     "%.2f,%.2f,%.2f,%.2f,%.2f,"
                     "%s,%u,"
                     "%.1f,%.1f,%u,%u,%u,%s,%s\n",
                     r->timestamp, r->video_source, r->sensor, r->usb_device_name,
                     r->resolution, r->scaled, r->quality, r->usb_mode, r->protocol,
                     lcd_ui_active() ? 1 : 0,
                     r->wifi_mode, r->bandwidth_mhz, r->rssi, r->capture_ts_meaning,
                     r->capture_fps, r->encode_fps, r->arrival_fps,
                     (unsigned)r->jpeg_avg_bytes, r->bitrate_mbps,
                     r->latency_mean_ms, r->latency_p95_ms, r->latency_max_ms,
                     r->latency_std_ms, r->encode_ms,
                     r->usb_inherent, (unsigned)r->usb_disconnect_count,
                     r->cpu0_pct, r->cpu1_pct,
                     (unsigned)r->free_heap, (unsigned)r->free_psram,
                     (unsigned)r->drop_frames,
                     r->udp_loss_rate, r->udp_incomplete_rate);
        }
        off += strlcpy(buf + off, line, buflen - off);
        if (off >= buflen - 1) break;
    }
    return off;
}
