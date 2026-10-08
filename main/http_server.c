/*
 * http_server.c — 管理 API（esp_http_server :80）
 *
 * 端点：
 *   GET  /                单页 Web UI（内嵌离线）
 *   GET  /overlay         光学闭环校验说明页 + 叠加开关
 *   GET  /api/sync        {"t_dev_us":…, "t_wall_ms":…}   时钟同步（Christian）
 *   GET  /api/status      JSON 实时状态（采集源 + USB 状态 + 扫描进度）
 *   POST /api/config      {"source","res","quality","fps_limit","overlay","target_mbps",
 *                         "usb_mode","usb_inherent_ms","boost"/"hifps"/"vts"(DVP 实验)}
 *   GET  /api/scan/start?modes=sta[,ap]&scope=dvp|usb|both
 *   GET  /api/scan/stop
 *   GET  /api/scan/result
 *   GET  /api/scan/csv
 *   POST /api/scan/report 浏览器/udp_receiver 回传（挂到当前扫描组）
 *
 * 时延测量的关键：/api/sync 在"即将写响应"前采样 esp_timer_get_time()，
 * 客户端以 t0+rtt/2 对齐该读数（推导见网页 JS 注释与 README「精度与误差分析」）。
 */
#include "http_server.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "camera_pipeline.h"
#include "source_if.h"
#include "metrics.h"
#include "app_config.h"
#include "scan_ctrl.h"
#include "stream_server.h"
#include "wifi_net.h"
#include "web_ui.h"
#include "lcd_ui.h"
#include "sdkconfig.h"
#include "esp_system.h"

static const char *TAG = "http_api";
static httpd_handle_t s_server;

/* /api/scan/report 收发计数（status.scan.rep_* 暴露：区分「页面没发」vs「服务端没收」） */
static uint32_t s_rep_ok, s_rep_bad;

/* status 缓冲：rows JSON ≈741B × 128 行 + 基础块 ≈ 105KB。原 32768 时 rows 循环
 * 溢出，used 越过 cap 后末尾 snprintf(cap−used) size_t 下溢 → 越界写 → 堆损坏重启
 *（2026-10-09 扫描中实测；响应还越界发送出 0x8c 垃圾字节） */
#define STATUS_CAP 163840

/* PSRAM 大缓冲（httpd 单任务访问，无竞争；懒分配） */
static char *status_buf(void)
{
    static char *buf;
    if (!buf) buf = heap_caps_malloc(STATUS_CAP, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return buf;
}

/* ---------------- / ---------------- */
static esp_err_t h_index(httpd_req_t *req)
{
    const uint8_t *html; size_t len;
    web_ui_get(&html, &len);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)html, len);
}

/* ---------------- /overlay（光学闭环校验说明） ---------------- */
static const char OVERLAY_HTML[] =
"<!DOCTYPE html><html lang=zh-CN><head><meta charset=utf-8>"
"<meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>光学闭环校验 · CamTest</title>"
"<style>body{background:#0d1117;color:#e6edf3;font:15px/1.6 sans-serif;max-width:760px;margin:24px auto;padding:0 16px}"
"code{background:#21262d;padding:1px 5px;border-radius:4px}li{margin:6px 0}"
"button{background:#1f6feb;color:#fff;border:0;border-radius:8px;padding:8px 16px;font-size:14px}</style></head><body>"
"<h1>光学闭环校验（金标准）</h1>"
"<ol>"
"<li>点下面按钮开启<b>画面叠加</b>：图像正中会烧录设备本地毫秒计数（esp_timer，单调钟）。</li>"
"<li>回主页勾选「显示标定毫秒计时器」，把计时器放进摄像头视野（与流同屏即可，无需手机/第二块屏）。</li>"
"<li>用手机<b>连拍 ≥10 张</b>显示器画面（拍的是流+计时器同框）。一张照片同时冻结两块钟："
"画面<b>内容里</b>的计时器 = 摄像头<b>曝光瞬间</b>的真实时间 s；画面里的<b>设备计数</b> = 该帧<b>到达 ESP32 瞬间</b> c。"
"两者之差 c−s 就是「从曝光到到达」的真实耗时①——同一瞬间冻结，<b>不依赖任何时钟同步</b>。</li>"
"<li>把每张照片的 c、s 填进主页 USB 卡片的标定向导：页面自动扣掉两钟的<b>纪元差</b>"
"（计时器从页面加载起算、设备计数从开机起算，差一个常数 = clock offset，不扣会差出一个天文数字），"
"均值一键写入设备。</li>"
"</ol>"
"<p><b>为什么网页时钟同步法测不到①</b>：USB 通路的 t_capture 打点是「帧完整到达 ESP32」"
"（TS_MEANING=frame_arrival），曝光→摄像头内部 ISP/JPEG 编码→USB 传输这一段发生在打点之前、"
"设备完全不可见——它正是①的定义，只能靠光学法让<b>内容自己带出曝光时刻</b>来测。</p>"
"<p><b>USB 摄像头注意</b>：叠加只能在 <code>reencode</code> 模式绘制（直通模式不解码）；"
"开启叠加时若 USB 处于直通模式，设备会自动切换到重编码（MJPEG 档硬解码，帧率不受限）。</p>"
"<button onclick=\"fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({overlay:true})}).then(()=>document.body.append(' 已开启，请回主页用标定向导采样'))\">开启画面毫秒叠加</button>"
"<p>标定后：真实光学端到端 = ① + 网页时钟同步法时延（②+③）。误差来源分析见 README。</p>"
"</body></html>";

static esp_err_t h_overlay(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, OVERLAY_HTML, HTTPD_RESP_USE_STRLEN);
}

/* ---------------- /api/sync ---------------- */
static esp_err_t h_sync(httpd_req_t *req)
{
    /* 在即将写响应前采样设备单调钟 */
    int64_t t_dev = esp_timer_get_time();
    int64_t wall_ms = wifi_net_time_synced() ? (int64_t)(time(NULL)) * 1000 : 0;
    char out[96];
    int n = snprintf(out, sizeof(out), "{\"t_dev_us\":%lld,\"t_wall_ms\":%lld}",
                     (long long)t_dev, (long long)wall_ms);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out, n);
}

/* ---------------- /api/status（采集源 + USB + 扫描进度） ---------------- */
static esp_err_t h_status(httpd_req_t *req)
{
    char *out = status_buf();
    if (!out) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    const size_t cap = STATUS_CAP;
    size_t used = 0;
    /* 追加式 snprintf：返回值是未截断长度，used 一旦 > cap，下一次 cap−used 即
     * size_t 下溢 → 越界写。统一夹取：used 恒 ≤ cap（满则丢 JSON 尾巴，页面表现
     * 为 stale，绝不越界写/越界读）。 */
#define SAPP(...) do { \
        if (used < cap) { \
            int _n = snprintf(out + used, cap - used, __VA_ARGS__); \
            if (_n > 0) used += (size_t)_n; \
            if (used > cap) used = cap; \
        } \
    } while (0)

    src_info_t *ci = src_if_info();
    metrics_t *m = metrics_get();
    wifi_info_t *w = wifi_net_info();
    stream_client_stat_t cli[8];
    int ncli = stream_server_get_clients(cli, 8);

    char clients[512] = "";
    int coff = 0;
    /* 单条 ~75B × 8 客户端 ≈ 600B > 512：不夹取会复现同样的 size_t 下溢越界写 */
    for (int i = 0; i < ncli && coff < (int)sizeof(clients) - 120; i++) {
        coff += snprintf(clients + coff, sizeof(clients) - coff,
                         "%s{\"type\":\"%s\",\"ip\":\"%s\",\"fps\":%.1f,\"mbps\":%.2f}",
                         coff ? "," : "", cli[i].type, cli[i].ip, cli[i].fps, cli[i].mbps);
    }
    char res_list[160] = "";
    src_if_supported_res(res_list, sizeof(res_list));
    lcd_ui_stats_t lcd = lcd_ui_stats();

    SAPP(
        "{\"device\":{"
        "\"source\":\"%s\",\"usb_mode\":\"%s\","
        "\"sensor\":\"%s\",\"usb_device\":\"%s\","
        "\"res\":\"%ux%u\",\"native\":\"%ux%u\",\"scaled\":%s,"
        "\"pix_fmt\":\"%s\",\"fmt\":\"%s\",\"quality\":%u,"
        "\"quality_desc\":\"DVP/USB重编码：0~100 越大画质越高；USB passthrough：画质由摄像头固件决定，不可控\","
        "\"fps\":{\"capture\":%.2f,\"encode\":%.2f,\"send\":%.2f},\"fps_limit\":%d,"
        "\"ts_meaning\":\"%s\","
        "\"jpeg_avg_bytes\":%u,\"bitrate_mbps\":%.3f,"
        "\"cpu0\":%.1f,\"cpu1\":%.1f,"
        "\"free_heap\":%u,\"min_heap\":%u,\"free_psram\":%u,\"min_psram\":%u,"
        "\"drop_capture\":%u,\"drop_encode\":%u,"
        "\"stack\":[%d,%d],"
        "\"uptime_s\":%u,\"rst_reason\":%d,"
        "\"overlay\":%s,\"target_mbps\":%.1f,\"gov_last\":\"%s\",\"hifps\":%d,"
        "\"tcp_nodelay\":1,\"lcd_on\":%d,\"lcd_prev_fps\":%.1f,\"lcd_dec_ms\":%.1f,"
        "\"wifi\":{\"mode\":\"%s\",\"phy\":\"%s\",\"channel\":%d,\"band_mhz\":%d,\"rssi\":%d,\"ssid\":\"%s\"},"
        "\"ip\":\"%s\",\"mdns\":\"%s.local\","
        "\"clients\":[%s],"
        "\"idf\":\"%s\"},",
        src_if_source_name(ci->source), src_if_usb_mode_name(ci->usb_mode),
        ci->sensor_name, ci->usb_device_name,
        ci->w, ci->h, ci->native_w, ci->native_h, ci->scaled ? "true" : "false",
        ci->pix_fmt_str, ci->fmt_name, ci->quality,
        m->cap_fps, m->enc_fps, m->send_fps, ci->fps_limit,
        src_if_ts_meaning_name(ci->ts_meaning),
        (unsigned)m->jpeg_avg_bytes, m->bitrate_mbps,
        m->cpu0, m->cpu1,
        (unsigned)m->free_heap, (unsigned)m->min_heap,
        (unsigned)m->free_psram, (unsigned)m->min_psram,
        (unsigned)m->drop_capture, (unsigned)m->drop_encode,
        m->stack_cap, m->stack_enc, (unsigned)m->uptime_s, esp_reset_reason(),
        src_if_overlay() ? "true" : "false", m->target_mbps, m->gov_last, cam_boost_level(),
        lcd.active ? 1 : 0, lcd.preview_fps, lcd.dec_ms,
        w->mode == WIFI_MODE_STA_M ? "STA" : "AP", w->phy, w->channel, w->band_mhz,
        w->rssi, w->ssid, w->ip, CONFIG_CAMTEST_MDNS_HOSTNAME, clients,
        esp_get_idf_version());

    /* USB 专段：状态/掉线计数/标定值/描述符档位列表（U3/U5） */
    {
        usb_tier_t tiers[USB_TIER_MAX];
        int nt = src_if_usb_get_tiers(tiers, USB_TIER_MAX);
        bool mjpeg = false, yuy2 = false;
        for (int i = 0; i < nt; i++) { if (tiers[i].mjpeg) mjpeg = true; else yuy2 = true; }
        SAPP(
            "\"usb\":{\"state\":\"%s\",\"device\":\"%s\",\"disconnects\":%u,"
            "\"mjpeg\":%s,\"yuy2\":%s,\"inherent_ms\":%.1f,\"tiers\":[",
            src_if_usb_state() == USB_STATE_DISABLED ? "disabled" :
            src_if_usb_state() == USB_STATE_NO_DEVICE ? "no-device" :
            src_if_usb_state() == USB_STATE_DEVICE_READY ? "ready" :
            src_if_usb_state() == USB_STATE_STREAMING ? "streaming" : "error",
            src_if_usb_device_name(), (unsigned)src_if_usb_disconnects(),
            mjpeg ? "true" : "false", yuy2 ? "true" : "false",
            src_if_usb_inherent_ms());
        for (int i = 0; i < nt && used < cap - 600; i++) {
            SAPP(
                             "%s{\"fmt\":\"%s\",\"w\":%u,\"h\":%u,\"fps\":[",
                             i ? "," : "", tiers[i].fmt, tiers[i].w, tiers[i].h);
            for (int f = 0; f < tiers[i].fps_n; f++) {
                SAPP( "%s%d", f ? "," : "", tiers[i].fps[f]);
            }
            SAPP( "]}");
        }
        SAPP( "]},\"supported_res\":[");
    }

    char tmp[160];    strlcpy(tmp, res_list, sizeof(tmp));
    char *tok, *save = NULL;
    bool first = true;
    tok = strtok_r(tmp, ",", &save);
    while (tok && used < cap - 4000) {
        SAPP( "%s\"%s\"", first ? "" : ",", tok);
        first = false;
        tok = strtok_r(NULL, ",", &save);
    }
    SAPP( "],");

    /* 扫描（两级进度：当前源/当前组） */
    static scan_row_t *rows;   /* PSRAM */
    if (!rows) rows = heap_caps_calloc(128, sizeof(scan_row_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int n = rows ? scan_ctrl_rows(rows, 128) : 0;
    scan_prog_t prog;
    scan_ctrl_progress(&prog);
    SAPP(
                     "\"scan\":{\"state\":\"%s\",\"idx\":%d,\"total\":%d,"
                     "\"cur_source\":\"%s\",\"cur_protocol\":\"%s\",\"cur_res\":\"%s\",\"cur_quality\":\"%s\","
                     "\"usb_skipped\":%s,\"rep_ok\":%u,\"rep_bad\":%u,\"rows\":[",
                     prog.state == SCAN_RUNNING ? "running" : prog.state == SCAN_DONE ? "done" : "idle",
                     prog.idx, prog.total, prog.cur_source, prog.cur_protocol,
                     prog.cur_res, prog.cur_quality,
                     prog.usb_skipped ? "true" : "false",
                     (unsigned)s_rep_ok, (unsigned)s_rep_bad);
    for (int i = 0; i < n && used < cap - 1024; i++) {   /* 单行 ~741B + 尾部 ]}} 需留 1024 */
        scan_row_t *r = &rows[i];
        SAPP(
                        "%s{\"timestamp\":\"%s\",\"video_source\":\"%s\",\"sensor\":\"%s\","
                        "\"usb_device_name\":\"%s\",\"resolution\":\"%s\",\"scaled\":%d,"
                        "\"quality\":\"%s\",\"usb_mode\":\"%s\",\"protocol\":\"%s\","
                        "\"wifi_mode\":\"%s\",\"bandwidth_mhz\":%d,\"rssi\":%d,"
                        "\"capture_ts_meaning\":\"%s\","
                        "\"capture_fps\":%.2f,\"encode_fps\":%.2f,\"arrival_fps\":%.2f,"
                        "\"jpeg_avg_bytes\":%u,\"bitrate_mbps\":%.3f,"
                        "\"latency_mean_ms\":%.2f,\"latency_p95_ms\":%.2f,\"latency_max_ms\":%.2f,"
                        "\"latency_std_ms\":%.2f,\"encode_ms\":%.2f,"
                        "\"usb_cam_inherent_latency_ms\":\"%s\",\"usb_disconnect_count\":%u,"
                        "\"cpu0_pct\":%.1f,\"cpu1_pct\":%.1f,\"free_heap\":%u,\"free_psram\":%u,"
                        "\"drop_frames\":%u,\"udp_loss_rate\":\"%s\",\"udp_incomplete_rate\":\"%s\","
                        "\"unsupported\":%s}",
                        i ? "," : "", r->timestamp, r->video_source, r->sensor,
                        r->usb_device_name, r->resolution, r->scaled,
                        r->quality, r->usb_mode, r->protocol,
                        r->wifi_mode, r->bandwidth_mhz, r->rssi,
                        r->capture_ts_meaning,
                        r->capture_fps, r->encode_fps, r->arrival_fps,
                        (unsigned)r->jpeg_avg_bytes, r->bitrate_mbps,
                        r->latency_mean_ms, r->latency_p95_ms, r->latency_max_ms,
                        r->latency_std_ms, r->encode_ms,
                        r->usb_inherent, (unsigned)r->usb_disconnect_count,
                        r->cpu0_pct, r->cpu1_pct,
                        (unsigned)r->free_heap, (unsigned)r->free_psram,
                        (unsigned)r->drop_frames,
                        r->udp_loss_rate, r->udp_incomplete_rate,
                        r->unsupported ? "true" : "false");
    }
    SAPP( "]}}");   /* rows 数组 + scan 对象 + 根对象 */
#undef SAPP

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, out, used);
}

/* ---------------- /api/config ---------------- */
static esp_err_t h_config(httpd_req_t *req)
{
    char body[640] = {0};
    int len = req->content_len;
    if (len <= 0 || len >= (int)sizeof(body))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad length");
    if (httpd_req_recv(req, body, len) <= 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");

    cJSON *j = cJSON_Parse(body);
    if (!j) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");

    /* 配置应用的完整逻辑在 app_config.c（与启动恢复共用同一实现）；
     * 仅全部成功才合并落盘 —— 部分更新语义：只覆盖本次 POST 出现的字段 */
    app_cfg_result_t r;
    app_config_apply(j, &r);
    if (r.ok) app_config_persist(j);
    cJSON_Delete(j);

    src_info_t *ci = src_if_info();
    char reply[256];
    snprintf(reply, sizeof(reply),
             "{\"ok\":%s,\"restarted\":%s,\"source\":\"%s\",\"usb_mode\":\"%s\","
             "\"res\":\"%ux%u\",\"quality\":%u,\"error\":\"%s\"}",
             r.ok ? "true" : "false", r.restarted ? "true" : "false",
             src_if_source_name(ci->source), src_if_usb_mode_name(ci->usb_mode),
             ci->w, ci->h, ci->quality, r.error);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, reply, HTTPD_RESP_USE_STRLEN);
}

/* ---------------- /api/scan 系列 ---------------- */
static esp_err_t h_scan_start(httpd_req_t *req)
{
    bool include_ap = false;
    char scope[16] = "both";
    char q[96] = {0};
    if (httpd_req_get_url_query_str(req, q, sizeof(q) - 1) == ESP_OK) {
        char v[32] = {0};
        if (httpd_query_key_value(q, "modes", v, sizeof(v) - 1) == ESP_OK) {
            if (strstr(v, "ap")) include_ap = true;
        }
        char s[16] = {0};
        if (httpd_query_key_value(q, "scope", s, sizeof(s) - 1) == ESP_OK) {
            if (!strcmp(s, "dvp") || !strcmp(s, "usb")) strlcpy(scope, s, sizeof(scope));
        }
    }
    esp_err_t err = scan_ctrl_start(include_ap, scope);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req,
        err == ESP_OK ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"scan already running, not ready, or no rows in scope\"}",
        HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_scan_stop(httpd_req_t *req)
{
    scan_ctrl_stop();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_scan_result(httpd_req_t *req)
{
    return h_status(req);
}

static esp_err_t h_scan_csv(httpd_req_t *req)
{
    static char *csv;   /* 128 行 × 行缓冲 640B ≈ 82KB 上界：32768 会截断，且
                         * scan_ctrl_csv 返回未截断累加值 → 越界发送 */
    if (!csv) csv = heap_caps_malloc(98304, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!csv) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    int n = scan_ctrl_csv(csv, 98304 - 1);
    if (n < 0 || n > 98303) n = 98303;
    httpd_resp_set_type(req, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=camtest_scan.csv");
    return httpd_resp_send(req, csv, n);
}

static double jnum(const cJSON *j, const char *key)
{
    const cJSON *v = cJSON_GetObjectItem(j, key);
    return (v && cJSON_IsNumber(v)) ? v->valuedouble : 0;
}

static esp_err_t h_scan_report(httpd_req_t *req)
{
    char body[512] = {0};
    int len = req->content_len;
    if (len <= 0 || len >= (int)sizeof(body)) {
        s_rep_bad++;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad length");
    }
    httpd_req_recv(req, body, len);
    cJSON *j = cJSON_Parse(body);
    if (j) {
        scan_ctrl_report(jnum(j, "arrival_fps"), jnum(j, "bitrate_mbps"),
                         jnum(j, "latency_mean_ms"), jnum(j, "latency_p95_ms"),
                         jnum(j, "latency_max_ms"), jnum(j, "latency_std_ms"));
        /* udp_receiver.py 专有字段（浏览器无法测 UDP） */
        const cJSON *ul = cJSON_GetObjectItem(j, "udp_loss_rate");
        const cJSON *ui = cJSON_GetObjectItem(j, "udp_incomplete_rate");
        if ((ul && cJSON_IsNumber(ul)) || (ui && cJSON_IsNumber(ui))) {
            scan_ctrl_report_udp(jnum(j, "udp_loss_rate"), jnum(j, "udp_incomplete_rate"));
        }
        cJSON_Delete(j);
        s_rep_ok++;
    } else {
        s_rep_bad++;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

/* ---------------- 注册 ---------------- */
static const httpd_uri_t s_uris[] = {
    { .uri = "/",                .method = HTTP_GET,  .handler = h_index },
    { .uri = "/overlay",         .method = HTTP_GET,  .handler = h_overlay },
    { .uri = "/api/sync",        .method = HTTP_GET,  .handler = h_sync },
    { .uri = "/api/status",      .method = HTTP_GET,  .handler = h_status },
    { .uri = "/api/config",      .method = HTTP_POST, .handler = h_config },
    { .uri = "/api/scan/start",  .method = HTTP_GET,  .handler = h_scan_start },
    { .uri = "/api/scan/stop",   .method = HTTP_GET,  .handler = h_scan_stop },
    { .uri = "/api/scan/result", .method = HTTP_GET,  .handler = h_scan_result },
    { .uri = "/api/scan/csv",    .method = HTTP_GET,  .handler = h_scan_csv },
    { .uri = "/api/scan/report", .method = HTTP_POST, .handler = h_scan_report },
};

esp_err_t http_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* LWIP 总 socket 池还要留给流服务/UDP，这里保守取 7（若 LWIP 配置更大也够用） */
    cfg.max_open_sockets = 7;
    cfg.max_uri_handlers = 16;      /* v6 已移除同名 Kconfig，只能运行时设 */
    cfg.lru_purge_enable = true;
    cfg.stack_size = 8192;
    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) return err;
    for (size_t i = 0; i < sizeof(s_uris) / sizeof(s_uris[0]); i++) {
        httpd_register_uri_handler(s_server, &s_uris[i]);
    }
    ESP_LOGI(TAG, "http api server on :%u", cfg.server_port);
    return ESP_OK;
}
