/*
 * http_server.c — 管理 API（esp_http_server :80）— Tab5 版
 *
 * 端点：
 *   GET  /                单页 Web UI（内嵌离线）
 *   GET  /overlay         光学闭环校验说明页 + 叠加开关
 *   GET  /api/sync        {"t_dev_us":…}   时钟同步（Christian）
 *   GET  /api/status      JSON 实时状态
 *   POST /api/config      {"source","res","quality","fps_limit","overlay","target_mbps",
 *                         "usb_mode","usb_inherent_ms","h264_kbps"}
 * （S31 的 /api/scan 系列待 DVP 源接入后移植）
 *
 * 时延测量关键：/api/sync 在"即将写响应"前采样 esp_timer_get_time()。
 */
#include "http_server.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_https_server.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "source_if.h"
#include "metrics.h"
#include "app_config.h"
#include "stream_server.h"
#include "wifi_net.h"
#include "web_ui.h"
#include "sdkconfig.h"

static const char *TAG = "http_api";
static httpd_handle_t s_server;
static httpd_handle_t s_server_https;

/* 自签证书（构建期生成，见 certs/；SAN 含 tab5-cam.local/localhost/127.0.0.1/192.168.3.44）。
 * WebCodecs 是 Secure-Context-Only API——:443 https 入口让 Chrome/Edge 直连也能
 * 硬解 H.264（http 入口保留，页面自动落到 JPEG 预览流） */
extern const uint8_t tab5_cert_start[] asm("_binary_tab5_cert_pem_start");
extern const uint8_t tab5_cert_end[]   asm("_binary_tab5_cert_pem_end");
extern const uint8_t tab5_key_start[]  asm("_binary_tab5_key_pem_start");
extern const uint8_t tab5_key_end[]    asm("_binary_tab5_key_pem_end");

static char *status_buf(void)
{
    static char *buf;
    if (!buf) buf = heap_caps_malloc(32768, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
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
"<title>光学闭环校验 · CamTest Tab5</title>"
"<style>body{background:#0d1117;color:#e6edf3;font:15px/1.6 sans-serif;max-width:760px;margin:24px auto;padding:0 16px}"
"code{background:#21262d;padding:1px 5px;border-radius:4px}li{margin:6px 0}"
"button{background:#1f6feb;color:#fff;border:0;border-radius:8px;padding:8px 16px;font-size:14px}</style></head><body>"
"<h1>光学闭环校验（金标准）</h1>"
"<ol>"
"<li>点下面按钮开启<b>画面叠加</b>：图像正中会烧录设备本地毫秒计数（esp_timer，单调钟）。</li>"
"<li>回主页勾选「显示标定毫秒计时器」，把计时器放进摄像头视野。</li>"
"<li>截图 ≥10 张（流+计时器同框）。一张截图同时冻结两块钟："
"画面<b>内容里</b>的计时器 = 曝光瞬间真实时间 s；画面里的<b>设备计数</b> = 该帧到达 ESP32 瞬间 c。"
"c−s 即「从曝光到到达」的真实耗时①，不依赖时钟同步。</li>"
"<li>把每张的 c、s 填进主页标定向导（自动扣纪元差）。</li>"
"</ol>"
"<p><b>叠加只能在 h264/reencode 模式绘制</b>（直通不解码）；开启叠加时若 USB 处于直通模式，"
"设备会自动切换到 h264 模式。</p>"
"<button onclick=\"fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({overlay:true})}).then(()=>document.body.append(' 已开启，请回主页用标定向导采样'))\">开启画面毫秒叠加</button>"
"</body></html>";

static esp_err_t h_overlay(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, OVERLAY_HTML, HTTPD_RESP_USE_STRLEN);
}

/* ---------------- /api/sync ---------------- */
static esp_err_t h_sync(httpd_req_t *req)
{
    int64_t t_dev = esp_timer_get_time();
    int64_t wall_ms = wifi_net_time_synced() ? (int64_t)(time(NULL)) * 1000 : 0;
    char out[96];
    int n = snprintf(out, sizeof(out), "{\"t_dev_us\":%lld,\"t_wall_ms\":%lld}",
                     (long long)t_dev, (long long)wall_ms);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out, n);
}

/* ---------------- /api/status ---------------- */
static esp_err_t h_status(httpd_req_t *req)
{
    char *out = status_buf();
    if (!out) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    const size_t cap = 32768;
    size_t used = 0;

    src_info_t *ci = src_if_info();
    metrics_t *m = metrics_get();
    wifi_info_t *w = wifi_net_info();
    stream_client_stat_t cli[8];
    int ncli = stream_server_get_clients(cli, 8);

    char clients[512] = "";
    int coff = 0;
    for (int i = 0; i < ncli; i++) {
        coff += snprintf(clients + coff, sizeof(clients) - coff,
                         "%s{\"type\":\"%s\",\"ip\":\"%s\",\"fps\":%.1f,\"mbps\":%.2f}",
                         coff ? "," : "", cli[i].type, cli[i].ip, cli[i].fps, cli[i].mbps);
    }
    char res_list[160] = "";
    src_if_supported_res(res_list, sizeof(res_list));

    used += snprintf(out + used, cap - used,
        "{\"device\":{"
        "\"source\":\"%s\",\"usb_mode\":\"%s\","
        "\"sensor\":\"%s\",\"usb_device\":\"%s\","
        "\"res\":\"%ux%u\",\"native\":\"%ux%u\",\"scaled\":%s,"
        "\"pix_fmt\":\"%s\",\"fmt\":\"%s\",\"quality\":%u,"
        "\"fps\":{\"capture\":%.2f,\"encode\":%.2f,\"send\":%.2f,\"preview\":%.2f},\"fps_limit\":%d,"
        "\"ts_meaning\":\"%s\","
        "\"jpeg_avg_bytes\":%u,\"bitrate_mbps\":%.3f,"
        "\"cpu0\":%.1f,\"cpu1\":%.1f,"
        "\"free_heap\":%u,\"min_heap\":%u,\"free_psram\":%u,\"min_psram\":%u,"
        "\"drop_capture\":%u,\"drop_encode\":%u,"
        "\"stack\":[%d,%d],"
        "\"uptime_s\":%u,"
        "\"overlay\":%s,\"target_mbps\":%.1f,\"gov_last\":\"%s\","
        "\"h264_kbps\":%u,"
        "\"tcp_nodelay\":1,"
        "\"wifi\":{\"mode\":\"%s\",\"phy\":\"%s\",\"channel\":%d,\"band_mhz\":%d,\"rssi\":%d,\"ssid\":\"%s\"},"
        "\"ip\":\"%s\",\"mdns\":\"%s.local\","
        "\"clients\":[%s],"
        "\"idf\":\"%s\"},",
        src_if_source_name(ci->source), src_if_usb_mode_name(ci->usb_mode),
        ci->sensor_name, ci->usb_device_name,
        ci->w, ci->h, ci->native_w, ci->native_h, ci->scaled ? "true" : "false",
        ci->pix_fmt_str, ci->fmt_name, ci->quality,
        m->cap_fps, m->enc_fps, m->send_fps, m->pv_fps, ci->fps_limit,
        src_if_ts_meaning_name(ci->ts_meaning),
        (unsigned)m->jpeg_avg_bytes, m->bitrate_mbps,
        m->cpu0, m->cpu1,
        (unsigned)m->free_heap, (unsigned)m->min_heap,
        (unsigned)m->free_psram, (unsigned)m->min_psram,
        (unsigned)m->drop_capture, (unsigned)m->drop_encode,
        m->stack_cap, m->stack_enc, (unsigned)m->uptime_s,
        src_if_overlay() ? "true" : "false", m->target_mbps, m->gov_last,
        (unsigned)src_if_h264_kbps(),
        w->mode == WIFI_MODE_STA_M ? "STA" : "AP", w->phy, w->channel, w->band_mhz,
        w->rssi, w->ssid, w->ip, CONFIG_CAMTEST_MDNS_HOSTNAME, clients,
        esp_get_idf_version());

    /* USB 专段 */
    {
        usb_tier_t tiers[USB_TIER_MAX];
        int nt = src_if_usb_get_tiers(tiers, USB_TIER_MAX);
        bool mjpeg = false, yuy2 = false;
        for (int i = 0; i < nt; i++) { if (tiers[i].mjpeg) mjpeg = true; else yuy2 = true; }
        used += snprintf(out + used, cap - used,
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
            used += snprintf(out + used, cap - used,
                             "%s{\"fmt\":\"%s\",\"w\":%u,\"h\":%u,\"fps\":[",
                             i ? "," : "", tiers[i].fmt, tiers[i].w, tiers[i].h);
            for (int f = 0; f < tiers[i].fps_n; f++) {
                used += snprintf(out + used, cap - used, "%s%d", f ? "," : "", tiers[i].fps[f]);
            }
            used += snprintf(out + used, cap - used, "]}");
        }
        used += snprintf(out + used, cap - used, "]},\"supported_res\":[");
    }

    char tmp[160];    strlcpy(tmp, res_list, sizeof(tmp));
    char *tok, *save = NULL;
    bool first = true;
    tok = strtok_r(tmp, ",", &save);
    while (tok && used < cap - 4000) {
        used += snprintf(out + used, cap - used, "%s\"%s\"", first ? "" : ",", tok);
        first = false;
        tok = strtok_r(NULL, ",", &save);
    }
    used += snprintf(out + used, cap - used, "]}");   /* supported_res 数组 + 根对象（Tab5 无 scan 段） */

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

/* ---------------- 注册 ---------------- */
static const httpd_uri_t s_uris[] = {
    { .uri = "/",                .method = HTTP_GET,  .handler = h_index },
    { .uri = "/overlay",         .method = HTTP_GET,  .handler = h_overlay },
    { .uri = "/api/sync",        .method = HTTP_GET,  .handler = h_sync },
    { .uri = "/api/status",      .method = HTTP_GET,  .handler = h_status },
    { .uri = "/api/config",      .method = HTTP_POST, .handler = h_config },
};

esp_err_t http_server_start(void)
{
    /* :80 明文（全功能；http 页面走 JPEG 预览流） */
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = 7;
    cfg.max_uri_handlers = 16;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 8192;
    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) return err;
    for (size_t i = 0; i < sizeof(s_uris) / sizeof(s_uris[0]); i++) {
        httpd_register_uri_handler(s_server, &s_uris[i]);
    }
    ESP_LOGI(TAG, "http api server on :%u", cfg.server_port);

    /* :443 https（同一组 handler；每 SSL 连接 ~40KB 内部 RAM → 限 4 socket）。
     * ★ PEM 解析要求 NUL 终止——embed blob 不带终止符，必须拷贝补 NUL
     *   （否则证书解析失败，握手期连接被 RST，真机踩过） */
    static char https_cert_buf[4096], https_key_buf[4096];
    size_t cert_len = tab5_cert_end - tab5_cert_start;
    size_t key_len = tab5_key_end - tab5_key_start;
    if (cert_len < sizeof(https_cert_buf) && key_len < sizeof(https_key_buf)) {
        memcpy(https_cert_buf, tab5_cert_start, cert_len); https_cert_buf[cert_len] = 0;
        memcpy(https_key_buf, tab5_key_start, key_len); https_key_buf[key_len] = 0;
        httpd_ssl_config_t scfg = HTTPD_SSL_CONFIG_DEFAULT();
        scfg.servercert = (const uint8_t *)https_cert_buf;
        scfg.servercert_len = cert_len + 1;
        scfg.prvtkey_pem = (const uint8_t *)https_key_buf;
        scfg.prvtkey_len = key_len + 1;
        scfg.httpd.max_uri_handlers = 16;
        scfg.httpd.max_open_sockets = 4;
        scfg.httpd.lru_purge_enable = true;
        scfg.httpd.stack_size = 8192;
        err = httpd_ssl_start(&s_server_https, &scfg);
        if (err != ESP_OK) {
            /* https 失败不致命：http 入口仍可用 */
            ESP_LOGE(TAG, "https server 启动失败：%s（仅 http 可用）", esp_err_to_name(err));
            return ESP_OK;
        }
        for (size_t i = 0; i < sizeof(s_uris) / sizeof(s_uris[0]); i++) {
            httpd_register_uri_handler(s_server_https, &s_uris[i]);
        }
        ESP_LOGI(TAG, "https api server on :%u（自签证书，浏览器首次访问需信任）", scfg.port_secure);
    }
    return ESP_OK;
}
