/* stream_server.c — 见 stream_server.h
 *
 * WS 二进制协议 v2（向后兼容 S31 页面协议）：
 *   36B 小端应用头 + payload：
 *     [0:4)  magic："MJP1"=JPEG（S31 兼容）；"AVC1"=H.264 Annex-B access unit
 *     [4:8)  fid
 *     [8:16) t_capture_us
 *     [16:24) t_encode_done_us
 *     [24:26) w   [26:28) h
 *     [28]   quality（H264 恒 0）
 *     [29]   source（0=dvp 1=usb）
 *     [30:34) payload len
 *     [34]   key（H264：1=IDR；JPEG 恒 0）   ← v2 新增
 *     [35]   保留
 *   RFC6455 多字节长度为网络序大端（S31 坑 #1：小端写法消息边界失步）。
 */
#include "stream_server.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <lwip/sockets.h>   /* 含 poll/pollfd（lwip 提供，需 LWIP_SOCKET_SELECT） */
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_random.h"
#include "psa/crypto.h"   /* IDF v6：mbedtls 公共 SHA1 API 移入 PSA */
#include "mbedtls/ssl.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "frame_ring.h"
#include "source_if.h"
#include "sdkconfig.h"

extern const uint8_t tab5_cert_start[] asm("_binary_tab5_cert_pem_start");
extern const uint8_t tab5_cert_end[]   asm("_binary_tab5_cert_pem_end");
extern const uint8_t tab5_key_start[]  asm("_binary_tab5_key_pem_start");
extern const uint8_t tab5_key_end[]    asm("_binary_tab5_key_pem_end");

static const char *TAG = "stream_srv";

#define BOUNDARY "framecamtest1234567890"
#define MAX_CLIENTS CONFIG_CAMTEST_MAX_STREAM_CLIENTS
#define SND_TIMEOUT_MS 4000
#define TLS_PORT        8443   /* wss/https 流端口（https 页面用；:81 明文保留） */
#define MAX_TLS_CLIENTS 3      /* 每 TLS 连接 ~40KB 内部 RAM（mbedtls in/out 16KB×2） */

typedef struct {
    int fd;
    bool in_use;
    char type[8];
    char ip[16];
    uint32_t frames;
    uint64_t bytes;
    uint64_t win_start_us;
    stream_client_stat_t pub;
    mbedtls_ssl_context *ssl;   /* TLS 连接（:8443）；明文连接为 NULL */
} client_t;

static client_t s_cli[MAX_CLIENTS];
static SemaphoreHandle_t s_cli_lock;
static volatile uint32_t s_total_frames, s_total_bytes;
static int s_listen_fd = -1;
static int s_tls_listen_fd = -1;
static uint16_t s_port;

/* TLS 全局（stream_server_start 初始化一次）。IDF v6：entropy/ctr_drbg 已移
 * 私有头、conf_rng 已删除——RNG 由 PSA DRBG 提供（tls_global_init 里 init） */
static mbedtls_ssl_config s_tls_cfg;
static mbedtls_x509_crt s_tls_cert;
static mbedtls_pk_context s_tls_key;

static int tls_client_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (s_cli[i].in_use && s_cli[i].ssl) n++;
    return n;
}

/* mbedtls BIO：直连 lwip fd（阻塞语义由 socket 选项控制） */
static int tls_bio_send(void *ctx, const unsigned char *buf, size_t len)
{
    int n = send((int)(intptr_t)ctx, buf, len, 0);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return MBEDTLS_ERR_SSL_WANT_WRITE;
    return n;
}
static int tls_bio_recv(void *ctx, unsigned char *buf, size_t len)
{
    int n = recv((int)(intptr_t)ctx, buf, len, 0);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return MBEDTLS_ERR_SSL_WANT_READ;
    return n;
}
static int tls_bio_recv_nb(void *ctx, unsigned char *buf, size_t len)
{
    int n = recv((int)(intptr_t)ctx, buf, len, MSG_DONTWAIT);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return MBEDTLS_ERR_SSL_WANT_READ;
    return n;
}

/* TLS 握手（fd 已设 RCV/SNDTIMEO；WANT_* 重试） */
static mbedtls_ssl_context *tls_handshake(int fd)
{
    mbedtls_ssl_context *ssl = calloc(1, sizeof(*ssl));
    if (!ssl) return NULL;
    mbedtls_ssl_init(ssl);
    if (mbedtls_ssl_setup(ssl, &s_tls_cfg) != 0) goto fail;
    mbedtls_ssl_set_bio(ssl, (void *)(intptr_t)fd, tls_bio_send, tls_bio_recv, NULL);
    int spins = 0;
    int r;
    while ((r = mbedtls_ssl_handshake(ssl)) != 0) {
        if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) {
            if (++spins > 3000) goto fail;   /* 每次自旋 ~几 ms，上限 ≈10s */
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        goto fail;   /* 真错误（证书/协议） */
    }
    /* 握手完成：恢复与明文一致的阻塞语义（无读超时；发送超时保留） */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return ssl;
fail:
    mbedtls_ssl_free(ssl);
    free(ssl);
    return NULL;
}

static void tls_teardown(client_t *c)
{
    if (c->ssl) {
        mbedtls_ssl_close_notify(c->ssl);
        mbedtls_ssl_free(c->ssl);
        free(c->ssl);
        c->ssl = NULL;
    }
}

/* ---------------- 客户端统一读写（明文/TLS 透明） ---------------- */
/* 返回 0=成功，-1=连接失败（客户端应退出） */
static int cli_send(client_t *c, const void *buf, size_t len)
{
    if (!c->ssl) {
        const uint8_t *p = buf;
        while (len) {
            int n = send(c->fd, p, len, 0);
            if (n <= 0) return -1;
            p += n; len -= n;
        }
        return 0;
    }
    const uint8_t *p = buf;
    while (len) {
        int n = mbedtls_ssl_write(c->ssl, p, len);
        if (n == MBEDTLS_ERR_SSL_WANT_WRITE || n == MBEDTLS_ERR_SSL_WANT_READ) {
            vTaskDelay(pdMS_TO_TICKS(2));   /* SNDTIMEO 下防自旋 */
            continue;
        }
        if (n <= 0) return -1;
        p += n; len -= n;
    }
    return 0;
}
/* 非阻塞读：>0 数据；0 对端关闭；-1 且 errno=EAGAIN/EWOULDBLOCK=无数据 */
static int cli_recv(client_t *c, void *buf, size_t len)
{
    if (!c->ssl)
        return recv(c->fd, buf, len, MSG_DONTWAIT);
    mbedtls_ssl_set_bio(c->ssl, (void *)(intptr_t)c->fd, tls_bio_send, tls_bio_recv_nb, NULL);
    int n = mbedtls_ssl_read(c->ssl, buf, len);
    mbedtls_ssl_set_bio(c->ssl, (void *)(intptr_t)c->fd, tls_bio_send, tls_bio_recv, NULL);
    if (n > 0) return n;
    if (n == 0 || n == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY || n == MBEDTLS_ERR_SSL_CONN_EOF)
        return 0;
    if (n == MBEDTLS_ERR_SSL_WANT_READ) { errno = EAGAIN; return -1; }
    return -1;
}

/* ---------------- 客户端管理 ---------------- */
static client_t *client_alloc(void)
{
    xSemaphoreTake(s_cli_lock, portMAX_DELAY);
    client_t *c = NULL;
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (!s_cli[i].in_use) { c = &s_cli[i]; memset(c, 0, sizeof(*c)); c->in_use = true; break; }
    xSemaphoreGive(s_cli_lock);
    return c;
}
static void client_free(client_t *c)
{
    xSemaphoreTake(s_cli_lock, portMAX_DELAY);
    c->in_use = false;
    xSemaphoreGive(s_cli_lock);
}
static void client_tick(client_t *c)
{
    uint64_t now = esp_timer_get_time();
    if (now - c->win_start_us >= 1000000) {
        float sec = (now - c->win_start_us) / 1e6;
        c->pub.fps = c->frames / sec;
        c->pub.mbps = c->bytes * 8 / sec / 1e6;
        c->pub.frames += c->frames;
        c->frames = 0; c->bytes = 0; c->win_start_us = now;
    }
}

/* ---------------- MJPEG 客户端 ---------------- */
static int send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len) {
        int n = send(fd, p, len, 0);
        if (n <= 0) return -1;
        p += n; len -= n;
    }
    return 0;
}

static void mjpeg_client_task(void *arg)
{
    client_t *c = arg;
    frame_ring_t *ring = src_if_ring();
    SemaphoreHandle_t notify = xSemaphoreCreateBinary();
    uint32_t last_fid = frame_ring_last_fid(ring);
    char hdr[512];

    snprintf(hdr, sizeof(hdr),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY "\r\n"
             "Access-Control-Allow-Origin: *\r\n"
             "Cache-Control: no-store\r\n"
             "\r\n");
    if (cli_send(c, hdr, strlen(hdr)) != 0) goto done;

    frame_ring_register(ring, notify);
    ESP_LOGI(TAG, "mjpeg client %s connected", c->ip);
    for (;;) {
        xSemaphoreTake(notify, pdMS_TO_TICKS(500));
        frame_slot_t *s = frame_ring_acquire(ring, last_fid);
        if (!s) {
            if (!c->in_use) break;
            continue;
        }
        if (s->codec != FRAME_CODEC_JPEG) {   /* H264 帧无法走 MJPEG part */
            frame_ring_release(ring, s);
            continue;
        }
        last_fid = s->fid;
        int n = snprintf(hdr, sizeof(hdr),
                         "\r\n--" BOUNDARY "\r\n"
                         "Content-Type: image/jpeg\r\n"
                         "Content-Length: %u\r\n"
                         "X-Frame-Id: %lu\r\n"
                         "X-Capture-Us: %llu\r\n"
                         "X-Encode-Us: %llu\r\n"
                         "X-Jpeg-Len: %u\r\n"
                         "X-Sensor: %s\r\n"
                         "X-Res: %ux%u\r\n"
                         "X-Quality: %u\r\n"
                         "X-Source: %s\r\n"
                         "X-Scaled: %u\r\n"
                         "X-Ts-Meaning: %s\r\n"
                         "\r\n",
                         (unsigned)s->len, (unsigned long)s->fid,
                         (unsigned long long)s->t_capture_us,
                         (unsigned long long)s->t_encode_done_us,
                         (unsigned)s->len, src_if_info()->sensor_name,
                         s->w, s->h, s->quality,
                         src_if_source_name((video_source_t)s->source),
                         s->scaled,
                         src_if_ts_meaning_name((ts_meaning_t)s->ts_meaning));
        int rc = cli_send(c, hdr, n) || cli_send(c, s->data, s->len) ||
                 cli_send(c, "\r\n", 2);
        frame_ring_release(ring, s);
        if (rc != 0) break;
        c->frames++; c->bytes += s->len + n + 2;
        s_total_frames++; s_total_bytes += s->len + n + 2;
        client_tick(c);
        vTaskDelay(1);
    }
    frame_ring_unregister(ring, notify);
    vSemaphoreDelete(notify);
done:
    ESP_LOGI(TAG, "mjpeg client %s gone", c->ip);
    tls_teardown(c);
    close(c->fd);
    client_free(c);
    vTaskDelete(NULL);
}

/* ---------------- WebSocket 客户端 ---------------- */
static void ws_send_frame(client_t *c, const frame_slot_t *s)
{
    bool h264 = (s->codec == FRAME_CODEC_H264);
    uint8_t hdr[36] = {0};
    memcpy(hdr, h264 ? "AVC1" : "MJP1", 4);
    memcpy(hdr + 4, &s->fid, 4);
    memcpy(hdr + 8, &s->t_capture_us, 8);
    memcpy(hdr + 16, &s->t_encode_done_us, 8);
    uint16_t w = s->w, h = s->h;
    memcpy(hdr + 24, &w, 2);
    memcpy(hdr + 26, &h, 2);
    hdr[28] = s->quality;
    hdr[29] = s->source;
    uint32_t len = s->len;
    memcpy(hdr + 30, &len, 4);
    hdr[34] = s->key;
    /* 两段 copy（头 + 数据）各自 send 会破坏 WS 消息边界，必须一次发出 */
    uint8_t *msg = malloc(36 + s->len);
    if (!msg) return;
    memcpy(msg, hdr, 36);
    memcpy(msg + 36, s->data, s->len);
    /* RFC6455：多字节长度是网络序大端（S31 坑 #1） */
    uint8_t wh[10];
    int whn = 0;
    wh[whn++] = 0x82;
    size_t total = s->len + 36;
    if (total < 126) {
        wh[whn++] = total;
    } else if (total < 65536) {
        wh[whn++] = 126;
        wh[whn++] = (uint8_t)(total >> 8);
        wh[whn++] = (uint8_t)(total & 0xFF);
    } else {
        wh[whn++] = 127;
        for (int i = 7; i >= 0; i--) wh[whn++] = (uint8_t)((uint64_t)total >> (i * 8));
    }
    int rc = cli_send(c, wh, whn) || cli_send(c, msg, 36 + s->len);
    free(msg);
    if (rc == 0) {
        c->frames++; c->bytes += s->len + 36;
        s_total_frames++; s_total_bytes += s->len + 36;
        client_tick(c);
    } else {
        c->frames = -1; /* 标记退出 */
    }
    vTaskDelay(1);
}

static void ws_client_task(void *arg)
{
    client_t *c = arg;
    frame_ring_t *ring = src_if_ring();
    SemaphoreHandle_t notify = xSemaphoreCreateBinary();
    uint32_t last_fid = frame_ring_last_fid(ring);
    uint8_t rbuf[512];

    frame_ring_register(ring, notify);
    ESP_LOGI(TAG, "ws client %s connected", c->ip);
    for (;;) {
        xSemaphoreTake(notify, pdMS_TO_TICKS(500));
        frame_slot_t *s = frame_ring_acquire(ring, last_fid);
        if (s) {
            last_fid = s->fid;
            ws_send_frame(c, s);
            frame_ring_release(ring, s);
            if ((int)c->frames < 0) break;
        }
        int n = cli_recv(c, rbuf, sizeof(rbuf));
        if (n == 0 || (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN)) break;
        if (n >= 2 && (rbuf[0] & 0x0F) == 0x8) break;
        if (!c->in_use) break;
    }
    frame_ring_unregister(ring, notify);
    vSemaphoreDelete(notify);
    ESP_LOGI(TAG, "ws client %s gone", c->ip);
    tls_teardown(c);
    close(c->fd);
    client_free(c);
    vTaskDelete(NULL);
}

/* WS 握手：Sec-WebSocket-Accept = base64(SHA1(key + GUID))（psa，IDF v6） */
static int ws_handshake_reply(client_t *c, const char *key)
{
    static bool psa_ready;
    if (!psa_ready) { psa_crypto_init(); psa_ready = true; }
    char buf[128];
    snprintf(buf, sizeof(buf), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key);
    unsigned char sha[20];
    size_t sha_len = 0;
    if (psa_hash_compute(PSA_ALG_SHA_1, (const uint8_t *)buf, strlen(buf),
                         sha, sizeof(sha), &sha_len) != PSA_SUCCESS || sha_len != 20) {
        return -1;
    }
    static const char *b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char out[32];
    int o = 0;
    for (int i = 0; i < 20; i += 3) {
        uint32_t v = sha[i] << 16 | (i + 1 < 20 ? sha[i + 1] << 8 : 0) | (i + 2 < 20 ? sha[i + 2] : 0);
        out[o++] = b64[(v >> 18) & 63];
        out[o++] = b64[(v >> 12) & 63];
        out[o++] = (i + 1 < 20) ? b64[(v >> 6) & 63] : '=';
        out[o++] = (i + 2 < 20) ? b64[v & 63] : '=';
    }
    out[o] = 0;
    char resp[160];
    int n = snprintf(resp, sizeof(resp),
                     "HTTP/1.1 101 Switching Protocols\r\n"
                     "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                     "Sec-WebSocket-Accept: %s\r\n\r\n", out);
    return cli_send(c, resp, n);
}

/* ---------------- accept 循环（:81 明文 + :8443 TLS 双监听） ---------------- */
/* 阻塞读一行（明文/TLS 通用；总超时靠 socket 选项） */
static int cli_read_line(client_t *c, char *buf, int cap)
{
    int n = 0;
    while (n < cap - 1) {
        char ch;
        int r = c->ssl ? mbedtls_ssl_read(c->ssl, (unsigned char *)&ch, 1)
                       : recv(c->fd, &ch, 1, 0);
        if (c->ssl && (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE)) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (r <= 0) break;
        buf[n++] = ch;
        if (ch == '\n') break;
    }
    buf[n] = 0;
    return n;
}

/* 读到 \r\n\r\n（WS 剩余头；明文/TLS 通用） */
static int cli_read_headers(client_t *c, char *buf, int cap)
{
    int total = 0;
    while (total < cap - 1) {
        int r = c->ssl ? mbedtls_ssl_read(c->ssl, (unsigned char *)buf + total, 1)
                       : recv(c->fd, buf + total, 1, 0);
        if (c->ssl && (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE)) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (r <= 0) break;
        total += r;
        buf[total] = 0;
        if (total >= 4 && strstr(buf + (total > 4 ? total - 4 : 0), "\r\n\r\n")) break;
    }
    buf[total] = 0;
    return total;
}

static void accept_one(int listen_fd, bool is_tls)
{
    struct sockaddr_in ca; socklen_t cl = sizeof(ca);
    int fd = accept(listen_fd, (struct sockaddr *)&ca, &cl);
    if (fd < 0) return;

    client_t *c = client_alloc();
    if (!c) {
        const char *busy = "HTTP/1.1 503 Busy\r\nConnection: close\r\n\r\n";
        if (!is_tls) send(fd, busy, strlen(busy), 0);
        close(fd);
        return;
    }
    struct timeval tv = { .tv_sec = SND_TIMEOUT_MS / 1000, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if (is_tls) {   /* 握手期读超时（成功后恢复无限阻塞） */
        struct timeval rt = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rt, sizeof(rt));
    }
    int nd = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
    c->fd = fd;
    strlcpy(c->ip, inet_ntoa(ca.sin_addr), sizeof(c->ip));
    strlcpy(c->pub.ip, c->ip, sizeof(c->pub.ip));
    c->win_start_us = esp_timer_get_time();

    if (is_tls) {
        if (tls_client_count() >= MAX_TLS_CLIENTS) {
            ESP_LOGW(TAG, "TLS 客户端超限（%d），拒绝", MAX_TLS_CLIENTS);
            close(fd);
            client_free(c);
            return;
        }
        c->ssl = tls_handshake(fd);
        if (!c->ssl) {
            ESP_LOGW(TAG, "TLS 握手失败（%s）", c->ip);
            close(fd);
            client_free(c);
            return;
        }
    }

    char req[256];
    cli_read_line(c, req, sizeof(req));
    char *key_hdr = strstr(req, "Sec-WebSocket-Key");
    if (strncmp(req, "GET /ws", 7) == 0 || key_hdr) {
        char rest[512];
        cli_read_headers(c, rest, sizeof(rest));
        char *key = strstr(rest, "Sec-WebSocket-Key:");
        if (!key) key = strstr(rest, "sec-websocket-key:");
        if (key && strncmp(req, "GET /ws", 7) == 0) {
            key += strlen("Sec-WebSocket-Key:");
            while (*key == ' ') key++;
            char *e = strstr(key, "\r\n");
            if (e) *e = 0;
            ws_handshake_reply(c, key);
            strlcpy(c->type, "ws", sizeof(c->type));
            strlcpy(c->pub.type, "ws", sizeof(c->pub.type));
            if (xTaskCreatePinnedToCore(ws_client_task, "ws_cli", 4096, c, 6, NULL, 0) != pdPASS) {
                tls_teardown(c);
                close(fd); client_free(c);
            }
            return;
        }
        const char *bad = "HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n";
        cli_send(c, bad, strlen(bad));
        tls_teardown(c);
        close(fd); client_free(c);
        return;
    }
    if (strncmp(req, "GET /stream", 11) == 0) {
        strlcpy(c->type, "mjpeg", sizeof(c->type));
        strlcpy(c->pub.type, "mjpeg", sizeof(c->pub.type));
        if (xTaskCreatePinnedToCore(mjpeg_client_task, "mjp_cli", 4096, c, 6, NULL, 0) != pdPASS) {
            tls_teardown(c);
            close(fd); client_free(c);
        }
        return;
    }
    const char *nf = "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
    cli_send(c, nf, strlen(nf));
    tls_teardown(c);
    close(fd); client_free(c);
}

static void accept_task(void *arg)
{
    struct pollfd pfds[2];
    while (1) {
        pfds[0].fd = s_listen_fd;
        pfds[0].events = POLLIN;
        pfds[1].fd = s_tls_listen_fd >= 0 ? s_tls_listen_fd : s_listen_fd;
        pfds[1].events = POLLIN;
        int r = poll(pfds, s_tls_listen_fd >= 0 ? 2 : 1, 200);
        if (r <= 0) continue;
        if (pfds[0].revents & POLLIN) accept_one(s_listen_fd, false);
        if (s_tls_listen_fd >= 0 && (pfds[1].revents & POLLIN)) accept_one(s_tls_listen_fd, true);
    }
}

static esp_err_t tls_global_init(void)
{
    /* PEM 解析要求 NUL 终止——embed blob 不保证尾部为 0，拷入静态缓冲补终止 */
    static char cert_buf[4096], key_buf[4096];
    size_t cert_len = tab5_cert_end - tab5_cert_start;
    size_t key_len = tab5_key_end - tab5_key_start;
    if (cert_len >= sizeof(cert_buf) || key_len >= sizeof(key_buf)) return ESP_FAIL;
    memcpy(cert_buf, tab5_cert_start, cert_len); cert_buf[cert_len] = 0;
    memcpy(key_buf, tab5_key_start, key_len); key_buf[key_len] = 0;

    psa_crypto_init();   /* IDF v6：TLS RNG 由 PSA DRBG 提供（无 conf_rng API） */
    mbedtls_x509_crt_init(&s_tls_cert);
    mbedtls_pk_init(&s_tls_key);
    int r = mbedtls_x509_crt_parse(&s_tls_cert, (const unsigned char *)cert_buf, cert_len + 1);
    if (r != 0) { ESP_LOGE(TAG, "cert parse=%d", r); return ESP_FAIL; }
    r = mbedtls_pk_parse_key(&s_tls_key, (const unsigned char *)key_buf, key_len + 1, NULL, 0);
    if (r != 0) { ESP_LOGE(TAG, "key parse=%d", r); return ESP_FAIL; }
    r = mbedtls_ssl_config_defaults(&s_tls_cfg, MBEDTLS_SSL_IS_SERVER,
                                    MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
    if (r != 0) return ESP_FAIL;
    mbedtls_ssl_conf_authmode(&s_tls_cfg, MBEDTLS_SSL_VERIFY_NONE);
    r = mbedtls_ssl_conf_own_cert(&s_tls_cfg, &s_tls_cert, &s_tls_key);
    if (r != 0) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t stream_server_start(uint16_t port)
{
    s_port = port;
    s_cli_lock = xSemaphoreCreateMutex();

    int yes = 1;
    s_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY),
                                .sin_port = htons(port) };
    ESP_RETURN_ON_ERROR(bind(s_listen_fd, (struct sockaddr *)&addr, sizeof(addr)), TAG, "bind");
    ESP_RETURN_ON_ERROR(listen(s_listen_fd, MAX_CLIENTS), TAG, "listen");

    /* :8443 TLS（wss/https 流端口；失败不致命——明文 :81 仍可用） */
    if (tls_global_init() == ESP_OK) {
        s_tls_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        setsockopt(s_tls_listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        struct sockaddr_in taddr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY),
                                     .sin_port = htons(TLS_PORT) };
        if (bind(s_tls_listen_fd, (struct sockaddr *)&taddr, sizeof(taddr)) == ESP_OK &&
            listen(s_tls_listen_fd, MAX_TLS_CLIENTS) == ESP_OK) {
            ESP_LOGI(TAG, "TLS stream server on :%u（wss/https，自签）", TLS_PORT);
        } else {
            ESP_LOGW(TAG, "TLS listen :%u 失败 errno=%d（仅明文 :%u 可用）", TLS_PORT, errno, port);
            close(s_tls_listen_fd);
            s_tls_listen_fd = -1;
        }
    } else {
        ESP_LOGW(TAG, "TLS 初始化失败（仅明文 :%u 可用）", port);
    }

    if (xTaskCreatePinnedToCore(accept_task, "stream_acc", 6144, NULL, 5, NULL, 0) != pdPASS)
        return ESP_FAIL;
    ESP_LOGI(TAG, "stream server on :%u (/stream, /ws)", port);
    return ESP_OK;
}

int stream_server_get_clients(stream_client_stat_t *out, int max)
{
    int n = 0;
    xSemaphoreTake(s_cli_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS && n < max; i++) {
        if (s_cli[i].in_use) out[n++] = s_cli[i].pub;
    }
    xSemaphoreGive(s_cli_lock);
    return n;
}

uint32_t stream_server_frames_sent(void) { return s_total_frames; }
uint32_t stream_server_bytes_sent(void) { return s_total_bytes; }

esp_err_t udp_push_start(uint16_t port)
{
    (void)port;   /* Tab5 版暂不启用 UDP 分片（CONFIG_CAMTEST_ENABLE_UDP 默认关） */
    return ESP_OK;
}
