/* stream_server.c — 见 stream_server.h */
#include "stream_server.h"
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <lwip/sockets.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_random.h"
#include "psa/crypto.h"   /* IDF v6：mbedtls 公共 SHA1 API 移入 PSA */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "frame_ring.h"
#include "source_if.h"
#include "sdkconfig.h"

static const char *TAG = "stream_srv";

#define BOUNDARY "framecamtest1234567890"
#define MAX_CLIENTS CONFIG_CAMTEST_MAX_STREAM_CLIENTS
#define SND_TIMEOUT_MS 4000

typedef struct {
    int fd;
    bool in_use;
    char type[8];
    char ip[16];
    /* 统计（秒窗） */
    uint32_t frames;
    uint64_t bytes;
    uint64_t win_start_us;
    stream_client_stat_t pub;   /* 对外快照 */
} client_t;

static client_t s_cli[MAX_CLIENTS];
static SemaphoreHandle_t s_cli_lock;
static volatile uint32_t s_total_frames, s_total_bytes;
static int s_listen_fd = -1;
static uint16_t s_port;

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
    frame_ring_t *ring = src_if_ring();   /* 环在 init 后永不销毁，跨源切换持续有效 */
    SemaphoreHandle_t notify = xSemaphoreCreateBinary();
    uint32_t last_fid = frame_ring_last_fid(ring);   /* 新客户端从下一帧开始，不给旧帧 */
    char hdr[512];

    snprintf(hdr, sizeof(hdr),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY "\r\n"
             "Access-Control-Allow-Origin: *\r\n"
             "Cache-Control: no-store\r\n"
             "\r\n");
    if (send_all(c->fd, hdr, strlen(hdr)) != 0) goto done;

    frame_ring_register(ring, notify);
    ESP_LOGI(TAG, "mjpeg client %s connected", c->ip);
    for (;;) {
        xSemaphoreTake(notify, pdMS_TO_TICKS(500));
        frame_slot_t *s = frame_ring_acquire(ring, last_fid);
        if (!s) {
            if (!c->in_use) break;   /* 服务关闭 */
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
        int rc = send_all(c->fd, hdr, n) || send_all(c->fd, s->data, s->len) ||
                 send_all(c->fd, "\r\n", 2);
        frame_ring_release(ring, s);
        if (rc != 0) break;   /* 客户端断开 */
        c->frames++; c->bytes += s->len + n + 2;
        s_total_frames++; s_total_bytes += s->len + n + 2;
        client_tick(c);
        /* 发送节奏：短暂让出，避免占满 CPU；PSRAM→socket 大拷贝自身耗时足够 */
        vTaskDelay(1);
    }
    frame_ring_unregister(ring, notify);
    vSemaphoreDelete(notify);
done:
    ESP_LOGI(TAG, "mjpeg client %s gone", c->ip);
    close(c->fd);
    client_free(c);
    vTaskDelete(NULL);
}

/* ---------------- WebSocket 客户端 ---------------- */
static void ws_send_frame(client_t *c, const frame_slot_t *s)
{
    uint8_t hdr[36];
    memcpy(hdr, "MJP1", 4);   /* 与网页约定：头 4 字节 magic，浏览器按小端 u32 校验 0x31504A4D */
    memcpy(hdr + 4, &s->fid, 4);
    memcpy(hdr + 8, &s->t_capture_us, 8);
    memcpy(hdr + 16, &s->t_encode_done_us, 8);
    uint16_t w = s->w, h = s->h;
    memcpy(hdr + 24, &w, 2);
    memcpy(hdr + 26, &h, 2);
    hdr[28] = s->quality;
    hdr[29] = s->source;   /* 0=dvp 1=usb（网页 parseWsFrame 读取） */
    uint32_t len = s->len;
    memcpy(hdr + 30, &len, 4);
    /* 两段 copy（头 + 数据）各自 send 会破坏 WS 消息边界，必须一次发出 */
    uint8_t *msg = malloc(36 + s->len);
    if (!msg) return;
    memcpy(msg, hdr, 36);
    memcpy(msg + 36, s->data, s->len);
    /* WS 二进制帧：FIN=1 opcode=2；长度 7/16/64-bit */
    uint8_t wh[10];
    int whn = 0;
    wh[whn++] = 0x82;
    if (s->len + 36 < 126) {
        wh[whn++] = s->len + 36;
    } else if (s->len + 36 < 65536) {
        wh[whn++] = 126;
        uint16_t l = s->len + 36;
        memcpy(wh + whn, &l, 2); whn += 2;
    } else {
        wh[whn++] = 127;
        uint64_t l = s->len + 36;
        memcpy(wh + whn, &l, 8); whn += 8;
    }
    int rc = send_all(c->fd, wh, whn) || send_all(c->fd, msg, 36 + s->len);
    free(msg);
    if (rc == 0) {
        c->frames++; c->bytes += s->len + 36;
        s_total_frames++; s_total_bytes += s->len + 36;
        client_tick(c);
    } else {
        c->pub.frames = c->pub.frames; /* 断开由 task 退出处理 */
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

    /* 完成 WS 握手（Accept 由 http 握手阶段算好放在 c->ip？否——存于扩展） */
    extern int ws_handshake_reply(int fd, const char *key);
    /* 握手在 accept 线程已发完 101，这里直接进入推送循环 */
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
        /* 非阻塞读：检测客户端 CLOSE/掉线 */
        int n = recv(c->fd, rbuf, sizeof(rbuf), MSG_DONTWAIT);
        if (n == 0 || (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN)) break;
        if (n >= 2 && (rbuf[0] & 0x0F) == 0x8) break;   /* Close 帧 */
        if (!c->in_use) break;
    }
    frame_ring_unregister(ring, notify);
    vSemaphoreDelete(notify);
    ESP_LOGI(TAG, "ws client %s gone", c->ip);
    close(c->fd);
    client_free(c);
    vTaskDelete(NULL);
}

/* WS 握手：Sec-WebSocket-Accept = base64(SHA1(key + GUID)) */
int ws_handshake_reply(int fd, const char *key)
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
    /* 小型 base64 */
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
    return send_all(fd, resp, n);
}

/* ---------------- UDP 分片推送 ---------------- */
static uint16_t s_udp_port;
static void udp_push_task(void *arg)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY),
                                .sin_port = htons(s_udp_port) };
    bind(fd, (struct sockaddr *)&addr, sizeof(addr));
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in peer;
    bool subscribed = false;
    frame_ring_t *ring = src_if_ring();
    SemaphoreHandle_t notify = xSemaphoreCreateBinary();
    uint32_t last_fid = 0;
    static uint8_t buf[40000];   /* 分片缓冲：静态分配（任务栈只有 4 kB） */
    ESP_LOGI(TAG, "udp push ready on :%d", s_udp_port);

    for (;;) {
        if (!subscribed) {
            char tmp[32];
            struct sockaddr_in from; socklen_t fl = sizeof(from);
            int n = recvfrom(fd, tmp, sizeof(tmp) - 1, MSG_DONTWAIT, (struct sockaddr *)&from, &fl);
            if (n > 0) {
                tmp[n] = 0;
                if (strncmp(tmp, "SUBSCRIBE", 9) == 0) {
                    peer = from;
                    subscribed = true;
                    last_fid = frame_ring_last_fid(ring);
                    frame_ring_register(ring, notify);
                    ESP_LOGI(TAG, "udp subscriber %s", inet_ntoa(from.sin_addr));
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }
        xSemaphoreTake(notify, pdMS_TO_TICKS(500));
        frame_slot_t *s = frame_ring_acquire(ring, last_fid);
        if (!s) continue;
        last_fid = s->fid;
        /* 头（小端，24B）：magic UJPG | fid | t_cap | frag_idx | frag_total | frag_len | flags */
        const size_t FRAG_PAYLOAD = 1200;
        uint32_t magic = 0x47504A55;
        size_t total = (s->len + FRAG_PAYLOAD - 1) / FRAG_PAYLOAD;
        for (size_t i = 0; i < total; i++) {
            uint8_t *p = (uint8_t *)buf;
            uint8_t flags = (i == 0 ? 1 : 0) | (i == total - 1 ? 2 : 0);
            uint16_t flen = (i == total - 1) ? (s->len - i * FRAG_PAYLOAD) : FRAG_PAYLOAD;
            memcpy(p, &magic, 4); p += 4;
            memcpy(p, &s->fid, 4); p += 4;
            memcpy(p, &s->t_capture_us, 8); p += 8;
            uint16_t idx = i, tt = total;
            memcpy(p, &idx, 2); p += 2;
            memcpy(p, &tt, 2); p += 2;
            memcpy(p, &flen, 2); p += 2;
            *p++ = flags; *p++ = 0;
            memcpy(p, s->data + i * FRAG_PAYLOAD, flen);
            sendto(fd, buf, 24 + flen, 0, (struct sockaddr *)&peer, sizeof(peer));
        }
        frame_ring_release(ring, s);
        /* 心跳检测：500ms 无帧也没关系；订阅者静默 10 分钟自动退出 */
        static uint64_t last_data = 0;
        uint64_t now = esp_timer_get_time();
        if (s) last_data = now;
        if (last_data && now - last_data > 600ULL * 1000000) {
            subscribed = false;
            frame_ring_unregister(ring, notify);
        }
    }
}

/* ---------------- accept 循环 ---------------- */
static void accept_task(void *arg)
{
    while (1) {
        struct sockaddr_in ca; socklen_t cl = sizeof(ca);
        int fd = accept(s_listen_fd, (struct sockaddr *)&ca, &cl);
        if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        client_t *c = client_alloc();
        if (!c) {   /* 超限：礼貌拒绝 */
            const char *busy = "HTTP/1.1 503 Busy\r\nConnection: close\r\n\r\n";
            send(fd, busy, strlen(busy), 0);
            close(fd);
            continue;
        }
        struct timeval tv = { .tv_sec = SND_TIMEOUT_MS / 1000, .tv_usec = 0 };
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        int nd = 1;   /* 禁 Nagle：MJPEG 小包（part 头）不被凑包延迟（CSV tcp_nodelay=1） */
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
        c->fd = fd;
        strlcpy(c->ip, inet_ntoa(ca.sin_addr), sizeof(c->ip));
        strlcpy(c->pub.ip, c->ip, sizeof(c->pub.ip));
        c->win_start_us = esp_timer_get_time();

        /* 读请求行（client_type 判断） */
        char req[256];
        int n = 0, r;
        while (n < (int)sizeof(req) - 1 &&
               (r = recv(fd, req + n, 1, 0)) == 1) {
            if (req[n] == '\n') break;
            n++;
        }
        req[n] = 0;
        char *key_hdr = strstr(req, "Sec-WebSocket-Key");
        if (strncmp(req, "GET /ws", 7) == 0 || key_hdr) {
            /* WS：需要完整头部拿 Key —— 上面按行读到 \n 只到第一行；这里偷懒再读剩余 */
            char rest[512] = {0};
            int total = 0;
            fd_set rs; struct timeval t0 = { .tv_sec = 1, .tv_usec = 0 };
            FD_ZERO(&rs); FD_SET(fd, &rs);
            while (select(fd + 1, &rs, NULL, NULL, &t0) > 0) {
                r = recv(fd, rest + total, sizeof(rest) - 1 - total, 0);
                if (r <= 0) break;
                total += r;
                if (strstr(rest, "\r\n\r\n")) break;
                FD_ZERO(&rs); FD_SET(fd, &rs);
            }
            rest[total] = 0;
            char *key = strstr(rest, "Sec-WebSocket-Key:");
            if (!key) key = strstr(rest, "sec-websocket-key:");
            if (key && strncmp(req, "GET /ws", 7) == 0) {
                key += strlen("Sec-WebSocket-Key:");
                while (*key == ' ') key++;
                char *e = strstr(key, "\r\n");
                if (e) *e = 0;
                ws_handshake_reply(fd, key);
                strlcpy(c->type, "ws", sizeof(c->type));
                strlcpy(c->pub.type, "ws", sizeof(c->pub.type));
                if (xTaskCreatePinnedToCore(ws_client_task, "ws_cli", 4096, c, 6, NULL, 0) != pdPASS) {
                    close(fd); client_free(c);
                }
                continue;
            }
            const char *bad = "HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n";
            send(fd, bad, strlen(bad), 0);
            close(fd); client_free(c);
            continue;
        }
        if (strncmp(req, "GET /stream", 11) == 0) {
            strlcpy(c->type, "mjpeg", sizeof(c->type));
            strlcpy(c->pub.type, "mjpeg", sizeof(c->pub.type));
            if (xTaskCreatePinnedToCore(mjpeg_client_task, "mjp_cli", 4096, c, 6, NULL, 0) != pdPASS) {
                close(fd); client_free(c);
            }
            continue;
        }
        const char *nf = "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
        send(fd, nf, strlen(nf), 0);
        close(fd); client_free(c);
    }
}

esp_err_t stream_server_start(uint16_t port)
{
    s_port = port;
    s_cli_lock = xSemaphoreCreateMutex();
    s_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY),
                                .sin_port = htons(port) };
    ESP_RETURN_ON_ERROR(bind(s_listen_fd, (struct sockaddr *)&addr, sizeof(addr)), TAG, "bind");
    ESP_RETURN_ON_ERROR(listen(s_listen_fd, MAX_CLIENTS), TAG, "listen");
    if (xTaskCreatePinnedToCore(accept_task, "stream_acc", 4096, NULL, 5, NULL, 0) != pdPASS)
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
#if CONFIG_CAMTEST_ENABLE_UDP
    s_udp_port = port;   /* 必须在建任务前赋值（任务里 bind 用） */
    if (xTaskCreatePinnedToCore(udp_push_task, "udp_push", 4096, NULL, 5, NULL, 0) != pdPASS)
        return ESP_FAIL;
#else
    (void)port;
#endif
    return ESP_OK;
}
