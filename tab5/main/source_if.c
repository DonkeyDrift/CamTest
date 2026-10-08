/*
 * source_if.c — 统一采集源调度（移植自 S31）：vtable 分发 + 帧环持有
 *
 * Tab5 差异：帧环由本层创建（S31 在 camera_pipeline 内）；DVP 后端暂未接入，
 * 枚举位保留（esp_video/SC202CS 后续并入）。
 */
#include "source_if.h"
#include <string.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "src_if";

/* ---------- 后端 vtable ---------- */
typedef struct {
    esp_err_t (*init)(void);
    esp_err_t (*start)(void);
    void (*stop)(void);
    esp_err_t (*apply)(int w, int h, uint8_t quality, int fps_limit);
    esp_err_t (*set_quality)(uint8_t q);
    void (*set_fps_limit)(int fps);
    void (*set_overlay)(bool on);
    bool (*overlay)(void);
    src_info_t *(*info)(void);
    src_stats_t *(*stats)(void);
    frame_ring_t *(*ring)(void);
    int (*supported_res)(char *out, size_t outlen);
    bool (*res_supported)(int w, int h);
} src_backend_t;

#if CONFIG_CAMTEST_ENABLE_DVP
/* source_dvp.c */
esp_err_t source_dvp_init(void);
esp_err_t source_dvp_start(void);
void source_dvp_stop(void);
esp_err_t source_dvp_apply(int w, int h, uint8_t quality, int fps_limit);
esp_err_t source_dvp_set_quality(uint8_t q);
void source_dvp_set_fps_limit(int fps);
void source_dvp_set_overlay(bool on);
bool source_dvp_overlay(void);
src_info_t *source_dvp_info(void);
src_stats_t *source_dvp_stats(void);
frame_ring_t *source_dvp_ring(void);
int source_dvp_supported_res(char *out, size_t outlen);
bool source_dvp_res_supported(int w, int h);
#endif

#if CONFIG_CAMTEST_ENABLE_USB
/* source_usb.c */
esp_err_t source_usb_init(void);
esp_err_t source_usb_start(void);
void source_usb_stop(void);
esp_err_t source_usb_apply(int w, int h, uint8_t quality, int fps_limit);
esp_err_t source_usb_set_quality(uint8_t q);
void source_usb_set_fps_limit(int fps);
void source_usb_set_overlay(bool on);
bool source_usb_overlay(void);
src_info_t *source_usb_info(void);
src_stats_t *source_usb_stats(void);
frame_ring_t *source_usb_ring(void);
int source_usb_supported_res(char *out, size_t outlen);
bool source_usb_res_supported(int w, int h);
usb_state_t source_usb_state(void);
usb_mode_t source_usb_mode(void);
esp_err_t source_usb_set_mode(usb_mode_t m);
uint32_t source_usb_disconnects(void);
const char *source_usb_device_name(void);
int source_usb_get_tiers(usb_tier_t *out, int max);
float source_usb_inherent_ms(void);
void source_usb_set_inherent_ms(float ms);
esp_err_t source_usb_h264_set_bitrate_kbps(uint32_t kbps);
uint32_t source_usb_h264_kbps(void);
#endif

static const src_backend_t s_backends[] = {
#if CONFIG_CAMTEST_ENABLE_DVP
    [VIDEO_SOURCE_DVP] = {
        .init = source_dvp_init,
        .start = source_dvp_start,
        .stop = source_dvp_stop,
        .apply = source_dvp_apply,
        .set_quality = source_dvp_set_quality,
        .set_fps_limit = source_dvp_set_fps_limit,
        .set_overlay = source_dvp_set_overlay,
        .overlay = source_dvp_overlay,
        .info = source_dvp_info,
        .stats = source_dvp_stats,
        .ring = source_dvp_ring,
        .supported_res = source_dvp_supported_res,
        .res_supported = source_dvp_res_supported,
    },
#else
    [VIDEO_SOURCE_DVP] = { 0 },   /* DVP 编译期未启用 */
#endif
#if CONFIG_CAMTEST_ENABLE_USB
    [VIDEO_SOURCE_USB] = {
        .init = source_usb_init,
        .start = source_usb_start,
        .stop = source_usb_stop,
        .apply = source_usb_apply,
        .set_quality = source_usb_set_quality,
        .set_fps_limit = source_usb_set_fps_limit,
        .set_overlay = source_usb_set_overlay,
        .overlay = source_usb_overlay,
        .info = source_usb_info,
        .stats = source_usb_stats,
        .ring = source_usb_ring,
        .supported_res = source_usb_supported_res,
        .res_supported = source_usb_res_supported,
    },
#endif
};
#define BACKEND_N (sizeof(s_backends) / sizeof(s_backends[0]))

static struct {
    video_source_t current;
    SemaphoreHandle_t lock;
    bool inited;
    frame_ring_t *ring;
} s_if;

static const src_backend_t *be(video_source_t s)
{
    if ((int)s < 0 || (int)s >= (int)BACKEND_N || !s_backends[s].init) return NULL;
    return &s_backends[s];
}

const char *src_if_source_name(video_source_t s)
{
    return s == VIDEO_SOURCE_USB ? "usb" : "dvp";
}

const char *src_if_usb_mode_name(usb_mode_t m)
{
    switch (m) {
    case USB_MODE_PASSTHROUGH: return "passthrough";
    case USB_MODE_REENCODE:    return "reencode";
    case USB_MODE_H264:        return "h264";
    default:                   return "n/a";
    }
}

const char *src_if_ts_meaning_name(ts_meaning_t m)
{
    return m == TS_MEANING_FRAME_ARRIVAL ? "frame_arrival" : "sensor_out";
}

esp_err_t src_if_init(void)
{
    if (s_if.inited) return ESP_OK;
    memset(&s_if, 0, sizeof(s_if));
    s_if.lock = xSemaphoreCreateMutex();
    if (!s_if.lock) return ESP_ERR_NO_MEM;

    /* 帧环终身持有：5 槽 × 1.5MB（720p MJPEG 直通帧最坏 ~700KB 也有余量；
     * Tab5 32MB PSRAM 无压力）。S31 的"init 一次永不销毁"铁律保留 */
    s_if.ring = frame_ring_create(5, 1536 * 1024);
    ESP_RETURN_ON_FALSE(s_if.ring, ESP_ERR_NO_MEM, TAG, "frame ring");

    s_if.current = VIDEO_SOURCE_USB;
#if CONFIG_CAMTEST_ENABLE_DVP
    /* DVP 允许失败（无相机/ISP 初始化失败时仍可运行 USB 源） */
    if (be(VIDEO_SOURCE_DVP)->init() != ESP_OK) {
        ESP_LOGW(TAG, "DVP 后端初始化失败（MIPI 相机不可用，USB 源不受影响）");
    }
#endif
#if CONFIG_CAMTEST_ENABLE_USB
    const src_backend_t *u = be(VIDEO_SOURCE_USB);
    ESP_RETURN_ON_ERROR(u->init(), TAG, "usb init");
#endif
    s_if.inited = true;
    ESP_LOGI(TAG, "采集源就绪：USB-UVC（默认）%s",
             CONFIG_CAMTEST_ENABLE_DVP ? "+ DVP（SC202CS）" : "");
    return ESP_OK;
}

video_source_t src_if_current(void) { return s_if.current; }

esp_err_t src_if_switch(video_source_t want)
{
    if (!s_if.inited) return ESP_ERR_INVALID_STATE;
    const src_backend_t *wb = be(want);
    if (!wb) return ESP_ERR_NOT_SUPPORTED;

    xSemaphoreTake(s_if.lock, portMAX_DELAY);
    video_source_t prev = s_if.current;
    if (want == prev) {
        /* 同源切换：仍走一遍 stop/start，作为"修复"手段（幂等） */
    } else {
        ESP_LOGW(TAG, "切换采集源 %s → %s", src_if_source_name(prev), src_if_source_name(want));
        be(prev)->stop();
    }
    esp_err_t err = wb->start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动 %s 失败（%s），回滚到 %s",
                 src_if_source_name(want), esp_err_to_name(err), src_if_source_name(prev));
        if (want != prev) {
            wb->stop();
            be(prev)->start();
        }
        xSemaphoreGive(s_if.lock);
        return err;
    }
    s_if.current = want;
    xSemaphoreGive(s_if.lock);
    return ESP_OK;
}

esp_err_t src_if_apply(int w, int h, uint8_t quality, int fps_limit)
{
    if (!s_if.inited) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_if.lock, portMAX_DELAY);
    esp_err_t err = be(s_if.current)->apply(w, h, quality, fps_limit);
    xSemaphoreGive(s_if.lock);
    return err;
}

esp_err_t src_if_set_quality(uint8_t q)
{
    if (!s_if.inited) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_if.lock, portMAX_DELAY);
    esp_err_t err = be(s_if.current)->set_quality(q);
    xSemaphoreGive(s_if.lock);
    return err;
}

void src_if_set_fps_limit(int fps)
{
    if (!s_if.inited) return;
    be(s_if.current)->set_fps_limit(fps);
}

void src_if_set_overlay(bool on)
{
    if (!s_if.inited) return;
    be(s_if.current)->set_overlay(on);
}

bool src_if_overlay(void)
{
    if (!s_if.inited) return false;
    return be(s_if.current)->overlay();
}

src_info_t *src_if_info(void)
{
    return be(s_if.current)->info();
}

src_stats_t *src_if_stats(void)
{
    return be(s_if.current)->stats();
}

frame_ring_t *src_if_ring(void)
{
    return s_if.ring;
}

int src_if_supported_res(char *out, size_t outlen)
{
    return be(s_if.current)->supported_res(out, outlen);
}

bool src_if_res_supported(int w, int h)
{
    return be(s_if.current)->res_supported(w, h);
}

/* ---------- USB 专用查询 ---------- */
#if CONFIG_CAMTEST_ENABLE_USB
usb_state_t src_if_usb_state(void) { return source_usb_state(); }
usb_mode_t src_if_usb_mode(void) { return source_usb_mode(); }
esp_err_t src_if_usb_set_mode(usb_mode_t m) { return source_usb_set_mode(m); }
uint32_t src_if_usb_disconnects(void) { return source_usb_disconnects(); }
const char *src_if_usb_device_name(void) { return source_usb_device_name(); }
int src_if_usb_get_tiers(usb_tier_t *out, int max) { return source_usb_get_tiers(out, max); }
float src_if_usb_inherent_ms(void) { return source_usb_inherent_ms(); }
void src_if_usb_set_inherent_ms(float ms) { source_usb_set_inherent_ms(ms); }
esp_err_t src_if_h264_set_bitrate_kbps(uint32_t kbps) { return source_usb_h264_set_bitrate_kbps(kbps); }
uint32_t src_if_h264_kbps(void) { return source_usb_h264_kbps(); }
#else
usb_state_t src_if_usb_state(void) { return USB_STATE_DISABLED; }
usb_mode_t src_if_usb_mode(void) { return USB_MODE_NONE; }
esp_err_t src_if_usb_set_mode(usb_mode_t m) { return ESP_ERR_NOT_SUPPORTED; }
uint32_t src_if_usb_disconnects(void) { return 0; }
const char *src_if_usb_device_name(void) { return ""; }
int src_if_usb_get_tiers(usb_tier_t *out, int max) { return 0; }
float src_if_usb_inherent_ms(void) { return -1.0f; }
void src_if_usb_set_inherent_ms(float ms) { }
esp_err_t src_if_h264_set_bitrate_kbps(uint32_t kbps) { return ESP_ERR_NOT_SUPPORTED; }
uint32_t src_if_h264_kbps(void) { return 0; }
#endif
