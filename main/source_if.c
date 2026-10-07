/*
 * source_if.c — 统一采集源调度（U1）：vtable 分发 + 热切换（带回滚）
 *
 * 切换协议（不死机、可回滚）：
 *   1. 停当前源（任务停靠到安全点，流关闭）
 *   2. 启动目标源；任一步失败 → 尝试重启原源并返回错误
 *   帧环全程不销毁，流客户端跨切换持续收帧（fid 连续递增）。
 */
#include "source_if.h"
#include <string.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "src_if";

/* ---------- 后端 vtable（source_dvp.c / source_usb.c 实现） ---------- */
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
#endif

static const src_backend_t s_backends[] = {
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
    SemaphoreHandle_t lock;      /* 串行化 switch/apply（httpd 与 scan 可能并发） */
    bool inited;
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

    /* DVP 必须成功（原通路）；USB 允许失败（无设备/供电不足时仍可运行 DVP） */
    const src_backend_t *d = be(VIDEO_SOURCE_DVP);
    esp_err_t err = d->init();
    ESP_RETURN_ON_ERROR(err, TAG, "dvp init");
    s_if.current = VIDEO_SOURCE_DVP;
    s_if.inited = true;

#if CONFIG_CAMTEST_ENABLE_USB
    if (be(VIDEO_SOURCE_USB)->init() != ESP_OK) {
        ESP_LOGE(TAG, "USB 后端初始化失败（USB 源不可用，DVP 不受影响）");
    }
#endif
    ESP_LOGI(TAG, "采集源就绪：DVP（默认）%s", CONFIG_CAMTEST_ENABLE_USB ? "+ USB" : "（USB 未启用）");
    return ESP_OK;
}

video_source_t src_if_current(void) { return s_if.current; }

esp_err_t src_if_switch(video_source_t want)
{
    if (!s_if.inited) return ESP_ERR_INVALID_STATE;
    const src_backend_t *wb = be(want);
    if (!wb) return ESP_ERR_NOT_SUPPORTED;      /* USB 编译期未启用 */

    xSemaphoreTake(s_if.lock, portMAX_DELAY);
    video_source_t prev = s_if.current;
    if (want == prev) {
        /* 同源切换：仍走一遍 stop/start，作为"修复"手段（幂等） */
    } else {
        ESP_LOGW(TAG, "切换采集源 %s → %s", src_if_source_name(prev), src_if_source_name(want));
        be(prev)->stop();                        /* 先停旧源（安全停靠） */
    }
    esp_err_t err = wb->start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动 %s 失败（%s），回滚到 %s",
                 src_if_source_name(want), esp_err_to_name(err), src_if_source_name(prev));
        if (want != prev) {
            be(prev)->start();                   /* 尽力回滚，不死机 */
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
    return be(s_if.current)->ring();
}

int src_if_supported_res(char *out, size_t outlen)
{
    return be(s_if.current)->supported_res(out, outlen);
}

bool src_if_res_supported(int w, int h)
{
    return be(s_if.current)->res_supported(w, h);
}

/* ---------- USB 专用查询（转发到 source_usb 后端；编译关闭时给安全默认值） ---------- */
#if CONFIG_CAMTEST_ENABLE_USB
usb_state_t src_if_usb_state(void) { return source_usb_state(); }
usb_mode_t src_if_usb_mode(void) { return source_usb_mode(); }
esp_err_t src_if_usb_set_mode(usb_mode_t m) { return source_usb_set_mode(m); }
uint32_t src_if_usb_disconnects(void) { return source_usb_disconnects(); }
const char *src_if_usb_device_name(void) { return source_usb_device_name(); }
int src_if_usb_get_tiers(usb_tier_t *out, int max) { return source_usb_get_tiers(out, max); }
float src_if_usb_inherent_ms(void) { return source_usb_inherent_ms(); }
void src_if_usb_set_inherent_ms(float ms) { source_usb_set_inherent_ms(ms); }
#else
usb_state_t src_if_usb_state(void) { return USB_STATE_DISABLED; }
usb_mode_t src_if_usb_mode(void) { return USB_MODE_NONE; }
esp_err_t src_if_usb_set_mode(usb_mode_t m) { return ESP_ERR_NOT_SUPPORTED; }
uint32_t src_if_usb_disconnects(void) { return 0; }
const char *src_if_usb_device_name(void) { return ""; }
int src_if_usb_get_tiers(usb_tier_t *out, int max) { return 0; }
float src_if_usb_inherent_ms(void) { return -1.0f; }
void src_if_usb_set_inherent_ms(float ms) { }
#endif
