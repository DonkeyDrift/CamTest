/*
 * app_config.c — NVS 持久化配置实现（见 app_config.h 注释）
 *
 * apply 逻辑自 http_server.c h_config 原样迁移（步骤/顺序/错误文案不变），
 * 使 POST /api/config 与启动恢复共用同一实现。
 */
#include "app_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "camera_pipeline.h"
#include "source_if.h"
#include "metrics.h"

static const char *TAG = "app_cfg";

static const char *NVS_NS = "camtest";
static const char *NVS_KEY = "cfg";

static app_config_t s_cfg;

static void cfg_defaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.magic = APP_CFG_MAGIC;
    s_cfg.version = APP_CFG_VERSION;
    s_cfg.source = VIDEO_SOURCE_DVP;
    s_cfg.inherent_ms = -1.0f;   /* 未标定 */
}

/* magic/version 校验 + 各字段消毒：坏位直接剥掉（退回源默认） */
static void cfg_sanitize(app_config_t *c)
{
    if (c->source > VIDEO_SOURCE_USB) { c->valid &= ~APP_CFG_SRC; c->source = VIDEO_SOURCE_DVP; }
    if (c->usb_mode != USB_MODE_PASSTHROUGH && c->usb_mode != USB_MODE_REENCODE)
        c->valid &= ~APP_CFG_USB_MODE;
    if (c->quality == 0 || c->quality > 100) c->valid &= ~APP_CFG_QUALITY;
    if (c->w == 0 || c->h == 0 || c->w > 4096 || c->h > 4096) c->valid &= ~APP_CFG_RES;
    if (c->fps_limit <= 0) c->valid &= ~APP_CFG_FPS;
    if (c->hifps < 0 || c->hifps > 4) c->valid &= ~APP_CFG_HIFPS;
    if (!isfinite(c->target_mbps) || !isfinite(c->inherent_ms)) {
        c->valid &= ~(APP_CFG_MBPS | APP_CFG_INHERENT);
        c->target_mbps = 0;
        c->inherent_ms = -1.0f;
    }
}

void app_config_init(void)
{
    cfg_defaults();
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (e != ESP_OK) {
        ESP_LOGI(TAG, "无持久化配置（首次启动），默认 source=dvp");
        return;
    }
    app_config_t c;
    size_t len = sizeof(c);
    e = nvs_get_blob(h, NVS_KEY, &c, &len);
    nvs_close(h);
    if (e != ESP_OK || len != sizeof(c)) {
        ESP_LOGW(TAG, "配置读取失败（%s len=%u），用默认",
                 e == ESP_OK ? "size mismatch" : esp_err_to_name(e), (unsigned)len);
        return;
    }
    if (c.magic != APP_CFG_MAGIC || c.version != APP_CFG_VERSION) {
        ESP_LOGW(TAG, "配置版本不符（magic=%08x ver=%u），用默认", c.magic, c.version);
        return;
    }
    cfg_sanitize(&c);
    c.magic = APP_CFG_MAGIC;
    c.version = APP_CFG_VERSION;
    s_cfg = c;
    ESP_LOGI(TAG, "已载入持久化配置：valid=0x%lx source=%s mode=%s res=%ux%u q=%u fps=%d "
                  "ov=%d mbps=%.1f inherent=%.1f hifps=%d",
             (unsigned long)s_cfg.valid,
             src_if_source_name(s_cfg.valid & APP_CFG_SRC ? s_cfg.source : VIDEO_SOURCE_DVP),
             s_cfg.valid & APP_CFG_USB_MODE ? src_if_usb_mode_name(s_cfg.usb_mode) : "-",
             s_cfg.w, s_cfg.h, s_cfg.quality, s_cfg.fps_limit,
             s_cfg.overlay, s_cfg.target_mbps, s_cfg.inherent_ms, s_cfg.hifps);
}

const app_config_t *app_config_get(void) { return &s_cfg; }

static void cfg_store(const app_config_t *c)
{
    s_cfg = *c;
    s_cfg.magic = APP_CFG_MAGIC;
    s_cfg.version = APP_CFG_VERSION;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open 失败，配置未落盘");
        return;
    }
    esp_err_t e = nvs_set_blob(h, NVS_KEY, &s_cfg, sizeof(s_cfg));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "配置落盘失败：%s", esp_err_to_name(e));
    } else {
        ESP_LOGI(TAG, "配置已落盘：valid=0x%lx source=%s inherent=%.1f",
                 (unsigned long)s_cfg.valid,
                 src_if_source_name(s_cfg.source), s_cfg.inherent_ms);
    }
}

void app_config_persist(const cJSON *j)
{
    if (!j) return;
    app_config_t c = s_cfg;
    const cJSON *f;

    if ((f = cJSON_GetObjectItem(j, "source")) && cJSON_IsString(f)) {
        c.valid |= APP_CFG_SRC;
        c.source = strcmp(f->valuestring, "usb") == 0 ? VIDEO_SOURCE_USB : VIDEO_SOURCE_DVP;
    }
    if ((f = cJSON_GetObjectItem(j, "usb_mode")) && cJSON_IsString(f)) {
        if (strcmp(f->valuestring, "reencode") == 0) {
            c.valid |= APP_CFG_USB_MODE; c.usb_mode = USB_MODE_REENCODE;
        } else if (strcmp(f->valuestring, "passthrough") == 0) {
            c.valid |= APP_CFG_USB_MODE; c.usb_mode = USB_MODE_PASSTHROUGH;
        }
    }
    if ((f = cJSON_GetObjectItem(j, "usb_inherent_ms")) && cJSON_IsNumber(f)) {
        c.valid |= APP_CFG_INHERENT;
        c.inherent_ms = f->valuedouble >= 0 ? (float)f->valuedouble : -1.0f;
    }
    if ((f = cJSON_GetObjectItem(j, "overlay")) && cJSON_IsBool(f)) {
        c.valid |= APP_CFG_OVERLAY;
        c.overlay = cJSON_IsTrue(f) ? 1 : 0;
    }
    if ((f = cJSON_GetObjectItem(j, "target_mbps")) && cJSON_IsNumber(f)) {
        c.valid |= APP_CFG_MBPS;
        c.target_mbps = (float)f->valuedouble;
    }
    if ((f = cJSON_GetObjectItem(j, "res")) && cJSON_IsString(f) && strchr(f->valuestring, 'x')) {
        int w = 0, h = 0;
        if (sscanf(f->valuestring, "%dx%d", &w, &h) == 2 && w > 0 && h > 0) {
            c.valid |= APP_CFG_RES;
            c.w = (uint16_t)w;
            c.h = (uint16_t)h;
        }
    }
    if ((f = cJSON_GetObjectItem(j, "quality")) && cJSON_IsNumber(f) && f->valueint > 0) {
        c.valid |= APP_CFG_QUALITY;
        c.quality = (uint8_t)f->valueint;
    }
    if ((f = cJSON_GetObjectItem(j, "fps_limit")) && cJSON_IsNumber(f) && f->valueint > 0) {
        c.valid |= APP_CFG_FPS;
        c.fps_limit = (int16_t)f->valueint;
    }
    /* hifps 仅 DVP 生效（与 apply 同条件：切换后仍需当前源是 DVP） */
    if ((f = cJSON_GetObjectItem(j, "hifps")) && cJSON_IsNumber(f) &&
        src_if_current() == VIDEO_SOURCE_DVP) {
        c.valid |= APP_CFG_HIFPS;
        c.hifps = (int16_t)f->valueint;
    }
    cfg_store(&c);
}

cJSON *app_config_boot_json(void)
{
    const app_config_t *c = &s_cfg;
    cJSON *j = cJSON_CreateObject();
    if (!j) return NULL;
    /* source 恒发：无记录时 = dvp，等价旧固件的 src_if_switch(VIDEO_SOURCE_DVP) */
    video_source_t s = (c->valid & APP_CFG_SRC) && c->source == VIDEO_SOURCE_USB
                       ? VIDEO_SOURCE_USB : VIDEO_SOURCE_DVP;
    cJSON_AddStringToObject(j, "source", src_if_source_name(s));
    if ((c->valid & APP_CFG_USB_MODE) &&
        (c->usb_mode == USB_MODE_PASSTHROUGH || c->usb_mode == USB_MODE_REENCODE))
        cJSON_AddStringToObject(j, "usb_mode", src_if_usb_mode_name((usb_mode_t)c->usb_mode));
    if (c->valid & APP_CFG_INHERENT)
        cJSON_AddNumberToObject(j, "usb_inherent_ms", c->inherent_ms);
    if (c->valid & APP_CFG_OVERLAY)
        cJSON_AddBoolToObject(j, "overlay", c->overlay != 0);
    if (c->valid & APP_CFG_MBPS)
        cJSON_AddNumberToObject(j, "target_mbps", c->target_mbps);
    if ((c->valid & APP_CFG_RES) && c->w > 0 && c->h > 0) {
        char rs[16];
        snprintf(rs, sizeof(rs), "%ux%u", c->w, c->h);
        cJSON_AddStringToObject(j, "res", rs);
    }
    if ((c->valid & APP_CFG_QUALITY) && c->quality > 0)
        cJSON_AddNumberToObject(j, "quality", c->quality);
    if ((c->valid & APP_CFG_FPS) && c->fps_limit > 0)
        cJSON_AddNumberToObject(j, "fps_limit", c->fps_limit);
    if (c->valid & APP_CFG_HIFPS)
        cJSON_AddNumberToObject(j, "hifps", c->hifps);
    return j;
}

bool app_config_apply(const cJSON *j, app_cfg_result_t *r)
{
    memset(r, 0, sizeof(*r));

    const cJSON *src = cJSON_GetObjectItem(j, "source");
    const cJSON *res = cJSON_GetObjectItem(j, "res");
    const cJSON *q = cJSON_GetObjectItem(j, "quality");
    const cJSON *fps = cJSON_GetObjectItem(j, "fps_limit");
    const cJSON *ov = cJSON_GetObjectItem(j, "overlay");
    const cJSON *br = cJSON_GetObjectItem(j, "target_mbps");
    const cJSON *um = cJSON_GetObjectItem(j, "usb_mode");
    const cJSON *inh = cJSON_GetObjectItem(j, "usb_inherent_ms");
    const cJSON *vts = cJSON_GetObjectItem(j, "vts");     /* 兼容：仅 VTS（诊断用，不持久化） */
    const cJSON *bst = cJSON_GetObjectItem(j, "boost");   /* OV3660 高帧率窗口裁剪（不持久化） */
    const cJSON *hfp = cJSON_GetObjectItem(j, "hifps");   /* 0-4 档位（推荐入口） */

    char err_msg[64] = "";
    bool ok = true;

    /* 1) 采集源热切换（先于其他参数；失败立即回滚并返回错误） */
    if (src && cJSON_IsString(src)) {
        video_source_t want = strcmp(src->valuestring, "usb") == 0 ? VIDEO_SOURCE_USB : VIDEO_SOURCE_DVP;
        esp_err_t serr = src_if_switch(want);
        if (serr != ESP_OK) {
            snprintf(err_msg, sizeof(err_msg), "source switch to %s failed: %s",
                     src->valuestring, esp_err_to_name(serr));
            ok = false;
        }
    }
    /* 2) USB 模式切换（passthrough / reencode） */
    if (ok && um && cJSON_IsString(um)) {
        usb_mode_t m = strcmp(um->valuestring, "reencode") == 0 ? USB_MODE_REENCODE :
                       strcmp(um->valuestring, "passthrough") == 0 ? USB_MODE_PASSTHROUGH : USB_MODE_NONE;
        if (m != USB_MODE_NONE && src_if_usb_set_mode(m) != ESP_OK) {
            snprintf(err_msg, sizeof(err_msg), "usb mode switch to %s failed", um->valuestring);
            ok = false;
        }
    }
    /* 3) 摄像头内部固有延迟标定值（U4：只能由光学闭环人工标定，设备绝不自行生成）；
     *    负值 = 清除标定（回到未标定态） */
    if (ok && inh && cJSON_IsNumber(inh)) {
        src_if_usb_set_inherent_ms(inh->valuedouble >= 0 ? (float)inh->valuedouble : -1.0f);
    }

    if (ov && cJSON_IsBool(ov)) {
        bool ov_on = cJSON_IsTrue(ov);
        src_if_set_overlay(ov_on);
        /* passthrough 不解码、无法烧录毫秒计数器：开启叠加时自动切重编码（set_mode 同步重开并带回滚），
         * /overlay 校验页只发 {overlay:true}，靠这一步保证 USB 直通下计数器也能真正出现 */
        if (ov_on && src_if_current() == VIDEO_SOURCE_USB &&
            src_if_usb_mode() == USB_MODE_PASSTHROUGH) {
            esp_err_t merr = src_if_usb_set_mode(USB_MODE_REENCODE);
            if (merr != ESP_OK)
                ESP_LOGW(TAG, "overlay 需要重编码，但 USB 模式切换失败：%s", esp_err_to_name(merr));
        }
    }
    if (br && cJSON_IsNumber(br)) metrics_set_target_mbps(br->valuedouble);

    bool need_rebuild = false;
    int w = 0, h = 0;
    uint8_t quality = 0;
    int fps_limit = 0;
    int vts_val = (vts && cJSON_IsNumber(vts)) ? vts->valueint : -1;
    if (res && cJSON_IsString(res) && strchr(res->valuestring, 'x')) {
        sscanf(res->valuestring, "%dx%d", &w, &h);
        need_rebuild = src_if_res_supported(w, h);
        if (!need_rebuild) {
            ok = false;
            snprintf(err_msg, sizeof(err_msg), "resolution %s not supported", res->valuestring);
        }
    }
    if (q && cJSON_IsNumber(q) && q->valueint > 0) { quality = q->valueint; need_rebuild = need_rebuild || quality != src_if_info()->quality; }
    if (fps && cJSON_IsNumber(fps) && fps->valueint > 0) fps_limit = fps->valueint;

    esp_err_t err = ESP_OK;
    bool is_dvp = src_if_current() == VIDEO_SOURCE_DVP;
    if (ok && is_dvp && hfp && cJSON_IsNumber(hfp)) {
        cam_boost_apply_level(hfp->valueint);
        if (!w) { w = 160; h = 120; }   /* 高帧率档默认目标 160x120 */
        need_rebuild = true;
    }
    if (!ok) {
        /* 源/模式切换失败：不再动参数 */
    } else if (is_dvp && bst && cJSON_IsObject(bst)) {
        cam_boost_params_t bp = {0};
        const cJSON *f;
        if ((f = cJSON_GetObjectItem(bst, "vts")) && cJSON_IsNumber(f)) bp.vts = f->valueint;
        if ((f = cJSON_GetObjectItem(bst, "hts")) && cJSON_IsNumber(f)) bp.hts = f->valueint;
        if ((f = cJSON_GetObjectItem(bst, "vstart")) && cJSON_IsNumber(f)) bp.vstart = f->valueint;
        if ((f = cJSON_GetObjectItem(bst, "vend")) && cJSON_IsNumber(f)) bp.vend = f->valueint;
        if ((f = cJSON_GetObjectItem(bst, "hstart")) && cJSON_IsNumber(f)) bp.hstart = f->valueint;
        if ((f = cJSON_GetObjectItem(bst, "hend")) && cJSON_IsNumber(f)) bp.hend = f->valueint;
        if ((f = cJSON_GetObjectItem(bst, "c303b")) && cJSON_IsNumber(f)) cam_boost_clk_set(f->valueint, -1, -1);
        if ((f = cJSON_GetObjectItem(bst, "c303d")) && cJSON_IsNumber(f)) cam_boost_clk_set(-1, f->valueint, -1);
        if ((f = cJSON_GetObjectItem(bst, "c3824")) && cJSON_IsNumber(f)) cam_boost_clk_set(-1, -1, f->valueint);
        if (!w) { w = 240; h = 240; }   /* boost 基于母本档，未指定 res 时默认 240x240 */
        err = cam_pipe_apply_boost(w, h, quality, fps_limit, bp.vts ? &bp : NULL);
    } else if ((need_rebuild || vts_val >= 0) && is_dvp) {
        err = src_if_apply(w, h, quality, fps_limit);
    } else if (need_rebuild) {
        err = src_if_apply(w, h, quality, fps_limit);   /* USB 源：重协商流 */
    } else if (fps_limit) {
        src_if_set_fps_limit(fps_limit);   /* 轻量：不重建 */
    } else if (quality) {
        err = src_if_set_quality(quality); /* 轻量：只改质量 */
    }

    /* 错误文案与旧 /api/config 应答逐字一致 */
    if (err_msg[0]) {
        snprintf(r->error, sizeof(r->error), "%s", err_msg);
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        snprintf(r->error, sizeof(r->error), "resolution not supported by current source");
    } else if (err != ESP_OK) {
        snprintf(r->error, sizeof(r->error), "%s", esp_err_to_name(err));
    }
    r->ok = ok && err == ESP_OK;
    r->restarted = need_rebuild;
    return r->ok;
}
