/*
 * lcd_ui.c — Tab5 LCD 实时预览 + 运行参数面板 + 触屏调参
 * （竖屏 720x1280 MIPI-DSI，三代面板运行时探测；移植自 S31 lcd_ui c0c104c）
 *
 * 与 S31 版的架构等价（低开销纪律全部保留）：
 *  - 预览 = frame_ring 订阅者（只取最新帧、超速丢弃不排队）；
 *  - 解码经 jpeg_dec_share（P4 解码器单例，与 source_usb 互斥共享）；
 *  - 解码 RGB565 等比放大填满预览区后直写 DSI 后备帧缓冲，
 *    draw_bitmap(fb 指针) 触发驱动帧边界翻页（无撕裂，见 esp_lcd_panel_dpi.c）；
 *  - LVGL 只画状态/触控面板，局部刷新经 draw_shim 镜像同步两页 fb；
 *  - LVGL 渲染任务钉 core0（采集/编码/worker 都在 core1）；
 *  - 触屏动作 → 队列 → apply_task 走 app_config_apply（与网页同路径）。
 *
 * 平台差异（相对 S31）：
 *  - 显示初始化自 esp-bsp m5stack_tab5 1.3.2 的 bsp_display.c 提取（三代屏
 *    探测 + 三套 DSI 时序/init 表 + ILI9881C/ST7123 面板驱动 + GT911/ST7123
 *    触摸），I2C 总线用 board_power 的（BSP 整包会与我们抢 I2C 端口）。
 *  - Tab5 无 hifps（OV3660 专用），控制面板换成 H264 码率（Kbps）行。
 */
#include "lcd_ui.h"
#include "sdkconfig.h"

#if CONFIG_CAMTEST_ENABLE_LCD

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_ldo_regulator.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"   /* esp_lcd_panel_t vtable（flush shim） */
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_ili9881c.h"
#include "esp_lcd_st7123.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_touch_st7123.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "cJSON.h"
#include "app_config.h"
#include "board_power.h"
#include "frame_ring.h"
#include "jpeg_dec_share.h"
#include "metrics.h"
#include "source_if.h"
#include "wifi_net.h"

/* 面板 init 序列表（拷自 esp-bsp m5stack_tab5 1.3.2 priv_include） */
#include "disp_init_data.h"       /* ILI9881C（板版1） */
#include "disp_init_data_1.h"     /* ST7123（板版2） */
#include "disp_init_data_st7121.h"/* ST7121（板版3） */

static const char *TAG = "lcd_ui";

/* ---- 硬件常量（esp-bsp m5stack_tab5 1.3.2）---- */
#define LCD_H_RES            720    /* 面板原生竖屏：720 列 x 1280 行 */
#define LCD_V_RES            1280
#define LCD_MIPI_LANE_NUM    2
#define LCD_LANE_MBPS        1000   /* ILI9881C */
#define LCD_LANE_MBPS_ST     965    /* ST7121/ST7123 */
#define LCD_DPHY_LDO_CHAN    3      /* LDO_VO3 → VDD_MIPI_DPHY */
#define LCD_DPHY_LDO_MV      2500
#define LCD_BACKLIGHT_GPIO   GPIO_NUM_22
#define LCD_TOUCH_INT_GPIO   GPIO_NUM_23
#define LCD_LED_C_TIMER      LEDC_TIMER_0
#define LCD_LED_C_CH         LEDC_CHANNEL_0

/* ---- 布局（竖屏 720x1280）---- */
#define PREV_W               720
#define PREV_H               540
#define STAT_Y0              552
#define STAT_STEP            22
#define CTL_Y0               782
#define CTL_STEP             50

typedef enum {
    BOARD_V_UNKNOWN = 0,
    BOARD_V_ILI9881C_GT911,
    BOARD_V_ST7123,
    BOARD_V_ST7121,
} board_version_t;

/* ---------------- 全局状态 ---------------- */
static volatile bool s_active;
static volatile bool s_preview_on;      /* 预览任务存在（fb/分配成功） */
static volatile bool s_pv_on = true;    /* 用户预览开关（屏上按钮） */
static board_version_t s_board_ver = BOARD_V_UNKNOWN;

/* DSI 双帧缓冲（驱动分配，PSRAM 黑底）：预览写后备页，draw_bitmap(fb 指针)
 * 让驱动在帧完成边界翻页——消除"中间斜线"撕裂（写速率≈扫描速率所致） */
static uint16_t *s_fb[2];
static size_t s_fb_px;           /* 每页像素数 = 720*1280 */
static int s_cur;                /* 正在扫描输出的页序号（自跟踪：仅预览翻页会改） */

static volatile float s_prev_fps;
static volatile float s_dec_ms;
static volatile uint32_t s_last_frame_us;

static char s_status[112];
static uint32_t s_seq;
static portMUX_TYPE s_st_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct { char json[176]; } cfg_msg_t;
static QueueHandle_t s_cfgq;
static SemaphoreHandle_t s_pv_notify;   /* 帧环发布 + 按钮唤醒共用 */

static lv_obj_t *s_img, *s_nosig;
static lv_obj_t *l_fps, *l_rate, *l_cpu, *l_heap, *l_psram;
static lv_obj_t *l_src, *l_wifi, *l_ip, *l_lcd, *l_e2e;
static lv_obj_t *b_dvp, *b_usb, *b_pt, *b_h264, *b_re, *b_ov, *b_ov_lab, *b_pv, *b_pv_lab;
static lv_obj_t *v_res, *v_kb, *v_q, *v_fps, *v_mb;
static lv_obj_t *l_status;
static bool s_nosig_shown = true;
static int s_prev_src = -1, s_prev_um = -1, s_prev_ov = -1, s_prev_pv = -1, s_st_col = -1;

/* 预览解码输出缓冲（MCU 32 对齐） */
static uint8_t *s_out;
static size_t s_out_cap;

/* flush 观测（esp_lvgl_port 忽略 draw_bitmap 返回值） */
static esp_lcd_panel_handle_t s_panel;
static esp_err_t (*s_real_draw)(esp_lcd_panel_t *, int, int, int, int, const void *);
static volatile uint32_t s_flush_cnt, s_flush_err;

static inline bool ptr_in_fb(const void *p)
{
    for (int i = 0; i < 2; i++)
        if (s_fb[i] && (const uint16_t *)p >= s_fb[i] &&
            (const uint16_t *)p < s_fb[i] + s_fb_px) return true;
    return false;
}

/* LVGL 局部刷新经驱动 CPU 拷贝只落在当前显示页——把同一区域镜像到另一页，
 * 否则翻页后 UI 改动"消失"一帧。x1/y1 排他（与 DPI 驱动坐标语义一致） */
static void mirror_region(int x0, int y0, int x1, int y1)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > LCD_H_RES) x1 = LCD_H_RES;
    if (y1 > LCD_V_RES) y1 = LCD_V_RES;
    if (x1 <= x0 || y1 <= y0) return;
    uint16_t *from = s_fb[s_cur], *to = s_fb[s_cur ^ 1];
    for (int y = y0; y < y1; y++)
        memcpy(to + (size_t)y * LCD_H_RES + x0,
               from + (size_t)y * LCD_H_RES + x0, (size_t)(x1 - x0) * 2);
    esp_cache_msync(to + (size_t)y0 * LCD_H_RES + x0, (size_t)(y1 - y0) * LCD_H_RES * 2,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

static esp_err_t draw_shim(esp_lcd_panel_t *p, int x0, int y0, int x1, int y1, const void *d)
{
    s_flush_cnt++;
    esp_err_t e = s_real_draw(p, x0, y0, x1, y1, d);
    if (e == ESP_OK) {
        if (s_fb[0] && !ptr_in_fb(d)) mirror_region(x0, y0, x1, y1);
    } else if (s_flush_err++ < 3) {
        ESP_LOGE(TAG, "draw_bitmap 失败 %s (%d..%d, %d..%d)", esp_err_to_name(e), x0, x1, y0, y1);
    }
    return e;
}

/* ---------------- 状态行（seqlock） ---------------- */
static void status_set(const char *fmt, ...)
{
    char tmp[112];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    portENTER_CRITICAL(&s_st_mux);
    s_seq |= 1u;
    strlcpy(s_status, tmp, sizeof(s_status));
    s_seq++;
    portEXIT_CRITICAL(&s_st_mux);
}

static void status_copy(char *out, size_t n)
{
    for (int i = 0; i < 8; i++) {
        uint32_t a = s_seq;
        if (a & 1) continue;
        strlcpy(out, s_status, n);
        uint32_t b = s_seq;
        if (a == b && !(b & 1)) return;
    }
    strlcpy(out, s_status, n);
}

static void setl(lv_obj_t *lb, const char *txt)
{
    if (strcmp(lv_label_get_text(lb), txt) != 0) lv_label_set_text(lb, txt);
}

static const char *usb_state_str(void)
{
    switch (src_if_usb_state()) {
    case USB_STATE_DISABLED:     return "off";
    case USB_STATE_NO_DEVICE:    return "no-dev";
    case USB_STATE_DEVICE_READY: return "ready";
    case USB_STATE_STREAMING:    return "live";
    case USB_STATE_ERROR:        return "err";
    }
    return "?";
}

/* ---------------- 触屏动作 ---------------- */
typedef enum {
    ACT_SRC_DVP, ACT_SRC_USB,
    ACT_UM_PT, ACT_UM_H264, ACT_UM_RE,
    ACT_RES_DEC, ACT_RES_INC,
    ACT_KB_DEC, ACT_KB_INC,
    ACT_Q_DEC, ACT_Q_INC,
    ACT_FPS_DEC, ACT_FPS_INC,
    ACT_OV_TOGGLE,
    ACT_MB_DEC, ACT_MB_INC,
    ACT_PV_TOGGLE,
} act_t;

static void send_json(const char *json)
{
    cfg_msg_t m;
    strlcpy(m.json, json, sizeof(m.json));
    if (xQueueSend(s_cfgq, &m, 0) != pdTRUE) status_set("busy, try again");
    else status_set("queued: %s", json);
}

/* 触发链路重建的动作带上当前 fps_limit（apply 对缺省字段传 0=清限帧） */
static void pair_fps(char *json, size_t cap)
{
    int f = src_if_info()->fps_limit;
    if (f > 0) {
        size_t n = strlen(json);
        snprintf(json + n, cap - n - 2, ",\"fps_limit\":%d", f);
    }
    strcat(json, "}");
}

static void act_event(lv_event_t *e)
{
    if (!s_active) return;
    act_t a = (act_t)(uintptr_t)lv_event_get_user_data(e);
    src_info_t *ci = src_if_info();
    bool usb = ci->source == VIDEO_SOURCE_USB;
    char json[176];

    switch (a) {
    case ACT_SRC_DVP:
        if (!usb) return;
        snprintf(json, sizeof(json), "{\"source\":\"dvp\"}");
        break;
    case ACT_SRC_USB:
        if (usb) return;
        snprintf(json, sizeof(json), "{\"source\":\"usb\"}");
        break;
    case ACT_UM_PT:
        if (!usb || src_if_usb_mode() == USB_MODE_PASSTHROUGH) return;
        snprintf(json, sizeof(json), "{\"usb_mode\":\"passthrough\"}");
        break;
    case ACT_UM_H264:
        if (!usb || src_if_usb_mode() == USB_MODE_H264) return;
        snprintf(json, sizeof(json), "{\"usb_mode\":\"h264\"}");
        break;
    case ACT_UM_RE:
        if (!usb || src_if_usb_mode() == USB_MODE_REENCODE) return;
        snprintf(json, sizeof(json), "{\"usb_mode\":\"reencode\"}");
        break;
    case ACT_RES_DEC:
    case ACT_RES_INC: {
        char list[160];
        src_if_supported_res(list, sizeof(list));
        char *tok[12], *save = NULL;
        int n = 0;
        for (char *p = strtok_r(list, ",", &save); p && n < 12; p = strtok_r(NULL, ",", &save))
            tok[n++] = p;
        if (!n) return;
        char cur[16];
        snprintf(cur, sizeof(cur), "%ux%u", ci->w, ci->h);
        int idx = 0;
        for (int i = 0; i < n; i++) if (strcmp(tok[i], cur) == 0) { idx = i; break; }
        idx = (a == ACT_RES_INC) ? (idx + 1) % n : (idx - 1 + n) % n;
        snprintf(json, sizeof(json), "{\"res\":\"%s\"", tok[idx]);
        pair_fps(json, sizeof(json));
        break;
    }
    case ACT_KB_DEC:
    case ACT_KB_INC: {
        static const int kl[] = { 500, 1000, 1500, 2500, 4000, 6000 };
        int cur = (int)src_if_h264_kbps(), n = -1;
        if (a == ACT_KB_INC) {
            for (int i = 0; i < 6; i++) if (kl[i] > cur) { n = kl[i]; break; }
            if (n < 0) n = kl[0];
        } else {
            for (int i = 5; i >= 0; i--) if (kl[i] < cur) { n = kl[i]; break; }
            if (n < 0) n = kl[5];
        }
        snprintf(json, sizeof(json), "{\"h264_kbps\":%d}", n);
        break;
    }
    case ACT_Q_DEC:
    case ACT_Q_INC: {
        if (usb && src_if_usb_mode() != USB_MODE_REENCODE) {
            status_set("quality: reencode mode only");
            return;
        }
        int q = ci->quality;
        if (q <= 0) q = 40;
        int n;
        if (a == ACT_Q_INC) { n = (q / 10 + 1) * 10; if (n > 90) n = 10; }
        else                { n = ((q + 9) / 10 - 1) * 10; if (n < 10) n = 90; }
        snprintf(json, sizeof(json), "{\"quality\":%d", n);
        pair_fps(json, sizeof(json));
        break;
    }
    case ACT_FPS_DEC:
    case ACT_FPS_INC: {
        static const int fl[] = { 15, 30, 60, 120 };
        int cur = ci->fps_limit, n = -1;
        if (a == ACT_FPS_INC) {
            for (int i = 0; i < 4; i++) if (fl[i] > cur) { n = fl[i]; break; }
            if (n < 0) n = fl[0];
        } else {
            for (int i = 3; i >= 0; i--) if (fl[i] < cur) { n = fl[i]; break; }
            if (n < 0) n = fl[3];
        }
        snprintf(json, sizeof(json), "{\"fps_limit\":%d}", n);
        break;
    }
    case ACT_OV_TOGGLE:
        snprintf(json, sizeof(json), "{\"overlay\":%s}",
                 src_if_overlay() ? "false" : "true");
        break;
    case ACT_MB_DEC:
    case ACT_MB_INC: {
        static const float ml[] = { 0, 2, 4, 6, 8 };
        float cur = metrics_get()->target_mbps;
        int n = -1;
        if (a == ACT_MB_INC) {
            for (int i = 0; i < 5; i++) if (ml[i] > cur + 0.01f) { n = i; break; }
            if (n < 0) n = 0;
        } else {
            for (int i = 4; i >= 0; i--) if (ml[i] < cur - 0.01f) { n = i; break; }
            if (n < 0) n = 4;
        }
        if (n == 0) snprintf(json, sizeof(json), "{\"target_mbps\":0}");
        else        snprintf(json, sizeof(json), "{\"target_mbps\":%d.0}", n);
        break;
    }
    case ACT_PV_TOGGLE:
        /* LCD 本地开关（不动流水线）：关=预览任务注销帧环深度休眠 +
         * 通知 worker 停产 pv（无流客户端时），算力让给主链 */
        if (!s_preview_on) { status_set("preview task n/a"); return; }
        s_pv_on = !s_pv_on;
        src_if_pv_lcd(s_pv_on);
        if (s_pv_on && s_pv_notify) xSemaphoreGive(s_pv_notify);
        setl(s_nosig, s_pv_on ? "NO SIGNAL" : "preview off");
        status_set(s_pv_on ? "preview on" : "preview off: CPU -> main");
        return;
    default:
        return;
    }
    send_json(json);
}

/* ---------------- 配置应用任务（与 /api/config 完全同路径） ---------------- */
static void apply_task(void *arg)
{
    (void)arg;
    cfg_msg_t m;
    for (;;) {
        if (xQueueReceive(s_cfgq, &m, portMAX_DELAY) != pdTRUE) continue;
        status_set("applying: %.60s", m.json);
        cJSON *j = cJSON_Parse(m.json);
        if (!j) { status_set("bad cmd"); continue; }
        app_cfg_result_t r;
        app_config_apply(j, &r);
        if (r.ok) {
            app_config_persist(j);
            status_set(r.restarted ? "ok (pipeline rebuilt)" : "ok (saved)");
        } else {
            status_set("FAIL: %.80s", r.error);
        }
        cJSON_Delete(j);
    }
}

/* ---------------- 预览：解码 + 缩放 + 呈现 ---------------- */
static void ensure_out(uint32_t w, uint32_t h)
{
    size_t aw = ((size_t)w + 31) / 32 * 32;
    size_t ah = ((size_t)h + 31) / 32 * 32;
    size_t need = aw * ah * 2;
    if (s_out && s_out_cap >= need) return;
    jpeg_decode_memory_alloc_cfg_t mc = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
    size_t cap = 0;
    void *p = jpeg_alloc_decoder_mem(need, &mc, &cap);
    if (!p) { ESP_LOGW(TAG, "解码输出缓冲分配失败（%u B）", (unsigned)need); return; }
    if (s_out) heap_caps_free(s_out);
    s_out = p;
    s_out_cap = cap;
}

static uint32_t dec_stride(uint32_t w, uint32_t h, uint32_t out_size)
{
    if (out_size == w * h * 2) return w;
    uint32_t cands[3] = { (w + 15) / 16 * 16, (w + 7) / 8 * 8, w };
    for (int i = 0; i < 3; i++) {
        uint32_t st = cands[i];
        if (st && out_size % (st * 2) == 0) {
            uint32_t rows = out_size / (st * 2);
            if (rows >= h) return st;
        }
    }
    return w;
}

/* 等比放大填满预览区（最近邻）：src[w*h，行距 stride] → fb 预览区 720x540。
 * 档位变化时清一次两页预览区（含黑边），黑边此后无人写、保持常黑 */
static int s_geo_w = -1, s_geo_h = -1;

static void prev_fill(uint16_t *dst, const uint16_t *src, int w, int h, int stride)
{
    if (w < 1 || h < 1) return;
    if (w != s_geo_w || h != s_geo_h) {
        for (int f = 0; f < 2; f++) {
            if (!s_fb[f]) continue;
            memset(s_fb[f], 0, (size_t)PREV_W * PREV_H * 2);
            esp_cache_msync(s_fb[f], (size_t)PREV_W * PREV_H * 2,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        }
        s_geo_w = w;
        s_geo_h = h;
    }
    int dw, dh;
    if ((int64_t)w * PREV_H > (int64_t)h * PREV_W) { dw = PREV_W; dh = (int)((int64_t)h * PREV_W / w); }
    else                                           { dh = PREV_H; dw = (int)((int64_t)w * PREV_H / h); }
    if (dw < 1) dw = 1;
    if (dh < 1) dh = 1;
    int ox = (PREV_W - dw) / 2, oy = (PREV_H - dh) / 2;
    static uint16_t mapx[PREV_W];
    static int16_t mapy[PREV_H];
    for (int x = 0; x < dw; x++) mapx[x] = (uint16_t)((int64_t)x * w / dw);
    for (int y = 0; y < dh; y++) mapy[y] = (int16_t)((int64_t)y * h / dh);
    for (int y = 0; y < dh; y++) {
        const uint16_t *srow = src + (size_t)mapy[y] * stride;
        uint16_t *drow = dst + (size_t)(oy + y) * LCD_H_RES + ox;
        for (int x = 0; x < dw; x++) drow[x] = srow[mapx[x]];
    }
}

/* 翻页呈现：后备页写好后以"页指针"为源调 draw_bitmap——驱动识别为帧缓冲
 * 驻留，只做 cache 回写并切换 cur_fb_index，DMA 在下一次帧完成边界自动
 * 换链表（esp_lcd_panel_dpi.c draw_bitmap_2d no-copy 分支），全程无撕裂 */
static bool present_flip(void)
{
    if (!s_panel || !s_real_draw || !s_fb[0]) return false;
    uint16_t *back = s_fb[s_cur ^ 1];
    if (s_real_draw(s_panel, 0, 0, LCD_H_RES, PREV_H, back) != ESP_OK) return false;
    s_cur ^= 1;
    return true;
}

static void preview_task(void *arg)
{
    (void)arg;
    const int fps_max = CONFIG_CAMTEST_LCD_PREVIEW_FPS;
    const int64_t period = fps_max > 0 ? 1000000LL / fps_max : 0;

    jpeg_decode_cfg_t dcfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,   /* 小端匹配 LVGL RGB565 LE */
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };

    SemaphoreHandle_t notify = s_pv_notify;
    frame_ring_t *reg = NULL;
    uint32_t last_fid = 0;
    float ema_ms = 0;
    int64_t next_ok = 0, win_t0 = esp_timer_get_time();
    int win_cnt = 0;

    for (;;) {
        if (!s_pv_on) {
            /* 预览关（屏上按钮）：注销帧环订阅、深睡等唤醒——解码/放大/
             * 翻页全部停止，PSRAM 带宽与 core0 时间让给主链 */
            if (reg) { frame_ring_unregister(reg, notify); reg = NULL; }
            s_prev_fps = 0;
            xSemaphoreTake(notify, portMAX_DELAY);
            continue;
        }
        frame_ring_t *cur = src_if_ring();
        if (cur && cur != reg) {
            if (reg) frame_ring_unregister(reg, notify);
            frame_ring_register(cur, notify);
            reg = cur;
            last_fid = frame_ring_last_fid(cur);
        }

        xSemaphoreTake(notify, pdMS_TO_TICKS(300));
        int64_t now = esp_timer_get_time();

        if (now - win_t0 >= 1000000) {
            s_prev_fps = (float)win_cnt * 1000000.0f / (float)(now - win_t0);
            win_cnt = 0;
            win_t0 = now;
        }
        if (period && now < next_ok) continue;

        bool got_pv = false;
        if (reg) {
            /* 双流取帧：h264 模式帧环只保留最新帧——若最新恰是 H264，它前面的
             * JPEG 已被顶掉，本轮注定落空（连取 3 次也只会 NULL）。落空时
             * 20ms 后快速重查贴住 pv 发布节奏，取到 JPEG 才回限速周期 */
            for (int tries = 0; tries < 3 && !got_pv; tries++) {
                frame_slot_t *s = frame_ring_acquire(reg, last_fid);
                if (!s) break;
                if (s->codec == FRAME_CODEC_JPEG) {
                    got_pv = true;
                    jpeg_decode_picture_info_t info;
                    if (jpeg_decoder_get_info(s->data, (uint32_t)s->len, &info) == ESP_OK &&
                        info.width && info.height) {
                        ensure_out(info.width, info.height);
                        if (s_out) {
                            int64_t t0 = esp_timer_get_time();
                            uint32_t out_size = 0;
                            if (jpeg_dec_share_process(&dcfg, s->data, (uint32_t)s->len,
                                                       s_out, (uint32_t)s_out_cap,
                                                       &out_size) == ESP_OK) {
                                float ms = (float)(esp_timer_get_time() - t0) / 1000.0f;
                                ema_ms += 0.15f * (ms - ema_ms);
                                s_dec_ms = ema_ms;
                                uint32_t stride = dec_stride(info.width, info.height, out_size);
                                prev_fill(s_fb[s_cur ^ 1], (const uint16_t *)s_out,
                                          (int)info.width, (int)info.height, (int)stride);
                                present_flip();
                                s_last_frame_us = (uint32_t)esp_timer_get_time();
                                win_cnt++;
                            }
                        }
                    }
                    last_fid = s->fid;
                    frame_ring_release(reg, s);
                    break;
                }
                frame_ring_release(reg, s);
                last_fid = s->fid;   /* h264 帧也推进：否则下轮 acquire 重取同帧空转 */
            }
        }
        next_ok = now + (got_pv || !period ? period : 20000);
    }
}

/* ---------------- LVGL UI ---------------- */
static lv_obj_t *mk_btn(lv_obj_t *par, int x, int y, int w, int h,
                        const char *txt, act_t act)
{
    lv_obj_t *b = lv_button_create(par);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_radius(b, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b, lv_color_hex(0xd6e0ea), LV_PART_MAIN);
    lv_obj_set_style_bg_color(b, lv_color_hex(0xb8c8da), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(b, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(b, lv_color_hex(0x9db2c6), LV_PART_MAIN);
    lv_obj_set_style_pad_all(b, 0, LV_PART_MAIN);
    lv_obj_set_style_text_color(b, lv_color_hex(0x16222e), LV_PART_MAIN);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, act_event, LV_EVENT_CLICKED, (void *)(uintptr_t)act);
    return b;
}

static lv_obj_t *mk_cap(lv_obj_t *par, int x, int y, const char *txt)
{
    lv_obj_t *l = lv_label_create(par);
    lv_obj_set_pos(l, x, y);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, lv_color_hex(0x5a6b7d), 0);
    return l;
}

static lv_obj_t *mk_stat(lv_obj_t *par, int x, int y, int w)
{
    lv_obj_t *l = lv_label_create(par);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_width(l, w);
    lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(l, lv_color_hex(0x1e2a36), 0);
    lv_label_set_text(l, "-");
    return l;
}

static lv_obj_t *mk_val(lv_obj_t *par, int x, int y, int w)
{
    lv_obj_t *l = lv_label_create(par);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_width(l, w);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0x14202c), 0);
    lv_label_set_text(l, "-");
    return l;
}

static void hl(lv_obj_t *b, bool on)
{
    lv_obj_set_style_bg_color(b, on ? lv_color_hex(0x2f6fd0) : lv_color_hex(0xd6e0ea), LV_PART_MAIN);
    lv_obj_set_style_text_color(b, on ? lv_color_hex(0xffffff) : lv_color_hex(0x16222e), LV_PART_MAIN);
}

/* 2 Hz 面板刷新 */
static void panel_timer(lv_timer_t *t)
{
    (void)t;
    static int s_hb;
    if (++s_hb >= 10) {
        s_hb = 0;
        ESP_LOGI(TAG, "hb: flush=%u err=%u prev=%.1f dec=%.1fms",
                 (unsigned)s_flush_cnt, (unsigned)s_flush_err, s_prev_fps, s_dec_ms);
    }
    src_info_t *ci = src_if_info();
    metrics_t *m = metrics_get();
    wifi_info_t *w = wifi_net_info();
    bool usb = ci->source == VIDEO_SOURCE_USB;
    char buf[128];

    snprintf(buf, sizeof(buf), "FPS  cap %.1f  enc %.1f  send %.1f",
             m->cap_fps, m->enc_fps, m->send_fps);
    setl(l_fps, buf);
    snprintf(buf, sizeof(buf), "Rate  %.2f Mbps  (%u kB)  pv %.2f",
             m->bitrate_mbps, (unsigned)m->jpeg_avg_bytes, m->pv_mbps);
    setl(l_rate, buf);
    snprintf(buf, sizeof(buf), "CPU  %.0f%% / %.0f%%  drop %u/%u",
             m->cpu0, m->cpu1, (unsigned)m->drop_capture, (unsigned)m->drop_encode);
    setl(l_cpu, buf);
    snprintf(buf, sizeof(buf), "Heap  %.1fk (min %.1fk)  PSRAM %.1fM",
             m->free_heap / 1024.0f, m->min_heap / 1024.0f, m->free_psram / 1048576.0f);
    setl(l_heap, buf);
    setl(l_psram, buf);

    if (usb) {
        const char *um = ci->usb_mode == USB_MODE_H264 ? "h264" :
                         ci->usb_mode == USB_MODE_REENCODE ? "re" : "pt";
        snprintf(buf, sizeof(buf), "SRC usb/%s %s %ux%u", usb_state_str(), um, ci->w, ci->h);
    } else {
        snprintf(buf, sizeof(buf), "SRC dvp/%s %ux%u q%u", ci->sensor_name, ci->w, ci->h, ci->quality);
    }
    setl(l_src, buf);

    snprintf(buf, sizeof(buf), "WIFI %s %d dBm %s", w->mode == WIFI_MODE_STA_M ? "STA" : "AP",
             w->rssi, w->ssid);
    setl(l_wifi, buf);
    snprintf(buf, sizeof(buf), "IP %s  %s.local", w->ip, CONFIG_CAMTEST_MDNS_HOSTNAME);
    setl(l_ip, buf);
    snprintf(buf, sizeof(buf), "LCD  %.1f fps  %.1f ms  (board v%d)",
             s_prev_fps, s_dec_ms, (int)s_board_ver);
    setl(l_lcd, buf);
    setl(l_e2e, buf);   /* 占位（Tab5 无浏览器侧 e2e，显示 LCD 自身） */

    /* 控件值 */
    snprintf(buf, sizeof(buf), "%ux%u", ci->w, ci->h);
    setl(v_res, buf);
    snprintf(buf, sizeof(buf), "%lu kbps", (unsigned long)src_if_h264_kbps());
    setl(v_kb, buf);
    if (usb && ci->usb_mode == USB_MODE_REENCODE) {
        snprintf(buf, sizeof(buf), "q=%u", ci->quality);
    } else {
        snprintf(buf, sizeof(buf), usb ? "n/a" : "q=%u", ci->quality);
    }
    setl(v_q, buf);
    if (ci->fps_limit > 0) snprintf(buf, sizeof(buf), "max %d", ci->fps_limit);
    else                   snprintf(buf, sizeof(buf), "off");
    setl(v_fps, buf);
    if (m->target_mbps > 0.01f) snprintf(buf, sizeof(buf), "%.1f", m->target_mbps);
    else                        snprintf(buf, sizeof(buf), "off");
    setl(v_mb, buf);
    setl(b_ov_lab, src_if_overlay() ? "ON" : "OFF");

    int st_src = usb ? 1 : 0;
    if (st_src != s_prev_src) { hl(b_dvp, !usb); hl(b_usb, usb); s_prev_src = st_src; }
    int st_um = usb ? (int)ci->usb_mode : -1;
    if (st_um != s_prev_um) {
        hl(b_pt, st_um == USB_MODE_PASSTHROUGH);
        hl(b_h264, st_um == USB_MODE_H264);
        hl(b_re, st_um == USB_MODE_REENCODE);
        s_prev_um = st_um;
    }
    int st_ov = src_if_overlay() ? 1 : 0;
    if (st_ov != s_prev_ov) { hl(b_ov, st_ov != 0); s_prev_ov = st_ov; }
    int st_pv = s_pv_on ? 1 : 0;
    if (st_pv != s_prev_pv) {
        hl(b_pv, st_pv != 0);
        setl(b_pv_lab, st_pv ? "ON" : "OFF");
        s_prev_pv = st_pv;
    }

    if (s_fb[0]) {
        uint32_t now32 = (uint32_t)esp_timer_get_time();
        bool show = !s_pv_on || (s_last_frame_us == 0) ||
                    (uint32_t)(now32 - s_last_frame_us) > 3000000u;
        if (show != s_nosig_shown) {
            lv_obj_set_hidden(s_nosig, !show);
            s_nosig_shown = show;
        }
    }

    char st[112];
    status_copy(st, sizeof(st));
    setl(l_status, st);
    int col = 0;
    if (strncmp(st, "FAIL", 4) == 0) col = 1;
    else if (strncmp(st, "Locked", 6) == 0) col = 2;
    else if (strncmp(st, "ok", 2) == 0) col = 3;
    if (col != s_st_col) {
        static const uint32_t cols[] = { 0x51616f, 0xc0392b, 0xb07d18, 0x2e8b57 };
        lv_obj_set_style_text_color(l_status, lv_color_hex(cols[col]), 0);
        s_st_col = col;
    }
}

static void build_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xf2f5f9), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_scrollable(scr, false);

    /* 预览区（0,0,720,540） */
    lv_obj_t *pv = lv_obj_create(scr);
    lv_obj_set_pos(pv, 0, 0);
    lv_obj_set_size(pv, PREV_W, PREV_H);
    lv_obj_set_style_bg_color(pv, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(pv, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(pv, 0, 0);
    lv_obj_set_style_border_width(pv, 0, 0);
    lv_obj_set_style_radius(pv, 0, 0);
    lv_obj_set_scrollable(pv, false);
    /* 预览画面不走 LVGL（present 直写 DSI 帧缓冲）——这里只留黑底容器 */
    s_nosig = lv_label_create(pv);
    lv_label_set_text(s_nosig, s_fb[0] ? "NO SIGNAL" : "preview off (Kconfig)");
    lv_obj_set_style_bg_color(s_nosig, lv_color_hex(0xffe3e3), 0);
    lv_obj_set_style_bg_opa(s_nosig, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(s_nosig, lv_color_hex(0xc0392b), 0);
    lv_obj_set_style_pad_all(s_nosig, 8, 0);
    lv_obj_set_style_radius(s_nosig, 6, 0);
    lv_obj_center(s_nosig);

    /* 状态行两列 */
    l_fps   = mk_stat(scr, 8,   STAT_Y0 + 0 * STAT_STEP, 350);
    l_rate  = mk_stat(scr, 8,   STAT_Y0 + 1 * STAT_STEP, 350);
    l_cpu   = mk_stat(scr, 8,   STAT_Y0 + 2 * STAT_STEP, 350);
    l_heap  = mk_stat(scr, 8,   STAT_Y0 + 3 * STAT_STEP, 350);
    l_src   = mk_stat(scr, 368, STAT_Y0 + 0 * STAT_STEP, 344);
    l_wifi  = mk_stat(scr, 368, STAT_Y0 + 1 * STAT_STEP, 344);
    l_ip    = mk_stat(scr, 368, STAT_Y0 + 2 * STAT_STEP, 344);
    l_lcd   = mk_stat(scr, 368, STAT_Y0 + 3 * STAT_STEP, 344);
    l_psram = mk_stat(scr, 8,   STAT_Y0 + 4 * STAT_STEP, 350);
    l_e2e   = mk_stat(scr, 368, STAT_Y0 + 4 * STAT_STEP, 344);

    /* 控制面板 */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "CamTest Tab5 LCD Console");
    lv_obj_set_pos(title, 8, CTL_Y0 - 30);
    lv_obj_set_style_text_color(title, lv_color_hex(0x14202c), 0);

    int y = CTL_Y0;
    mk_cap(scr, 8, y + 6, "Source");
    b_dvp = mk_btn(scr, 240, y, 220, 42, "DVP (MIPI)", ACT_SRC_DVP);
    b_usb = mk_btn(scr, 480, y, 220, 42, "USB UVC", ACT_SRC_USB);

    y += CTL_STEP;
    mk_cap(scr, 8, y + 6, "Mode");
    b_pt   = mk_btn(scr, 240, y, 148, 42, "PassThru", ACT_UM_PT);
    b_h264 = mk_btn(scr, 396, y, 148, 42, "H264", ACT_UM_H264);
    b_re   = mk_btn(scr, 552, y, 148, 42, "ReEnc", ACT_UM_RE);

    y += CTL_STEP;
    mk_cap(scr, 8, y + 6, "Res");
    mk_btn(scr, 240, y, 70, 42, "<", ACT_RES_DEC);
    v_res = mk_val(scr, 318, y + 10, 304);
    mk_btn(scr, 630, y, 70, 42, ">", ACT_RES_INC);

    y += CTL_STEP;
    mk_cap(scr, 8, y + 6, "H264 kbps");
    mk_btn(scr, 240, y, 70, 42, "<", ACT_KB_DEC);
    v_kb = mk_val(scr, 318, y + 10, 304);
    mk_btn(scr, 630, y, 70, 42, ">", ACT_KB_INC);

    y += CTL_STEP;
    mk_cap(scr, 8, y + 6, "Quality");
    mk_btn(scr, 240, y, 70, 42, "<", ACT_Q_DEC);
    v_q = mk_val(scr, 318, y + 10, 304);
    mk_btn(scr, 630, y, 70, 42, ">", ACT_Q_INC);

    y += CTL_STEP;
    mk_cap(scr, 8, y + 6, "Fps max");
    mk_btn(scr, 240, y, 70, 42, "<", ACT_FPS_DEC);
    v_fps = mk_val(scr, 318, y + 10, 160);
    mk_btn(scr, 490, y, 70, 42, ">", ACT_FPS_INC);
    mk_cap(scr, 578, y + 6, "PV");
    b_pv = mk_btn(scr, 612, y, 100, 42, "ON", ACT_PV_TOGGLE);
    b_pv_lab = lv_obj_get_child(b_pv, 0);

    y += CTL_STEP;
    mk_cap(scr, 8, y + 6, "Overlay");
    b_ov = mk_btn(scr, 240, y, 140, 42, "OFF", ACT_OV_TOGGLE);
    b_ov_lab = lv_obj_get_child(b_ov, 0);
    mk_cap(scr, 430, y + 6, "Mbps cap");
    mk_btn(scr, 560, y, 55, 42, "<", ACT_MB_DEC);
    v_mb = mk_val(scr, 615, y + 10, 40);
    mk_btn(scr, 655, y, 55, 42, ">", ACT_MB_INC);

    y += CTL_STEP + 8;
    l_status = lv_label_create(scr);
    lv_obj_set_pos(l_status, 8, y);
    lv_obj_set_size(l_status, 704, 58);
    lv_label_set_long_mode(l_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_bg_color(l_status, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_bg_opa(l_status, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(l_status, 6, 0);
    lv_obj_set_style_radius(l_status, 6, 0);
    lv_obj_set_style_text_color(l_status, lv_color_hex(0x22303c), 0);
    lv_label_set_text(l_status, "-");

    /* 目验横幅 */
    lv_obj_t *banner = lv_label_create(scr);
    lv_obj_set_pos(banner, 8, y + 66);
    lv_obj_set_size(banner, 704, 40);
    lv_obj_set_style_bg_color(banner, lv_color_hex(0xffd54d), 0);
    lv_obj_set_style_bg_opa(banner, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(banner, 8, 0);
    lv_obj_set_style_text_color(banner, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_text_align(banner, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_top(banner, 8, 0);
    lv_label_set_text(banner, "LCD OK · 720x1280 DSI · CamTest Tab5");

    lv_timer_create(panel_timer, 500, NULL);
}

/* ---------------- 面板初始化（三代屏，自 esp-bsp 提取） ---------------- */
static board_version_t detect_board_version(void)
{
    if (s_board_ver != BOARD_V_UNKNOWN) return s_board_ver;
    i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)board_i2c_handle();

    /* 触摸 IC 与屏驱动板同电源域：先放 LCD 电源再上触摸，给足上电时序
     * （BSP 只开 TOUCH_EN；真机实测部分批次需要 LCD 电先就位触摸才应答） */
    board_lcd_enable(true);
    board_touch_enable(true);
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int round = 0; round < 2 && s_board_ver == BOARD_V_UNKNOWN; round++) {
        if (round == 1) {
            /* 二轮：把 TP_INT 拉低脉冲一次（触摸复位时序），再等 300ms */
            gpio_config_t ig = {
                .mode = GPIO_MODE_OUTPUT, .intr_type = GPIO_INTR_DISABLE,
                .pull_down_en = 0, .pull_up_en = 0, .pin_bit_mask = BIT64(LCD_TOUCH_INT_GPIO),
            };
            gpio_config(&ig);
            gpio_set_level(LCD_TOUCH_INT_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(LCD_TOUCH_INT_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        if (i2c_master_probe(bus, ESP_LCD_TOUCH_IO_I2C_ST7123_ADDRESS, 100) == ESP_OK) {
            esp_lcd_panel_io_handle_t io = NULL;
            esp_lcd_panel_io_i2c_config_t c = ESP_LCD_TOUCH_IO_I2C_ST7123_CONFIG();
            uint8_t fw = 0;
            if (esp_lcd_new_panel_io_i2c(bus, &c, &io) == ESP_OK) {
                esp_lcd_panel_io_rx_param(io, 0x0000, &fw, sizeof(fw));
                esp_lcd_panel_io_del(io);
            }
            if (fw == 1)      { s_board_ver = BOARD_V_ST7121; ESP_LOGI(TAG, "板版3：LCD ST7121 + 触摸 ST712x (fw=%u)", fw); }
            else if (fw == 3) { s_board_ver = BOARD_V_ST7123; ESP_LOGI(TAG, "板版2：LCD ST7123 + 触摸 ST7123 (fw=%u)", fw); }
            else { s_board_ver = BOARD_V_ST7123; ESP_LOGW(TAG, "ST712x 固件版本未知 fw=%u（按 ST7123 处理）", fw); }
        } else if (i2c_master_probe(bus, ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP, 100) == ESP_OK) {
            s_board_ver = BOARD_V_ILI9881C_GT911;
            ESP_LOGI(TAG, "板版1：LCD ILI9881C + 触摸 GT911");
        } else if (round == 1 && i2c_master_probe(bus, 0x5D, 100) == ESP_OK) {
            /* GT911 主地址（复位脚时序决定 0x5D/0x14） */
            s_board_ver = BOARD_V_ILI9881C_GT911;
            ESP_LOGI(TAG, "板版1：LCD ILI9881C + 触摸 GT911（主地址 0x5D）");
        }
    }
    if (s_board_ver == BOARD_V_UNKNOWN)
        ESP_LOGE(TAG, "无法识别屏版本（0x55/0x14/0x5D 均无应答）");
    return s_board_ver;
}

static esp_err_t backlight_init(void)
{
    const ledc_timer_config_t tm = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LCD_LED_C_TIMER, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t ch = {
        .gpio_num = LCD_BACKLIGHT_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LED_C_CH, .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LCD_LED_C_TIMER, .duty = 0, .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tm), TAG, "ledc timer");
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "ledc ch");
    return ESP_OK;
}

static esp_err_t panel_init(esp_lcd_panel_handle_t *out_panel, esp_lcd_panel_io_handle_t *out_io)
{
    board_lcd_enable(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_RETURN_ON_ERROR(backlight_init(), TAG, "backlight");

    /* DSI PHY 电源（内部 LDO3 2.5V） */
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t lc = { .chan_id = LCD_DPHY_LDO_CHAN, .voltage_mv = LCD_DPHY_LDO_MV };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&lc, &ldo), TAG, "dsi ldo");

    board_version_t v = detect_board_version();
    ESP_RETURN_ON_FALSE(v != BOARD_V_UNKNOWN, ESP_ERR_NOT_SUPPORTED, TAG, "unknown board");

    esp_lcd_dsi_bus_handle_t bus = NULL;
    esp_lcd_dsi_bus_config_t bc = {
        .bus_id = 0,
        .num_data_lanes = LCD_MIPI_LANE_NUM,
        .phy_clk_src = 0,
        .lane_bit_rate_mbps = (v == BOARD_V_ILI9881C_GT911) ? LCD_LANE_MBPS : LCD_LANE_MBPS_ST,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bc, &bus), TAG, "dsi bus");

    esp_lcd_dbi_io_config_t dbi = { .virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8 };
    esp_lcd_panel_io_handle_t io = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(bus, &dbi, &io), TAG, "dbi io");

    /* 三套 DSI 时序（esp-bsp 同源） */
    /* num_fbs=2：预览直写后备页，draw_bitmap(fb 指针) 在帧完成边界翻页
     * （无撕裂）。LVGL 局部刷新经 draw_shim 镜像同步两页 UI 区域。
     * 每页 1.8MB PSRAM（共 3.6MB），换掉撕裂的"写扫描中缓冲"路径 */
    const esp_lcd_dpi_panel_config_t dpi_ili9881c = {
        .virtual_channel = 0, .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 60, .in_color_format = LCD_COLOR_FMT_RGB565, .num_fbs = 2,
        .video_timing = { .h_size = LCD_H_RES, .v_size = LCD_V_RES,
                          .hsync_back_porch = 140, .hsync_pulse_width = 40, .hsync_front_porch = 40,
                          .vsync_back_porch = 20, .vsync_pulse_width = 4, .vsync_front_porch = 20 },
    };
    const esp_lcd_dpi_panel_config_t dpi_st7123 = {
        .virtual_channel = 0, .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 70, .in_color_format = LCD_COLOR_FMT_RGB565, .num_fbs = 2,
        .video_timing = { .h_size = LCD_H_RES, .v_size = LCD_V_RES,
                          .hsync_back_porch = 40, .hsync_pulse_width = 2, .hsync_front_porch = 40,
                          .vsync_back_porch = 8, .vsync_pulse_width = 2, .vsync_front_porch = 220 },
    };
    const esp_lcd_dpi_panel_config_t dpi_st7121 = {
        .virtual_channel = 0, .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 70, .in_color_format = LCD_COLOR_FMT_RGB565, .num_fbs = 2,
        .video_timing = { .h_size = LCD_H_RES, .v_size = LCD_V_RES,
                          .hsync_back_porch = 40, .hsync_pulse_width = 2, .hsync_front_porch = 40,
                          .vsync_back_porch = 24, .vsync_pulse_width = 20, .vsync_front_porch = 200 },
    };

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t pc = {
        .reset_gpio_num = -1,
        .flags.reset_active_high = 0,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    if (v == BOARD_V_ILI9881C_GT911) {
        const ili9881c_vendor_config_t vc = {
            .init_cmds = disp_init_data_ili9881c,
            .init_cmds_size = sizeof(disp_init_data_ili9881c) / sizeof(disp_init_data_ili9881c[0]),
            .mipi_config = { .dsi_bus = bus, .dpi_config = &dpi_ili9881c, .lane_num = LCD_MIPI_LANE_NUM },
        };
        pc.vendor_config = (void *)&vc;
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9881c(io, &pc, &panel), TAG, "ili9881c");
    } else {
        const st7123_vendor_config_t vc = {
            .init_cmds = (v == BOARD_V_ST7121) ? disp_init_data_st7121 : disp_init_data_st7123,
            .init_cmds_size = (v == BOARD_V_ST7121)
                ? sizeof(disp_init_data_st7121) / sizeof(disp_init_data_st7121[0])
                : sizeof(disp_init_data_st7123) / sizeof(disp_init_data_st7123[0]),
            .mipi_config = { .dsi_bus = bus,
                             .dpi_config = (v == BOARD_V_ST7121) ? &dpi_st7121 : &dpi_st7123 },
        };
        pc.vendor_config = (void *)&vc;
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7123(io, &pc, &panel), TAG, "st7123");
    }
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, false);
    esp_lcd_panel_mirror(panel, false, false);

    *out_panel = panel;
    *out_io = io;
    ESP_LOGI(TAG, "DSI 面板就绪（板版%d，%dx%d）", (int)v, LCD_H_RES, LCD_V_RES);
    return ESP_OK;
}

static esp_err_t touch_init(esp_lcd_touch_handle_t *tp)
{
    i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)board_i2c_handle();
    esp_lcd_touch_config_t tc = {
        .x_max = LCD_H_RES, .y_max = LCD_V_RES,
        .rst_gpio_num = -1, .int_gpio_num = LCD_TOUCH_INT_GPIO,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
    };
    esp_lcd_panel_io_handle_t io = NULL;
    if (s_board_ver == BOARD_V_ILI9881C_GT911) {
        /* GT911：INT 脚有 3V3 上拉电阻会卡住触摸——保持输出低（esp-bsp 同款修复） */
        const gpio_config_t ig = {
            .mode = GPIO_MODE_OUTPUT, .intr_type = GPIO_INTR_DISABLE,
            .pull_down_en = 0, .pull_up_en = 1, .pin_bit_mask = BIT64(LCD_TOUCH_INT_GPIO),
        };
        gpio_config(&ig);
        gpio_set_level(LCD_TOUCH_INT_GPIO, 0);
        tc.int_gpio_num = -1;
        esp_lcd_panel_io_i2c_config_t c = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
        c.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus, &c, &io), TAG, "gt911 io");
        return esp_lcd_touch_new_i2c_gt911(io, &tc, tp);
    }
    esp_lcd_panel_io_i2c_config_t c = ESP_LCD_TOUCH_IO_I2C_ST7123_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus, &c, &io), TAG, "st7123 io");
    return esp_lcd_touch_new_i2c_st7123(io, &tc, tp);
}

/* ---------------- 对外接口 ---------------- */
esp_err_t lcd_ui_start(void)
{
    if (s_active) return ESP_OK;

    if (detect_board_version() == BOARD_V_UNKNOWN) {
        ESP_LOGW(TAG, "屏版本未知，LCD UI 未启用");
        return ESP_ERR_NOT_FOUND;
    }

    s_cfgq = xQueueCreate(4, sizeof(cfg_msg_t));
    if (!s_cfgq) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "display-init前内存: internal=%u dma=%u psram=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_io_handle_t io = NULL;
    ESP_RETURN_ON_ERROR(panel_init(&panel, &io), TAG, "panel");
    esp_lcd_panel_disp_on_off(panel, true);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LED_C_CH, 1023);   /* 背光 100% */
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LED_C_CH);
    s_panel = panel;
    s_real_draw = panel->draw_bitmap;
    panel->draw_bitmap = draw_shim;

    /* 驱动双帧缓冲（num_fbs=2，PSRAM 黑底）：预览写后备页 + 页指针翻页 */
    void *fb0 = NULL, *fb1 = NULL;
    if (esp_lcd_dpi_panel_get_frame_buffer(panel, 2, &fb0, &fb1) == ESP_OK) {
        s_fb[0] = fb0;
        s_fb[1] = fb1;
        s_fb_px = (size_t)LCD_H_RES * LCD_V_RES;
        s_cur = 0;
        ESP_LOGI(TAG, "DSI 双帧缓冲就绪：fb0=%p fb1=%p（%u KB/页）",
                 fb0, fb1, (unsigned)((size_t)LCD_H_RES * LCD_V_RES * 2 / 1024));
    } else {
        ESP_LOGW(TAG, "DSI 帧缓冲获取失败，仅参数面板（无预览）");
    }

    lvgl_port_cfg_t lcfg = ESP_LVGL_PORT_INIT_CONFIG();
    lcfg.task_affinity = 0;   /* LVGL 渲染钉 core0（采集/编码/worker 在 core1） */
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lcfg), TAG, "lvgl init");

    lvgl_port_display_cfg_t dc = {
        .io_handle = io,
        .panel_handle = panel,
        .buffer_size = LCD_H_RES * 128,   /* 720x128 像素双缓冲（PSRAM） */
        .double_buffer = 1,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
        .flags = { .buff_dma = false, .buff_spiram = true, .sw_rotate = false, .swap_bytes = false },
    };
    const lvgl_port_display_dsi_cfg_t dsi = { .flags = { .avoid_tearing = false } };
    lv_display_t *disp = lvgl_port_add_disp_dsi(&dc, &dsi);
    if (!disp) {
        ESP_LOGE(TAG, "LVGL DSI 显示注册失败");
        return ESP_FAIL;
    }

    esp_lcd_touch_handle_t tp = NULL;
    if (touch_init(&tp) == ESP_OK && tp) {
        const lvgl_port_touch_cfg_t tcfg = { .disp = disp, .handle = tp };
        if (!lvgl_port_add_touch(&tcfg)) ESP_LOGW(TAG, "触摸输入注册失败（仅显示）");
    } else {
        ESP_LOGW(TAG, "触摸初始化失败（仅显示）");
    }

    s_active = true;
    if (!lvgl_port_lock(1000)) {
        ESP_LOGE(TAG, "LVGL 锁超时，UI 未构建");
        return ESP_FAIL;
    }
    build_ui();
    lvgl_port_unlock();

    if (xTaskCreatePinnedToCore(apply_task, "lcd_cfg", 8192, NULL, 5, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    s_pv_notify = xSemaphoreCreateBinary();
    if (s_fb[0] && s_pv_notify && CONFIG_CAMTEST_LCD_PREVIEW_FPS > 0) {
        /* ★ core0：core1 被 worker(prio13) 高占空碾压，prio2 的预览在 core1
         *   会饿死（实测 got=1-2 帧/s）；core0 只有 LVGL(prio4) 间歇忙，
         *   prio2 塞间隙足够（S31 版同思路：渲染/预览与采集编码分核） */
        if (xTaskCreatePinnedToCore(preview_task, "lcd_prev", 6144, NULL, 2, NULL, 0) == pdPASS)
            s_preview_on = true;
        else ESP_LOGE(TAG, "预览任务创建失败，仅参数面板");
    }
    status_set(s_preview_on ? "LCD ready (preview on)" : "LCD ready (panel only)");
    ESP_LOGI(TAG, "LCD UI 已启动 %dx%d 板版%d，预览 %s（限速 %d fps）",
             LCD_H_RES, LCD_V_RES, (int)s_board_ver,
             s_preview_on ? "on" : "off", CONFIG_CAMTEST_LCD_PREVIEW_FPS);
    ESP_LOGI(TAG, "display-start后内存: internal=%u dma=%u psram=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return ESP_OK;
}

lcd_ui_stats_t lcd_ui_stats(void)
{
    lcd_ui_stats_t s = {
        .active = s_active,
        .preview_on = s_preview_on,
        .preview_fps = s_prev_fps,
        .dec_ms = s_dec_ms,
        .flush_cnt = s_flush_cnt,
        .flush_err = s_flush_err,
    };
    return s;
}

bool lcd_ui_active(void) { return s_active; }

#else  /* !CONFIG_CAMTEST_ENABLE_LCD */

esp_err_t lcd_ui_start(void) { return ESP_ERR_NOT_SUPPORTED; }
lcd_ui_stats_t lcd_ui_stats(void) { lcd_ui_stats_t s = {0}; return s; }
bool lcd_ui_active(void) { return false; }

#endif /* CONFIG_CAMTEST_ENABLE_LCD */
