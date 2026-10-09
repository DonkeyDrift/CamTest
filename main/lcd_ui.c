/*
 * lcd_ui.c — LCD 实时预览 + 运行参数面板 + 触屏调参（800x480 RGB + GT1151 触摸）
 *
 * 【为什么这样做才能把对帧率/时延的影响压到最低】
 *  1. 预览 = frame_ring 订阅者，与流客户端同一路径：acquire 只取"比上次新"
 *     的帧（跳帧不排队，refcnt 槽保护发布者），对编码/发送零感知；
 *  2. 硬件 JPEG 编/解码器是 SoC 单例（codec_mutex 为二值信号量、无优先级
 *     继承），解码与编码互斥。三条纪律把碰撞压到最小：
 *       - 限速 CONFIG_CAMTEST_LCD_PREVIEW_FPS（0 = 不创建解码引擎，编码零影响）；
 *       - 收到"帧刚发布"的通知立刻解码——此刻编码刚结束、下一帧还要等采集
 *         DQBUF（典型 ≥7ms），解码正好落进编码空闲窗；
 *       - 预览任务钉 core1 最低优先级（2），采集14/编码12 可随时抢占它；
 *  3. 解码只丢不排：超速就等下一次通知，绝不积压（时延不累积）；
 *  4. LVGL 渲染任务钉 core0（不占采集/编码核），参数面板 2 Hz 刷新；
 *  5. 触屏改参与网页 POST /api/config 走完全同一条代码路径
 *     （app_config_apply 全部成功才 persist），扫描进行中拒绝修改；
 *  6. 双缓冲交替 set_src：LVGL 只读 front、解码只写 back，互不踩。
 *
 * 【已知取舍】
 *  - 解码偶尔会让编码器多等一个互斥窗（上限≈单帧解码时长）：靠限速+空闲窗
 *    对齐压碰撞概率；真机影响用扫描 CSV 的 lcd_on 列开关对照实测。
 *  - 硬解 RGB565 字节序取 BGR（小端，匹配 LVGL RGB565 LE）。若真机颜色异常
 *    （红蓝互换），把 LCD_DEC_RGB_ORDER 改成 RGB 重编译即可。
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
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"
#include "driver/jpeg_decode.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"   /* esp_lcd_panel_t vtable（flush shim 需要） */
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "bsp/esp32_s31_korvo_1.h"
#include "bsp/touch.h"
#include "cJSON.h"
#include "app_config.h"
#include "camera_pipeline.h"
#include "frame_ring.h"
#include "metrics.h"
#include "scan_ctrl.h"
#include "source_if.h"
#include "wifi_net.h"

static const char *TAG = "lcd_ui";

/* GT1151 触摸 I2C 地址 = esp_lcd_touch_gt1151.h 的
 * ESP_LCD_TOUCH_IO_I2C_GT1151_ADDRESS（与相机 0x78 / 音频 0x20 同总线不冲突） */
#define GT1151_I2C_ADDR     0x14

/* 硬解输出字节序：BGR=小端（匹配 LVGL RGB565 LE）；颜色异常时改 RGB 试验 */
#define LCD_DEC_RGB_ORDER   JPEG_DEC_RGB_ELEMENT_ORDER_BGR

#define RP_X                (BSP_LCD_H_RES - 320)   /* 右侧参数面板左边界 480 */
#define RP_W                320
#define PREV_W              (BSP_LCD_H_RES - RP_X)  /* 480 */
#define PREV_H              270
#define STAT_X1             8
#define STAT_X2             252
#define STAT_Y0             274
#define STAT_STEP           20

/* ---------------- 全局状态 ---------------- */
static volatile bool s_active;
static volatile bool s_preview_on;              /* 预览任务在跑 */
static volatile bool s_pv_on = true;            /* 屏上 PV 开关（关=任务深睡省 CPU，不落 NVS） */

/* 预览双缓冲（PSRAM）+ 硬解输出缓冲 */
static uint16_t *s_pbuf[2];
static lv_image_dsc_t s_dsc[2];
static int s_back;                              /* 下一帧写入的缓冲号（仅预览任务改） */
static jpeg_decoder_handle_t s_jdec;
static uint8_t *s_out;
static size_t s_out_cap;

/* 预览统计（面板/status JSON 读）；时间戳用 32 位避免跨核读撕裂（回绕安全） */
static volatile float s_prev_fps;
static volatile float s_dec_ms;
static volatile uint32_t s_last_frame_us;

/* 状态行：seqlock（LVGL 任务与配置任务都是写者） */
static char s_status[112];
static uint32_t s_seq;
static portMUX_TYPE s_st_mux = portMUX_INITIALIZER_UNLOCKED;

/* 配置队列（触屏 → 异步 apply，LVGL 任务永不阻塞在链路重建上） */
typedef struct { char json[176]; } cfg_msg_t;
static QueueHandle_t s_cfgq;

/* 预览唤醒信号量：帧环发布 + PV 按钮唤醒共用（预览关时任务深睡等它） */
static SemaphoreHandle_t s_pv_notify;

/* LVGL 控件（build_ui 创建；panel_timer 更新；均活在 LVGL 任务里） */
static lv_obj_t *s_img, *s_nosig;
static lv_obj_t *l_fps, *l_rate, *l_cpu, *l_heap, *l_psram;
static lv_obj_t *l_src, *l_wifi, *l_ip, *l_lcd, *l_scan;
static lv_obj_t *b_dvp, *b_usb, *b_pt, *b_re, *b_ov, *b_ov_lab, *b_pv, *b_pv_lab;
static lv_obj_t *v_res, *v_q, *v_fps, *v_hp, *v_mb;
static lv_obj_t *w_usb[3], *w_dvp[4];           /* 条件显隐的两组行 */
static lv_obj_t *l_status;
static bool s_nosig_shown = true;
static int s_prev_src = -1, s_prev_um = -1, s_prev_ov = -1, s_prev_pv = -1, s_prev_vis = -1, s_st_col = -1;

/* ---------------- 面板直通 + flush 观测 + 测试图案 ----------------
 * esp_lvgl_port 的 flush 忽略 draw_bitmap 返回值：写帧缓冲失败会"静默黑屏"、
 * LVGL 照常心跳。这里在面板 vtable 上垫一层 shim 计 flush 次数与首个错误，
 * 心跳日志一并打出；测试图案绕过 LVGL 直写帧缓冲，把"面板/背光/时序"与
 * "LVGL 通路"一分为二。 */
static esp_lcd_panel_handle_t s_panel;
static esp_err_t (*s_real_draw)(esp_lcd_panel_t *, int, int, int, int, const void *);
static volatile uint32_t s_flush_cnt, s_flush_err;
static volatile int64_t s_pattern_until_us;   /* 图案停留期：预览/面板刷新让位 */

static esp_err_t draw_shim(esp_lcd_panel_t *p, int x0, int y0, int x1, int y1, const void *d)
{
    s_flush_cnt++;
    esp_err_t e = s_real_draw(p, x0, y0, x1, y1, d);
    if (e != ESP_OK && s_flush_err++ < 3)
        ESP_LOGE(TAG, "draw_bitmap 失败 %s (%d..%d, %d..%d)", esp_err_to_name(e), x0, x1, y0, y1);
    return e;
}

/* 测试图案：8 竖彩条 + 中部棋盘格横带。逐行直写 RGB 帧缓冲（1.6KB 静态行缓冲，
 * 不占 PSRAM）。返回写入失败行数。 */
static int pattern_draw(esp_lcd_panel_handle_t panel)
{
    static uint16_t row[BSP_LCD_H_RES];
    static const uint16_t bars[8] = {
        0xffff, 0xffe0, 0x07ff, 0x07e0, 0xf81f, 0xf800, 0x001f, 0x0000
    };   /* 白 黄 青 绿 品红 红 蓝 黑（RGB565 小端原生值） */
    int bad = 0;
    for (int y = 0; y < BSP_LCD_V_RES; y++) {
        if (y >= 200 && y < 280) {
            for (int x = 0; x < BSP_LCD_H_RES; x++)
                row[x] = ((x / 25 + y / 25) & 1) ? 0xffff : 0x0000;
        } else {
            for (int x = 0; x < BSP_LCD_H_RES; x++)
                row[x] = bars[x * 8 / BSP_LCD_H_RES];
        }
        if (s_real_draw(panel, 0, y, BSP_LCD_H_RES, y + 1, row) != ESP_OK) bad++;
    }
    return bad;
}

static void pattern_restore_cb(lv_timer_t *t)
{
    (void)t;
    s_pattern_until_us = 0;
    lv_obj_invalidate(lv_screen_active());   /* 整屏失效 → LVGL 重绘恢复 UI */
}

/* 直写测试图案并停留 hold_ms 后自动恢复 UI（HTTP 触发；不阻塞调用者） */
void lcd_ui_test_pattern(int hold_ms)
{
    if (!s_panel) {
        ESP_LOGW(TAG, "面板未初始化，测试图案不可用");
        return;
    }
    if (hold_ms < 500) hold_ms = 500;
    if (hold_ms > 15000) hold_ms = 15000;
    s_pattern_until_us = esp_timer_get_time() + (int64_t)hold_ms * 1000;
    int bad = pattern_draw(s_panel);
    ESP_LOGI(TAG, "测试图案已直写面板：失败 %d 行，停留 %d ms 后恢复 UI", bad, hold_ms);
    if (s_active && bsp_display_lock(1000)) {
        lv_timer_t *tm = lv_timer_create(pattern_restore_cb, (uint32_t)hold_ms, NULL);
        if (tm) lv_timer_set_repeat_count(tm, 1);
        bsp_display_unlock();
    } else {
        s_pattern_until_us = 0;
    }
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
    s_seq |= 1u;                       /* 奇数 = 写入中 */
    strlcpy(s_status, tmp, sizeof(s_status));
    s_seq++;                           /* 偶数 = 稳定 */
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
    strlcpy(out, s_status, n);         /* 放弃一致性，给个近似值 */
}

/* 仅当文本变化才 set_text（避免 2Hz 全量重绘） */
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
    ACT_UM_PT, ACT_UM_RE,
    ACT_RES_DEC, ACT_RES_INC,
    ACT_Q_DEC, ACT_Q_INC,
    ACT_FPS_DEC, ACT_FPS_INC,
    ACT_HP_DEC, ACT_HP_INC,
    ACT_OV_TOGGLE,
    ACT_PV_TOGGLE,
    ACT_MB_DEC, ACT_MB_INC,
} act_t;

/* 把一条配置送进队列（单字段部分更新，与网页同一语义） */
static void send_json(const char *json)
{
    cfg_msg_t m;
    strlcpy(m.json, json, sizeof(m.json));
    if (xQueueSend(s_cfgq, &m, 0) != pdTRUE) {
        status_set("busy, try again");
    } else {
        status_set("queued: %s", json);
    }
}

/* 会触发链路重建的动作（res/quality/hifps）必须带当前 fps_limit：
 * app_config_apply 对未出现的字段传 0，而 cam_pipe/usb 的 apply 把 0 解读为
 * "清空软件帧率上限"——不带上就会被动清掉用户设的限帧 */
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
    if (scan_ctrl_state() == SCAN_RUNNING) {
        status_set("Locked: scan in progress");
        return;
    }
    act_t a = (act_t)(uintptr_t)lv_event_get_user_data(e);
    src_info_t *ci = src_if_info();
    bool usb = ci->source == VIDEO_SOURCE_USB;
    bool pt = usb && ci->usb_mode == USB_MODE_PASSTHROUGH;
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
    case ACT_Q_DEC:
    case ACT_Q_INC: {
        if (pt) { status_set("quality fixed by camera (passthrough)"); return; }
        int q = ci->quality;
        if (q <= 0) q = 40;
        int n;
        if (a == ACT_Q_INC) {
            n = (q / 10 + 1) * 10;
            if (n > 90) n = 10;
        } else {
            n = ((q + 9) / 10 - 1) * 10;
            if (n < 10) n = 90;
        }
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
    case ACT_HP_DEC:
    case ACT_HP_INC: {
        if (usb) return;
        int n = (cam_boost_level() + (a == ACT_HP_INC ? 1 : 5)) % 5;
        snprintf(json, sizeof(json), "{\"hifps\":%d,\"res\":\"%ux%u\"", n, ci->w, ci->h);
        pair_fps(json, sizeof(json));
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
        /* LCD 本地开关（不动流水线，与 Tab5 同设计）：关=预览任务注销帧环
         * 深度休眠——解码/缩放/翻页全停，CPU 让给主链。S31 预览消费主 MJPEG
         * 帧环，无附产流门控，故不调 src_if_pv_*（该 API 为 Tab5 专有） */
        lcd_ui_pv_set(!s_pv_on);
        if (s_preview_on) setl(s_nosig, s_pv_on ? "NO SIGNAL" : "preview off");
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
        if (scan_ctrl_state() == SCAN_RUNNING) {
            status_set("Locked: scan in progress");
            continue;
        }
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

/* ---------------- 预览：硬解 + 缩放 + 呈现 ---------------- */

/* 解码输出按 MCU 上限 32 对齐分配（防 SOF 宽高非 MCU 倍数时输出大于 w*h） */
static void ensure_out(uint32_t w, uint32_t h)
{
    size_t aw = ((size_t)w + 31) / 32 * 32;
    size_t ah = ((size_t)h + 31) / 32 * 32;
    size_t need = aw * ah * 2;
    if (s_out && s_out_cap >= need) return;
    jpeg_decode_memory_alloc_cfg_t mc = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
    size_t cap = 0;
    void *p = jpeg_alloc_decoder_mem(need, &mc, &cap);
    if (!p) {
        ESP_LOGW(TAG, "解码输出缓冲分配失败（%u B）", (unsigned)need);
        return;
    }
    if (s_out) heap_caps_free(s_out);
    s_out = p;
    s_out_cap = cap;
}

/* 硬解输出可能带 MCU 补边：从 out_size 反推行距（像素）。
 * 常见情形（SOF 已是 MCU 倍数）第一步就命中；补高 → 行距=SOF 宽；
 * 补宽 → 命中 16 像素对齐候选 */
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

/* 等比 letterbox 最近邻：src[w*h，行距 stride] → dst[PREV_W*PREV_H]（黑边居中） */
static void prev_scale(uint16_t *dst, const uint16_t *src, int w, int h, int stride)
{
    if (w < 1 || h < 1) return;
    int dw, dh;
    if ((int64_t)w * PREV_H > (int64_t)h * PREV_W) {
        dw = PREV_W; dh = (int)((int64_t)h * PREV_W / w);
    } else {
        dh = PREV_H; dw = (int)((int64_t)w * PREV_H / h);
    }
    if (dw < 1) dw = 1;
    if (dh < 1) dh = 1;
    int ox = (PREV_W - dw) / 2, oy = (PREV_H - dh) / 2;
    static uint16_t mapx[PREV_W];
    static int16_t mapy[PREV_H];
    for (int x = 0; x < dw; x++) mapx[x] = (uint16_t)((int64_t)x * w / dw);
    for (int y = 0; y < dh; y++) mapy[y] = (int16_t)((int64_t)y * h / dh);
    memset(dst, 0, (size_t)PREV_W * PREV_H * 2);
    for (int y = 0; y < dh; y++) {
        const uint16_t *srow = src + (size_t)mapy[y] * stride;
        uint16_t *drow = dst + (size_t)(oy + y) * PREV_W + ox;
        for (int x = 0; x < dw; x++) drow[x] = srow[mapx[x]];
    }
}

/* 呈现：换 src（双缓冲交替指针必变 → LVGL 必失效重绘） */
static bool present(int idx)
{
    if (esp_timer_get_time() < s_pattern_until_us) return false;  /* 图案期不覆盖 */
    if (!bsp_display_lock(500)) return false;
    lv_image_set_src(s_img, &s_dsc[idx]);
    bsp_display_unlock();
    return true;
}

static void preview_task(void *arg)
{
    (void)arg;
    const int fps_max = CONFIG_CAMTEST_LCD_PREVIEW_FPS;
    const int64_t period = fps_max > 0 ? 1000000LL / fps_max : 0;

    jpeg_decode_engine_cfg_t ecfg = {
        .intr_priority = 0,      /* 驱动默认中断优先级 */
        .timeout_ms = 500,       /* 单帧解码看门狗；>720p 硬解耗时余量充足 */
        .flags = { .allow_pd = 0 },
    };
    if (jpeg_new_decoder_engine(&ecfg, &s_jdec) != ESP_OK) {
        ESP_LOGE(TAG, "硬解引擎创建失败，预览退出（编码通路不受影响）");
        s_preview_on = false;
        vTaskDelete(NULL);
        return;
    }
    jpeg_decode_cfg_t dcfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = LCD_DEC_RGB_ORDER,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };

    SemaphoreHandle_t notify = s_pv_notify;   /* 与 PV 按钮共用（lcd_ui_start 创建） */
    frame_ring_t *reg = NULL;             /* 当前已注册的环（源切换会换环） */
    uint32_t last_fid = 0;
    float ema_ms = 0;
    int64_t next_ok = 0, win_t0 = esp_timer_get_time();
    int win_cnt = 0;

    for (;;) {
        if (!s_pv_on) {
            /* 预览关（屏上按钮）：注销帧环订阅、深睡等唤醒——解码/缩放/
             * 翻页全部停止，CPU 与内存带宽让给主链 */
            if (reg) { frame_ring_unregister(reg, notify); reg = NULL; }
            s_prev_fps = 0;
            xSemaphoreTake(notify, portMAX_DELAY);
            continue;
        }
        /* 源切换（DVP↔USB）可能更换帧环：换环重注册、帧号重置 */
        frame_ring_t *cur = src_if_ring();
        if (cur && cur != reg) {
            if (reg) frame_ring_unregister(reg, notify);
            frame_ring_register(cur, notify);
            reg = cur;
            last_fid = frame_ring_last_fid(cur);
        }

        xSemaphoreTake(notify, pdMS_TO_TICKS(300));
        int64_t now = esp_timer_get_time();
        if (now < s_pattern_until_us) {       /* 测试图案期：预览整体让位 */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (now - win_t0 >= 1000000) {    /* 1s 统计窗（无帧时也走这里归零） */
            s_prev_fps = (float)win_cnt * 1000000.0f / (float)(now - win_t0);
            win_cnt = 0;
            win_t0 = now;
        }
        if (period && now < next_ok) continue;   /* 限速：只丢不排 */

        if (reg) {
            frame_slot_t *s = frame_ring_acquire(reg, last_fid);
            if (s) {
                last_fid = s->fid;
                jpeg_decode_picture_info_t info;
                if (jpeg_decoder_get_info(s->data, (uint32_t)s->len, &info) == ESP_OK &&
                    info.width && info.height) {
                    ensure_out(info.width, info.height);
                    if (s_out) {
                        int64_t t0 = esp_timer_get_time();
                        uint32_t out_size = 0;
                        if (jpeg_decoder_process(s_jdec, &dcfg, s->data, (uint32_t)s->len,
                                                  s_out, (uint32_t)s_out_cap,
                                                  &out_size) == ESP_OK) {
                            float ms = (float)(esp_timer_get_time() - t0) / 1000.0f;
                            ema_ms += 0.15f * (ms - ema_ms);
                            s_dec_ms = ema_ms;
                            uint32_t stride = dec_stride(info.width, info.height, out_size);
                            int b = s_back;
                            prev_scale(s_pbuf[b], (const uint16_t *)s_out,
                                       (int)info.width, (int)info.height, (int)stride);
                            if (present(b)) s_back = b ^ 1;
                            s_last_frame_us = (uint32_t)esp_timer_get_time();
                            win_cnt++;
                        }
                    }
                }
                frame_ring_release(reg, s);   /* 解码完立即释放槽，缩短 refcnt 占用 */
            }
        }
        next_ok = now + period;
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

/* 选中态高亮（仅状态变化时调用，避免无谓失效） */
static void hl(lv_obj_t *b, bool on)
{
    lv_obj_set_style_bg_color(b, on ? lv_color_hex(0x2f6fd0) : lv_color_hex(0xd6e0ea),
                              LV_PART_MAIN);
    lv_obj_set_style_text_color(b, on ? lv_color_hex(0xffffff) : lv_color_hex(0x16222e),
                                LV_PART_MAIN);
}

/* 2 Hz 面板刷新：读 metrics/src/wifi/scan 快照 → 文本有变化才 set */
static void panel_timer(lv_timer_t *t)
{
    (void)t;
    /* 心跳：每 5 s 打 flush 计数（LVGL 任务活着 = flush 在完成；
     * esp_lvgl_port 对 RGB 面板每 flush 必回 ready，任务停摆即停打点） */
    static int s_hb;
    if (++s_hb >= 10) {
        s_hb = 0;
        ESP_LOGI(TAG, "hb: flush=%u err=%u prev=%.1f dec=%.1fms",
                 (unsigned)s_flush_cnt, (unsigned)s_flush_err, s_prev_fps, s_dec_ms);
    }
    if (esp_timer_get_time() < s_pattern_until_us) return;   /* 图案期不刷新 */
    src_info_t *ci = src_if_info();
    metrics_t *m = metrics_get();
    wifi_info_t *w = wifi_net_info();
    bool usb = ci->source == VIDEO_SOURCE_USB;
    bool pt = usb && ci->usb_mode == USB_MODE_PASSTHROUGH;
    char buf[128];

    snprintf(buf, sizeof(buf), "FPS  cap %.1f  enc %.1f  send %.1f",
             m->cap_fps, m->enc_fps, m->send_fps);
    setl(l_fps, buf);
    snprintf(buf, sizeof(buf), "Rate  %.2f Mbps  (%u kB)",
             m->bitrate_mbps, (unsigned)m->jpeg_avg_bytes);
    setl(l_rate, buf);
    snprintf(buf, sizeof(buf), "CPU  %.0f%% / %.0f%%  drop %u",
             m->cpu0, m->cpu1, (unsigned)(m->drop_capture + m->drop_encode));
    setl(l_cpu, buf);
    snprintf(buf, sizeof(buf), "Heap  %.1fk (min %.1fk)",
             m->free_heap / 1024.0f, m->min_heap / 1024.0f);
    setl(l_heap, buf);
    snprintf(buf, sizeof(buf), "PSRAM %.1fM free", m->free_psram / 1048576.0f);
    setl(l_psram, buf);

    if (usb) {
        if (pt) snprintf(buf, sizeof(buf), "SRC usb/%s %ux%u fw",
                         usb_state_str(), ci->w, ci->h);
        else    snprintf(buf, sizeof(buf), "SRC usb/%s %ux%u q%u",
                         usb_state_str(), ci->w, ci->h, ci->quality);
    } else {
        snprintf(buf, sizeof(buf), "SRC dvp/%s %ux%u q%u",
                 ci->sensor_name, ci->w, ci->h, ci->quality);
    }
    setl(l_src, buf);

    snprintf(buf, sizeof(buf), "WIFI %s %d dBm %s",
             w->mode == WIFI_MODE_STA_M ? "STA" : "AP", w->rssi, w->ssid);
    setl(l_wifi, buf);
    snprintf(buf, sizeof(buf), "IP %s  %s.local", w->ip, CONFIG_CAMTEST_MDNS_HOSTNAME);
    setl(l_ip, buf);

    if (s_preview_on) {
        if (s_pv_on) snprintf(buf, sizeof(buf), "LCD  %.1f fps  %.1f ms", s_prev_fps, s_dec_ms);
        else         snprintf(buf, sizeof(buf), "LCD  preview off (saved CPU)");
    } else {
        snprintf(buf, sizeof(buf), "LCD  no preview (fps=0)");
    }
    setl(l_lcd, buf);

    scan_state_t ss = scan_ctrl_state();
    if (ss == SCAN_RUNNING) {
        scan_prog_t p;
        scan_ctrl_progress(&p);
        snprintf(buf, sizeof(buf), "SCAN %d/%d %s %s",
                 p.idx, p.total, p.cur_protocol, p.cur_res);
    } else {
        snprintf(buf, sizeof(buf), "SCAN %s", ss == SCAN_DONE ? "done" : "idle");
    }
    setl(l_scan, buf);

    /* 控件值 */
    snprintf(buf, sizeof(buf), "%ux%u", ci->w, ci->h);
    setl(v_res, buf);
    if (pt) setl(v_q, "camera");
    else { snprintf(buf, sizeof(buf), "q=%u", ci->quality); setl(v_q, buf); }
    if (ci->fps_limit > 0) snprintf(buf, sizeof(buf), "max %d", ci->fps_limit);
    else                   snprintf(buf, sizeof(buf), "off");
    setl(v_fps, buf);
    if (cam_boost_level() > 0) snprintf(buf, sizeof(buf), "L%d", cam_boost_level());
    else                       snprintf(buf, sizeof(buf), "off");
    setl(v_hp, buf);
    if (m->target_mbps > 0.01f) snprintf(buf, sizeof(buf), "%.1f Mbps", m->target_mbps);
    else                        snprintf(buf, sizeof(buf), "off");
    setl(v_mb, buf);
    setl(b_ov_lab, src_if_overlay() ? "ON" : "OFF");

    /* 高亮/显隐（仅变化时操作，避免每 tick 失效重绘） */
    int st_src = usb ? 1 : 0;
    if (st_src != s_prev_src) { hl(b_dvp, !usb); hl(b_usb, usb); s_prev_src = st_src; }
    int st_um = usb ? (int)src_if_usb_mode() : -1;
    if (st_um != s_prev_um) {
        hl(b_pt, st_um == USB_MODE_PASSTHROUGH);
        hl(b_re, st_um == USB_MODE_REENCODE);
        s_prev_um = st_um;
    }
    int st_ov = src_if_overlay() ? 1 : 0;
    if (st_ov != s_prev_ov) { hl(b_ov, st_ov != 0); s_prev_ov = st_ov; }
    int st_pv = s_pv_on ? 1 : 0;
    if (st_pv != s_prev_pv) {
        hl(b_pv, st_pv != 0);
        setl(b_pv_lab, st_pv ? "PV ON" : "PV OFF");
        s_prev_pv = st_pv;
    }
    int st_vis = usb ? 1 : 0;
    if (st_vis != s_prev_vis) {
        for (int i = 0; i < 3; i++)
            lv_obj_set_hidden(w_usb[i], !st_vis);
        for (int i = 0; i < 4; i++)
            lv_obj_set_hidden(w_dvp[i], st_vis);
        s_prev_vis = st_vis;
    }

    /* 无帧 >3s 重新亮 NO SIGNAL（预览缓冲存在时）；PV 关时常显提示 */
    if (s_pbuf[0]) {
        setl(s_nosig, s_pv_on ? "NO SIGNAL" : "preview off");
        uint32_t now32 = (uint32_t)esp_timer_get_time();
        bool show = !s_pv_on || (s_last_frame_us == 0) ||
                    (uint32_t)(now32 - s_last_frame_us) > 3000000u;
        if (show != s_nosig_shown) {
            lv_obj_set_hidden(s_nosig, !show);
            s_nosig_shown = show;
        }
    }

    /* 状态行 + 颜色分级 */
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

/* 构建整屏（调用时必须已持 LVGL 锁） */
static void build_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xf2f5f9), 0);   /* 亮底：避免"工作中的暗 UI"被读成黑屏 */
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_scrollable(scr, false);

    /* 预览区（0,0,480,270） */
    lv_obj_t *pv = lv_obj_create(scr);
    lv_obj_set_pos(pv, 0, 0);
    lv_obj_set_size(pv, PREV_W, PREV_H);
    lv_obj_set_style_bg_color(pv, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(pv, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(pv, 0, 0);
    lv_obj_set_style_border_width(pv, 0, 0);
    lv_obj_set_style_radius(pv, 0, 0);
    lv_obj_set_scrollable(pv, false);
    if (s_pbuf[0]) {
        s_img = lv_image_create(pv);
        lv_obj_set_pos(s_img, 0, 0);
        lv_image_set_src(s_img, &s_dsc[0]);
    }
    s_nosig = lv_label_create(pv);
    lv_label_set_text(s_nosig, s_pbuf[0] ? "NO SIGNAL" : "preview off (Kconfig)");
    lv_obj_set_style_bg_color(s_nosig, lv_color_hex(0xffe3e3), 0);
    lv_obj_set_style_bg_opa(s_nosig, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(s_nosig, lv_color_hex(0xc0392b), 0);
    lv_obj_set_style_pad_all(s_nosig, 8, 0);
    lv_obj_set_style_radius(s_nosig, 6, 0);
    lv_obj_center(s_nosig);

    /* 左下参数两列 */
    l_fps   = mk_stat(scr, STAT_X1, STAT_Y0 + 0 * STAT_STEP, 236);
    l_rate  = mk_stat(scr, STAT_X1, STAT_Y0 + 1 * STAT_STEP, 236);
    l_cpu   = mk_stat(scr, STAT_X1, STAT_Y0 + 2 * STAT_STEP, 236);
    l_heap  = mk_stat(scr, STAT_X1, STAT_Y0 + 3 * STAT_STEP, 236);
    l_psram = mk_stat(scr, STAT_X1, STAT_Y0 + 4 * STAT_STEP, 236);
    l_src   = mk_stat(scr, STAT_X2, STAT_Y0 + 0 * STAT_STEP, 220);
    l_wifi  = mk_stat(scr, STAT_X2, STAT_Y0 + 1 * STAT_STEP, 220);
    l_ip    = mk_stat(scr, STAT_X2, STAT_Y0 + 2 * STAT_STEP, 220);
    l_lcd   = mk_stat(scr, STAT_X2, STAT_Y0 + 3 * STAT_STEP, 220);
    l_scan  = mk_stat(scr, STAT_X2, STAT_Y0 + 4 * STAT_STEP, 220);

    /* 右侧控制面板 */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "CamTest LCD Console");
    lv_obj_set_pos(title, RP_X + 8, 10);
    lv_obj_set_width(title, RP_W - 16);
    lv_obj_set_style_text_color(title, lv_color_hex(0x14202c), 0);

    mk_cap(scr, RP_X + 8, 47, "Source");
    b_dvp = mk_btn(scr, 572, 40, 100, 30, "DVP", ACT_SRC_DVP);
    b_usb = mk_btn(scr, 676, 40, 100, 30, "USB", ACT_SRC_USB);

    w_usb[0] = mk_cap(scr, RP_X + 8, 85, "USB mode");
    w_usb[1] = b_pt = mk_btn(scr, 572, 78, 100, 30, "Passthrough", ACT_UM_PT);
    w_usb[2] = b_re = mk_btn(scr, 676, 78, 100, 30, "Reencode", ACT_UM_RE);

    mk_cap(scr, RP_X + 8, 123, "Res");
    mk_btn(scr, 572, 116, 32, 30, "<", ACT_RES_DEC);
    v_res = mk_val(scr, 610, 121, 130);
    mk_btn(scr, 746, 116, 32, 30, ">", ACT_RES_INC);

    mk_cap(scr, RP_X + 8, 161, "Quality");
    mk_btn(scr, 572, 154, 32, 30, "<", ACT_Q_DEC);
    v_q = mk_val(scr, 610, 159, 130);
    mk_btn(scr, 746, 154, 32, 30, ">", ACT_Q_INC);

    mk_cap(scr, RP_X + 8, 199, "Fps max");
    mk_btn(scr, 572, 192, 32, 30, "<", ACT_FPS_DEC);
    v_fps = mk_val(scr, 610, 197, 130);
    mk_btn(scr, 746, 192, 32, 30, ">", ACT_FPS_INC);

    w_dvp[0] = mk_cap(scr, RP_X + 8, 237, "HiFps");
    w_dvp[1] = mk_btn(scr, 572, 230, 32, 30, "<", ACT_HP_DEC);
    w_dvp[2] = v_hp = mk_val(scr, 610, 235, 130);
    w_dvp[3] = mk_btn(scr, 746, 230, 32, 30, ">", ACT_HP_INC);

    mk_cap(scr, RP_X + 8, 275, "Overlay");
    b_ov = mk_btn(scr, 572, 268, 100, 30, "OFF", ACT_OV_TOGGLE);
    b_ov_lab = lv_obj_get_child(b_ov, 0);
    /* 预览开关（Tab5 同设计）：Overlay 行右侧空位，文字自描述 PV ON/OFF */
    b_pv = mk_btn(scr, 676, 268, 100, 30, "PV ON", ACT_PV_TOGGLE);
    b_pv_lab = lv_obj_get_child(b_pv, 0);

    mk_cap(scr, RP_X + 8, 313, "Mbps");
    mk_btn(scr, 572, 306, 32, 30, "<", ACT_MB_DEC);
    v_mb = mk_val(scr, 610, 311, 130);
    mk_btn(scr, 746, 306, 32, 30, ">", ACT_MB_INC);

    l_status = lv_label_create(scr);
    lv_obj_set_pos(l_status, RP_X + 8, 346);
    lv_obj_set_size(l_status, RP_W - 16, 74);
    lv_label_set_long_mode(l_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_bg_color(l_status, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_bg_opa(l_status, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(l_status, 6, 0);
    lv_obj_set_style_radius(l_status, 6, 0);
    lv_obj_set_style_text_color(l_status, lv_color_hex(0x22303c), 0);
    lv_label_set_text(l_status, "-");

    lv_obj_t *hint = lv_label_create(scr);
    lv_obj_set_pos(hint, RP_X + 8, 430);
    lv_obj_set_size(hint, RP_W - 16, 44);
    lv_label_set_text(hint, "touch changes run the same path as web /api/config (NVS saved)");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x4a5b6b), 0);

    /* 醒目横幅（左下空白区）：亮屏后一眼可辨，兼作"LCD 在工作"的目验标记 */
    lv_obj_t *banner = lv_label_create(scr);
    lv_obj_set_pos(banner, 8, 388);
    lv_obj_set_size(banner, PREV_W - 16, 80);
    lv_obj_set_style_bg_color(banner, lv_color_hex(0xffd54d), 0);
    lv_obj_set_style_bg_opa(banner, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(banner, 8, 0);
    lv_obj_set_style_text_color(banner, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_text_align(banner, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_top(banner, 22, 0);
    lv_label_set_text(banner, "LCD OK · 800x480 · CamTest");

    lv_timer_create(panel_timer, 500, NULL);
}

/* ---------------- 对外接口 ---------------- */

esp_err_t lcd_ui_start(void)
{
    if (s_active) return ESP_OK;

    /* 先探测触摸芯片：CONFIG_BSP_ERROR_CHECK=y 会让 BSP 内部错误直接 abort，
     * 未接 LCD 子板时必须在进 BSP 初始化前挡住 */
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus || i2c_master_probe(bus, GT1151_I2C_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "未探测到 GT1151 触摸（I2C 0x%02x），LCD UI 未启用", GT1151_I2C_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    /* 配置队列先于显示创建：失败可以干净退出 */
    s_cfgq = xQueueCreate(4, sizeof(cfg_msg_t));
    if (!s_cfgq) return ESP_ERR_NO_MEM;

    /* 预览双缓冲（PSRAM 259KB×2）；失败只降级为"参数面板" */
    for (int i = 0; i < 2; i++) {
        s_pbuf[i] = heap_caps_malloc((size_t)PREV_W * PREV_H * 2,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_pbuf[i]) memset(s_pbuf[i], 0, (size_t)PREV_W * PREV_H * 2);
    }
    if (s_pbuf[0] && s_pbuf[1]) {
        for (int i = 0; i < 2; i++) {
            s_dsc[i].header.cf = LV_COLOR_FORMAT_RGB565;
            s_dsc[i].header.w = PREV_W;
            s_dsc[i].header.h = PREV_H;
            s_dsc[i].data_size = (uint32_t)PREV_W * PREV_H * 2;
            s_dsc[i].data = (const uint8_t *)s_pbuf[i];
        }
        s_back = 1;                 /* img src 初始 = dsc[0]，先写另一块 */
    } else {
        if (s_pbuf[0]) { heap_caps_free(s_pbuf[0]); s_pbuf[0] = NULL; }
        if (s_pbuf[1]) { heap_caps_free(s_pbuf[1]); s_pbuf[1] = NULL; }
        ESP_LOGW(TAG, "预览缓冲分配失败，降级为参数面板");
    }

    bsp_display_cfg_t bcfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * 100,   /* 160KB×2 双缓冲（PSRAM） */
        .double_buffer = 1,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = false,
        },
    };
    bcfg.lvgl_port_cfg.task_affinity = 0;     /* 渲染任务钉 core0，让出采集/编码核 */

    /* RGB 面板的帧缓冲 DMA 链表节点必须在内部 DMA RAM（~8KB/fb），BSP 内部
     * 错误会直接 abort，这里先把内存水位打出来便于定位 */
    ESP_LOGI(TAG, "display-start前内存: internal=%u 最大块=%u dma=%u 最大块=%u psram=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* 显式走 BSP 公开 API 初始化（bsp_display_new → disp_on → lvgl_port_add_disp_rgb
     * → bsp_touch_new → lvgl_port_add_touch），与 bsp_display_start_with_config 的
     * 内部顺序一致，但保留面板句柄：测试图案直写 + draw_bitmap shim 都需要它 */
    const bsp_display_config_t bdisp = { .dummy = 0 };
    esp_lcd_panel_io_handle_t io = NULL;
    if (bsp_display_new(&bdisp, &s_panel, &io) != ESP_OK || !s_panel) {
        ESP_LOGE(TAG, "RGB 面板初始化失败");
        return ESP_FAIL;
    }
    esp_lcd_panel_disp_on_off(s_panel, true);
    s_real_draw = s_panel->draw_bitmap;
    s_panel->draw_bitmap = draw_shim;

    /* 开机测试图案（绕过 LVGL 直写帧缓冲）：彩条可见 = 面板时序/数据/背光全通；
     * 仍黑 = 面板供电/背光/接线问题，与固件显示通路无关 */
    int64_t t0 = esp_timer_get_time();
    int bad = pattern_draw(s_panel);
    ESP_LOGI(TAG, "开机测试图案: %d/480 行失败, 耗时 %.1f ms（停留 2s 供目验）",
             bad, (float)(esp_timer_get_time() - t0) / 1000.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));

    lvgl_port_cfg_t lcfg = bcfg.lvgl_port_cfg;
    if (lvgl_port_init(&lcfg) != ESP_OK) {
        ESP_LOGE(TAG, "lvgl_port_init 失败");
        return ESP_FAIL;
    }
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = s_panel,
        .buffer_size = bcfg.buffer_size,
        .double_buffer = bcfg.double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
        .flags = {
            .buff_dma = bcfg.flags.buff_dma,
            .buff_spiram = bcfg.flags.buff_spiram,
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
            .sw_rotate = bcfg.flags.sw_rotate,
        },
    };
    const lvgl_port_display_rgb_cfg_t rgb_cfg = {
        .flags = { .bb_mode = 0, .avoid_tearing = false },
    };
    lv_display_t *disp = lvgl_port_add_disp_rgb(&disp_cfg, &rgb_cfg);
    if (!disp) {
        ESP_LOGE(TAG, "LVGL 显示注册失败");
        return ESP_FAIL;
    }

    esp_lcd_touch_handle_t tp = NULL;
    if (bsp_touch_new(NULL, &tp) != ESP_OK || !tp) {
        ESP_LOGE(TAG, "触摸初始化失败");
        return ESP_FAIL;
    }
    const lvgl_port_touch_cfg_t tcfg = { .disp = disp, .handle = tp };
    if (!lvgl_port_add_touch(&tcfg)) ESP_LOGW(TAG, "触摸输入注册失败（仅显示）");

    s_active = true;
    if (!bsp_display_lock(1000)) {
        ESP_LOGE(TAG, "LVGL 锁超时，UI 未构建");
        return ESP_FAIL;
    }
    build_ui();
    bsp_display_unlock();

    if (xTaskCreatePinnedToCore(apply_task, "lcd_cfg", 8192, NULL, 5, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "配置任务创建失败");
        return ESP_ERR_NO_MEM;
    }
    s_pv_notify = xSemaphoreCreateBinary();
    if (s_pbuf[0] && s_pv_notify && CONFIG_CAMTEST_LCD_PREVIEW_FPS > 0) {
        if (xTaskCreatePinnedToCore(preview_task, "lcd_prev", 6144, NULL, 2, NULL, 1)
            == pdPASS) {
            s_preview_on = true;
        } else {
            ESP_LOGE(TAG, "预览任务创建失败，仅参数面板");
        }
    }
    status_set(s_preview_on ? "LCD ready (preview on)" : "LCD ready (panel only)");
    ESP_LOGI(TAG, "LCD UI 已启动 %dx%d，预览 %s（限速 %d fps）",
             BSP_LCD_H_RES, BSP_LCD_V_RES,
             s_preview_on ? "on" : "off", CONFIG_CAMTEST_LCD_PREVIEW_FPS);
    /* 阶段1总账（面板链表+LVGL+两任务）：与 main 的 DMA_MARK 构成启动水位链 */
    ESP_LOGI(TAG, "display-start后内存: internal=%u 最大块=%u dma=%u 最大块=%u psram=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return ESP_OK;
}

lcd_ui_stats_t lcd_ui_stats(void)
{
    lcd_ui_stats_t s = {
        .active = s_active,
        .preview_on = s_preview_on,
        .pv_on = s_pv_on,
        .preview_fps = s_prev_fps,
        .dec_ms = s_dec_ms,
        .flush_cnt = s_flush_cnt,
        .flush_err = s_flush_err,
    };
    return s;
}

void lcd_ui_pv_set(bool on)
{
    if (!s_preview_on) {
        status_set("preview task n/a");
        return;
    }
    s_pv_on = on;
    if (on && s_pv_notify) xSemaphoreGive(s_pv_notify);
    status_set(on ? "preview on" : "preview off: CPU -> main");
}

bool lcd_ui_active(void) { return s_active; }

#else  /* !CONFIG_CAMTEST_ENABLE_LCD */

esp_err_t lcd_ui_start(void) { return ESP_ERR_NOT_SUPPORTED; }
void lcd_ui_test_pattern(int hold_ms) { (void)hold_ms; }
void lcd_ui_pv_set(bool on) { (void)on; }

lcd_ui_stats_t lcd_ui_stats(void)
{
    lcd_ui_stats_t s = {0};
    return s;
}

bool lcd_ui_active(void) { return false; }

#endif /* CONFIG_CAMTEST_ENABLE_LCD */
