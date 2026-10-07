/* yuv_osd.c — 5x7 点阵数字叠加，见 yuv_osd.h */
#include "yuv_osd.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

/* 5x7 字体：'0'-'9' 与 '.'（列优先，bit0 为最上行） */
static const uint8_t font5x7[11][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x30,0x30,0x00,0x00}, /* . */
};

static inline void put_px(uint8_t *buf, size_t stride, int w, int h, osd_fmt_t fmt,
                          int x, int y, uint8_t v)
{
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    if (fmt == OSD_FMT_RGB565) {
        uint16_t *p = (uint16_t *)(buf + y * stride) + x;
        /* 灰阶 v(0..255) → RGB565（小端原样写，仅提亮） */
        uint16_t g = ((v >> 2) & 0x1F) << 5 | ((v >> 3) & 0x07);
        g = ((v >> 3) << 11) | ((v >> 2 & 0x1F) << 5) | (v >> 3);
        *p = __builtin_bswap16(g); /* 传感器 RGB565 为 BE */
    } else {
        /* YUYV/UYVY：每像素 2 字节，Y 在偶/奇字节，直接覆盖 Y */
        buf[y * stride + x * 2 + (fmt == OSD_FMT_UYVY ? 1 : 0)] = v;
    }
}

void yuv_osd_draw_ms_counter(uint8_t *buf, size_t stride, int w, int h,
                             osd_fmt_t fmt, uint64_t now_us)
{
    char text[40];
    uint64_t ms = now_us / 1000;
    snprintf(text, sizeof(text), "%llu.%03llu", (unsigned long long)(ms / 1000),
             (unsigned long long)(ms % 1000));
    int scale = (w >= 640) ? 4 : (w >= 320) ? 3 : 2;
    int cx = 4, cy = 4;
    for (const char *p = text; *p; p++) {
        int gi = (*p == '.') ? 10 : (*p - '0');
        if (gi < 0 || gi > 10) { cx += scale; continue; }
        const uint8_t *glyph = font5x7[gi];
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if (glyph[col] & (1 << row)) {
                    for (int dy = 0; dy < scale; dy++)
                        for (int dx = 0; dx < scale; dx++)
                            put_px(buf, stride, w, h, fmt, cx + col * scale + dx,
                                   cy + row * scale + dy, 235);
                }
            }
        }
        cx += 6 * scale;
    }
}
