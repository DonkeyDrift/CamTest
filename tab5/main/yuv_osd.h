/*
 * yuv_osd.h — 把"设备本地毫秒计数器"叠加进未编码的原始帧（光学闭环校验用）
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

typedef enum {
    OSD_FMT_UYVY,
    OSD_FMT_YUYV,
    OSD_FMT_RGB565,
    OSD_FMT_OUYEV,   /* P4 专用：O_UYY_E_VYY（YUV420 打包，奇行 UYY/偶行 VYY），jpeg 解码 YUV420 输出直供 H264 */
} osd_fmt_t;

/*
 * 在帧正中绘制 esp_timer 毫秒计数（秒.毫秒），5x7 点阵字体放大 scale 倍。
 * stride 为一行字节数（可能大于宽*bpp）。只改 luma（Y/R/G 通道），色度不动，
 * 供摄像头对准"另一块屏上显示的毫秒计时"做光学校验（见 README）。
 */
void yuv_osd_draw_ms_counter(uint8_t *buf, size_t stride, int w, int h,
                             osd_fmt_t fmt, uint64_t now_us);
