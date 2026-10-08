/*
 * h264_pipeline.h — ESP32-P4 硬件 H.264 编码器封装（esp_h264 1.4.x HW single stream）
 *
 * 数据流（source_usb reencode-h264 路径）：
 *   MJPEG → esp_driver_jpeg 硬件解码（UYVY 输出）→ 本模块 encode()（UYVY 输入）
 *   → Annex-B access unit（IDR 帧自动前置 SPS/PPS）→ 帧环 → WS → 浏览器 WebCodecs
 *
 * 延迟要点：baseline 无 B 帧，process() 同步出帧（1 帧 in-flight）；
 * GOP 内 P 帧极小，码率由 rc.bitrate 控制（driver 内部 CBR 语义）。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct h264_pipeline *h264_pipeline_handle_t;

/* 创建并打开编码器（UYVY 输入，尺寸需 16 对齐——调用方保证） */
esp_err_t h264_pipeline_open(h264_pipeline_handle_t *out,
                             int w, int h, int fps,
                             uint32_t bitrate_kbps, uint8_t gop);

/* 编码一帧：in_buf 为 w*h*3/2 字节（O_UYY_E_VYY，由 PPA 从 UYVY 硬件转换而来）；
 * 成功时 *out_len 与 *out_idr 有效，数据在内部输出缓冲
 * （下次 encode 前有效——发布者需在本调用返回后立即拷走，语义与单 worker 串行匹配） */
esp_err_t h264_pipeline_encode(h264_pipeline_handle_t hp, const uint8_t *in_uyvy,
                               uint8_t **out_data, size_t *out_len, bool *out_idr);

/* 运行时改码率（码率自适应 governor 用；bps） */
esp_err_t h264_pipeline_set_bitrate(h264_pipeline_handle_t hp, uint32_t bitrate_bps);

/* 强制下一帧 IDR（开流首帧调用：浏览器一连上就有 SPS/PPS 可立即起播） */
esp_err_t h264_pipeline_force_idr(h264_pipeline_handle_t hp);

void h264_pipeline_close(h264_pipeline_handle_t hp);

#ifdef __cplusplus
}
#endif
