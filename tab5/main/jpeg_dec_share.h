/*
 * jpeg_dec_share.h — P4 硬件 JPEG 解码器全局共享（单例互斥）
 *
 * P4 的 JPEG 解码外设是 SoC 单例：source_usb 的 h264/reencode 管线与
 * lcd_ui 的预览都要用它。S31 上靠 BSP 的 codec_mutex 在"编码空闲窗"抢；
 * P4 上编码器与解码器是独立外设（可并行），争用只在【解码器】使用者之间。
 *
 * 语义：
 *   - 惰性创建引擎（首个使用者），永不销毁；
 *   - process 内部持互斥（优先级继承），一次一帧；
 *   - 超时 500ms（720p 硬解 <10ms，余量充足）。
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/jpeg_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 解码一帧（阻塞直到完成或超时）。out 必须来自 jpeg_alloc_decoder_mem。 */
esp_err_t jpeg_dec_share_process(const jpeg_decode_cfg_t *cfg,
                                 const uint8_t *in, uint32_t in_len,
                                 uint8_t *out, uint32_t out_cap,
                                 uint32_t *out_size);

#ifdef __cplusplus
}
#endif
