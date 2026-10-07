/*
 * camera_pipeline.h — 摄像头采集 → 硬件 JPEG 编码 → 最新帧环 发布流水线
 *
 * 三级流水（不同任务）：
 *   capture_task (高优先级, core1): DVP DQBUF → 记 t_capture_us → 换格式/拷贝进编码器输入槽 → 编码器 OUTPUT QBUF
 *   encode_task  (中优先级, core1): 编码器 CAPTURE DQBUF（内部同步触发硬件编码）→ 记 t_encode_done_us → 发布帧环
 *   （网络发送不在此模块；客户端任务从帧环自取最新帧）
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "frame_ring.h"

typedef struct {
    char     sensor_name[16];      /* 侦测到的 sensor（dev->name） */
    char     fmt_name[64];         /* 当前 sensor 格式名（如 DVP_8bit_20Minput_YUV422_UYVY_1280x720_25fps） */
    uint16_t w, h;                 /* 当前分辨率 */
    uint8_t  v4l2_fourcc[5];       /* 当前像素格式四字符码 */
    int      fps;                  /* sensor 格式标称帧率 */
    int      pclk_hz;              /* PCLK（来自格式表） */
    uint8_t  quality;              /* 当前 JPEG 质量（1..100，越大越清晰） */
    int      fps_limit;            /* 软件 fps 上限（0=不限） */
} cam_pipe_info_t;

typedef struct {
    uint32_t capture_drops;        /* 编码器输入槽忙导致的丢帧 */
    uint32_t encode_drops;         /* 帧环发布失败丢帧 */
    uint32_t cap_frames, enc_frames;
    uint64_t enc_bytes;            /* 编码输出累计字节（设备侧码率口径） */
    uint64_t enc_time_acc_us;      /* 编码耗时累计（算均值） */
} cam_pipe_stats_t;

esp_err_t cam_pipe_init(void);                       /* 含 BSP 摄像头上电 + 传感器侦测 */
void      cam_pipe_start(void);                      /* 启动采集/编码任务 */
esp_err_t cam_pipe_apply(int w, int h, uint8_t quality, int fps_limit);  /* 重建链路（可在线调用） */
esp_err_t cam_pipe_apply_vts(int w, int h, uint8_t quality, int fps_limit, int vts);  /* 兼容入口 */

/* OV3660 高帧率实验：窗口裁剪 + 时序改写（全 0 = 恢复原生表；推导见 camera_pipeline.c） */
typedef struct {
    int vts, hts, vstart, vend, hstart, hend;
} cam_boost_params_t;
esp_err_t cam_pipe_apply_boost(int w, int h, uint8_t quality, int fps_limit,
                               const cam_boost_params_t *bp);
void cam_boost_clk_set(int c303b, int c303d, int c3824);   /* 实验台：时钟树补丁（-1=不变） */
void cam_boost_apply_level(int level);                    /* 0=off, 1..4 实测标定档 */
int  cam_boost_level(void);
esp_err_t cam_pipe_set_quality(uint8_t quality);     /* 轻量：在线改 JPEG 质量，不重建链路 */
void      cam_pipe_set_fps_limit(int fps);           /* 轻量：在线改软件帧率上限 */
void      cam_pipe_stop(void);                       /* 完全停流（采集源切换用）：任务停靠 + STREAMOFF + 关 fd；帧环/队列保留 */
bool      cam_pipe_scaled(int *native_w, int *native_h);  /* 当前输出是否经抽取/裁剪得到，是则带出原生分辨率 */

frame_ring_t     *cam_pipe_ring(void);
cam_pipe_info_t  *cam_pipe_info(void);
cam_pipe_stats_t *cam_pipe_stats(void);
bool              cam_pipe_overlay(void);
void              cam_pipe_set_overlay(bool on);

/* 支持的分辨率档（来自 sensor 格式表枚举，逗号分隔 "WxH", 供 UI/扫描） */
int  cam_pipe_supported_res(char *out, size_t outlen);
/* 该分辨率是否支持（含具体 sensor 格式匹配） */
bool cam_pipe_res_supported(int w, int h);
