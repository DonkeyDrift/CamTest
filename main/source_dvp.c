/*
 * source_dvp.c — DVP 采集源后端：对既有 camera_pipeline.c 的薄适配（U1）
 *
 * camera_pipeline.c 一行逻辑未改（只增量加了 cam_pipe_stop/cam_pipe_scaled），
 * 本文件只做接口翻译，保证原通路零回归。
 */
#include "source_if.h"
#include "camera_pipeline.h"
#include <string.h>

static src_info_t  s_info;
static src_stats_t s_stats;

static void refresh_info_locked(void)
{
    cam_pipe_info_t *ci = cam_pipe_info();
    int nw = 0, nh = 0;
    bool scaled = cam_pipe_scaled(&nw, &nh);

    s_info.source = VIDEO_SOURCE_DVP;
    s_info.w = ci->w;
    s_info.h = ci->h;
    s_info.needs_encode = true;
    s_info.scaled = scaled;
    s_info.native_w = scaled ? nw : ci->w;
    s_info.native_h = scaled ? nh : ci->h;
    s_info.quality = ci->quality;
    s_info.usb_mode = USB_MODE_NONE;
    s_info.ts_meaning = TS_MEANING_SENSOR_OUT;
    s_info.nominal_fps = ci->fps;
    s_info.fps_limit = ci->fps_limit;
    memcpy(s_info.pix_fmt_str, ci->v4l2_fourcc, 4);   /* fourcc 字符串直接展示 */
    s_info.pix_fmt_str[4] = 0;
    strlcpy(s_info.sensor_name, ci->sensor_name, sizeof(s_info.sensor_name));
    s_info.usb_device_name[0] = 0;
    strlcpy(s_info.fmt_name, ci->fmt_name, sizeof(s_info.fmt_name));
}

static void refresh_stats(void)
{
    cam_pipe_stats_t *ps = cam_pipe_stats();
    s_stats.cap_frames = ps->cap_frames;
    s_stats.out_frames = ps->enc_frames;
    s_stats.cap_drops  = ps->capture_drops;
    s_stats.out_drops  = ps->encode_drops;
    s_stats.out_bytes  = ps->enc_bytes;
    s_stats.proc_acc_us = ps->enc_time_acc_us;
}

esp_err_t source_dvp_init(void)
{
    memset(&s_info, 0, sizeof(s_info));
    esp_err_t err = cam_pipe_init();
    if (err != ESP_OK) return err;
    refresh_info_locked();
    return ESP_OK;
}

esp_err_t source_dvp_start(void)
{
    cam_pipe_start();
    refresh_info_locked();
    return ESP_OK;   /* cam_pipe_start 失败经 ESP_ERROR_CHECK_WITHOUT_ABORT 打日志，链路=停止态 */
}

void source_dvp_stop(void)
{
    cam_pipe_stop();
}

esp_err_t source_dvp_apply(int w, int h, uint8_t quality, int fps_limit)
{
    esp_err_t err = cam_pipe_apply(w, h, quality, fps_limit);
    refresh_info_locked();
    refresh_stats();
    return err;
}

esp_err_t source_dvp_set_quality(uint8_t q)
{
    esp_err_t err = cam_pipe_set_quality(q);
    refresh_info_locked();
    return err;
}

void source_dvp_set_fps_limit(int fps)
{
    cam_pipe_set_fps_limit(fps);
}

void source_dvp_set_overlay(bool on)
{
    cam_pipe_set_overlay(on);
}

bool source_dvp_overlay(void)
{
    return cam_pipe_overlay();
}

src_info_t *source_dvp_info(void)
{
    refresh_info_locked();
    return &s_info;
}

src_stats_t *source_dvp_stats(void)
{
    refresh_stats();
    return &s_stats;
}

frame_ring_t *source_dvp_ring(void)
{
    return cam_pipe_ring();
}

int source_dvp_supported_res(char *out, size_t outlen)
{
    return cam_pipe_supported_res(out, outlen);
}

bool source_dvp_res_supported(int w, int h)
{
    return cam_pipe_res_supported(w, h);
}
