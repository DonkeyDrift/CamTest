#include "h264_pipeline.h"
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_single_hw.h"

struct h264_pipeline {
    esp_h264_enc_handle_t enc;
    esp_h264_enc_param_handle_t param;
    uint8_t *out;          /* 编码输出缓冲（≥ w*h*2，esp_h264 说明：输出不会超过输入尺寸） */
    uint32_t out_cap;
    int w, h;
};

static const char *TAG = "h264";

esp_err_t h264_pipeline_open(h264_pipeline_handle_t *out,
                             int w, int h, int fps,
                             uint32_t bitrate_kbps, uint8_t gop)
{
    esp_err_t ret = ESP_FAIL;
    h264_pipeline_handle_t hp = calloc(1, sizeof(*hp));
    ESP_RETURN_ON_FALSE(hp, ESP_ERR_NO_MEM, TAG, "no mem");
    hp->w = w; hp->h = h;

    esp_h264_enc_cfg_hw_t cfg = {
        /* ★ 本板 P4 rev1.3（<3.0）硬件编码器为 HW v3，仅接受 O_UYY_E_VYY
         * （隔行 UYY/VYY 的优化 YUV420 打包）。UYVY→该格式由 PPA 硬件完成
         * （source_usb 侧 ppa_do_scale_rotate_mirror，顺带完成缩放）。 */
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = gop,
        .fps = (uint8_t)(fps > 0 ? fps : 30),
        .res = { .width = (uint16_t)w, .height = (uint16_t)h },
        .rc = { .bitrate = bitrate_kbps * 1000, .qp_min = 18, .qp_max = 45 },
    };
    esp_h264_err_t he = esp_h264_enc_hw_new(&cfg, &hp->enc);
    ESP_GOTO_ON_FALSE(he == ESP_H264_ERR_OK, ESP_FAIL, fail, TAG, "hw_new=%d", he);
    he = esp_h264_enc_hw_get_param_hd(hp->enc, (esp_h264_enc_param_hw_handle_t *)&hp->param);
    ESP_GOTO_ON_FALSE(he == ESP_H264_ERR_OK, ESP_FAIL, fail, TAG, "get_param_hd=%d", he);
    he = esp_h264_enc_open(hp->enc);
    ESP_GOTO_ON_FALSE(he == ESP_H264_ERR_OK, ESP_FAIL, fail, TAG, "enc open=%d", he);

    hp->out_cap = (uint32_t)w * h * 2;
    /* PSRAM（32MB 充裕）：720p 输出缓冲 1.8MB */
    hp->out = heap_caps_aligned_alloc(64, hp->out_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_GOTO_ON_FALSE(hp->out, ESP_ERR_NO_MEM, fail, TAG, "out buf");

    ESP_LOGI(TAG, "H.264 HW 编码器就绪：%dx%d@%dfps gop=%u %lukbps O_UYY_E_VYY",
             w, h, fps, gop, (unsigned long)bitrate_kbps);
    *out = hp;
    return ESP_OK;
fail:
    if (hp->enc) {
        esp_h264_enc_close(hp->enc);
        esp_h264_enc_del(hp->enc);
    }
    free(hp);
    return ret;
}

esp_err_t h264_pipeline_encode(h264_pipeline_handle_t hp, const uint8_t *in_uyvy,
                               uint8_t **out_data, size_t *out_len, bool *out_idr)
{
    esp_h264_enc_in_frame_t in = {
        .raw_data.buffer = (uint8_t *)in_uyvy,
        .raw_data.len = (uint32_t)hp->w * hp->h * 3 / 2,   /* O_UYY_E_VYY = YUV420 打包 1.5B/px */
    };
    esp_h264_enc_out_frame_t outf = {
        .raw_data.buffer = hp->out,
        .raw_data.len = hp->out_cap,
    };
    esp_h264_err_t he = esp_h264_enc_process(hp->enc, &in, &outf);
    if (he != ESP_H264_ERR_OK) {
        static uint32_t enc_err;
        if (++enc_err % 30 == 1)
            ESP_LOGW(TAG, "encode fail=%d（第 %u 次）", he, (unsigned)enc_err);
        return ESP_FAIL;
    }
    *out_data = hp->out;
    *out_len = outf.length;
    /* GOP 边界出的是 I 帧（非 IDR），对解码器同样可作为随机访问点（baseline
     * 无前向参考）——浏览器 WebCodecs 的 key 块两者都收 */
    *out_idr = (outf.frame_type == ESP_H264_FRAME_TYPE_IDR ||
                outf.frame_type == ESP_H264_FRAME_TYPE_I);
    return ESP_OK;
}

esp_err_t h264_pipeline_force_idr(h264_pipeline_handle_t hp)
{
    if (!hp || !hp->param) return ESP_ERR_INVALID_STATE;
    return esp_h264_enc_force_idr(hp->param) == ESP_H264_ERR_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t h264_pipeline_set_bitrate(h264_pipeline_handle_t hp, uint32_t bitrate_bps)
{
    if (!hp || !hp->param) return ESP_ERR_INVALID_STATE;
    esp_h264_err_t he = esp_h264_enc_set_bitrate(hp->param, bitrate_bps);
    if (he != ESP_H264_ERR_OK) {
        ESP_LOGW(TAG, "set_bitrate(%lu)=%d", (unsigned long)bitrate_bps, he);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "码率自适应 → %lu kbps", (unsigned long)(bitrate_bps / 1000));
    return ESP_OK;
}

void h264_pipeline_close(h264_pipeline_handle_t hp)
{
    if (!hp) return;
    if (hp->enc) {
        esp_h264_enc_close(hp->enc);
        esp_h264_enc_del(hp->enc);
    }
    free(hp->out);
    free(hp);
}
