/* stream_server.h — TCP 流服务：/stream（MJPEG）与 /ws（二进制帧） */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char type[8];    /* "mjpeg" / "ws" */
    char ip[16];
    float fps;
    float mbps;
    uint32_t frames;
} stream_client_stat_t;

esp_err_t stream_server_start(uint16_t port);
int  stream_server_get_clients(stream_client_stat_t *out, int max);
uint32_t stream_server_frames_sent(void);
uint32_t stream_server_bytes_sent(void);
esp_err_t udp_push_start(uint16_t port);

#ifdef __cplusplus
}
#endif
