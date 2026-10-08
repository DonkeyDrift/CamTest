/*
 * frame_ring.h — 最新帧环形发布/订阅（PSRAM 缓冲 + 引用计数）
 *
 * 职责：
 *   编码任务调用 frame_ring_publish() 写入"最新一帧（JPEG 或 H.264 AU）"；
 *   每个流客户端任务 register 后用 frame_ring_acquire() 取"比自己上次新的帧"。
 *
 * 关键设计（为什么不会互相踩）：
 *   - 写者只挑选 refcnt==0 的槽位覆写（最旧优先），被客户端引用中的槽永不覆写；
 *   - 客户端把帧"发完"才释放引用；发不出（socket 慢）只影响自己，写者直接丢新帧并计数；
 *   - 队列不堆积：客户端永远只追"当前帧"，落后即跳帧 —— 时延不会累积。
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*
 * 帧级元数据：
 *   source     0=DVP 1=USB
 *   scaled     本帧是否由 ESP32 从更大原生分辨率缩放/裁剪得到
 *   ts_meaning 0=t_capture_us 为传感器输出时刻（DVP）；1=完整帧到达 ESP32 时刻（USB）
 *   codec      0=JPEG（MJPEG 语义）1=H.264（Annex-B access unit）
 */
#define FRAME_SRC_DVP            0
#define FRAME_SRC_USB            1
#define FRAME_TS_SENSOR_OUT      0
#define FRAME_TS_FRAME_ARRIVAL   1
#define FRAME_CODEC_JPEG         0
#define FRAME_CODEC_H264         1

typedef struct {
    uint8_t source;
    uint8_t scaled;
    uint8_t ts_meaning;
    uint8_t codec;
    uint8_t key;       /* H264：1=IDR/I（随机访问点）；JPEG 恒 0 */
} frame_meta_t;

typedef struct {
    uint8_t  *data;              /* PSRAM，JPEG 数据或 H.264 AU（Annex-B） */
    size_t    cap;               /* 槽容量 */
    size_t    len;               /* 本帧长度 */
    uint64_t  t_capture_us;      /* 采集打点：DVP=DQBUF 返回时刻；USB=帧到达时刻 */
    uint64_t  t_encode_done_us;  /* 编码完成时刻（esp_timer）；直通模式与 t_capture 相同 */
    uint32_t  fid;               /* 递增帧号 */
    uint16_t  w, h;
    uint8_t   quality;           /* JPEG 质量；H.264 恒 0（不可控语义，勿伪造） */
    uint8_t   key;               /* H.264：1=IDR/关键帧；JPEG 恒 0 */
    uint8_t   sensor_fmt_is_uyvy; /* 保留 */
    uint8_t   source;             /* FRAME_SRC_* */
    uint8_t   scaled;             /* 原生→输出经 ESP32 缩放 */
    uint8_t   ts_meaning;         /* FRAME_TS_* */
    uint8_t   codec;              /* FRAME_CODEC_* */
    int       refcnt;
    bool      active;
    bool      writing;           /* 发布者正在锁外拷贝此槽（消费者跳过） */
} frame_slot_t;

typedef struct frame_ring frame_ring_t;

frame_ring_t *frame_ring_create(int slots, size_t slot_cap);
void          frame_ring_destroy(frame_ring_t *r);

/* 写侧：把一帧拷入环；若无空闲槽返回 false（丢帧） */
bool frame_ring_publish(frame_ring_t *r, const uint8_t *jpeg, size_t len,
                        uint64_t t_cap, uint64_t t_enc, uint16_t w, uint16_t h, uint8_t q);
bool frame_ring_publish_ex(frame_ring_t *r, const uint8_t *jpeg, size_t len,
                           uint64_t t_cap, uint64_t t_enc, uint16_t w, uint16_t h, uint8_t q,
                           const frame_meta_t *meta);

/* 读侧：注册/注销（每个客户端一个） */
void frame_ring_register(frame_ring_t *r, SemaphoreHandle_t notify);  /* notify: 二值信号量 */
void frame_ring_unregister(frame_ring_t *r, SemaphoreHandle_t notify);

/*
 * 取 fid > last_fid 的最新帧；成功时 *out 槽 refcnt+1（用完必须 release）。
 * 无新帧返回 NULL（调用方先 wait 自己的 notify 再试）。
 */
frame_slot_t *frame_ring_acquire(frame_ring_t *r, uint32_t last_fid);
void          frame_ring_release(frame_ring_t *r, frame_slot_t *s);

uint32_t frame_ring_last_fid(frame_ring_t *r);
