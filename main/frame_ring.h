/*
 * frame_ring.h — 最新帧环形发布/订阅（PSRAM 缓冲 + 引用计数）
 *
 * 职责：
 *   编码任务调用 frame_ring_publish() 写入"最新 JPEG 帧"；
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

typedef struct {
    uint8_t  *data;              /* PSRAM，JPEG 数据 */
    size_t    cap;               /* 槽容量 */
    size_t    len;               /* 本帧 JPEG 长度 */
    uint64_t  t_capture_us;      /* DQBUF 返回时刻（esp_timer） */
    uint64_t  t_encode_done_us;  /* 编码完成时刻（esp_timer） */
    uint32_t  fid;               /* 递增帧号 */
    uint16_t  w, h;
    uint8_t   quality;
    uint8_t   sensor_fmt_is_uyvy; /* 保留 */
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
