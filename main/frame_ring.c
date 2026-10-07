/* frame_ring.c — 见 frame_ring.h
 *
 * 锁设计：互斥锁（非自旋锁临界区）。发布路径的 233KB PSRAM 拷贝必须在锁外：
 * 先取"writing"标记占坑 → 锁外拷贝 → 再上锁落账并广播信号量。
 * 消费者只会引用 active && !writing 的槽，绝无撕裂。
 */
#include "frame_ring.h"
#include <string.h>
#include "esp_heap_caps.h"

struct frame_ring {
    SemaphoreHandle_t lock;       /* 互斥锁保护全部元数据 */
    frame_slot_t *slots;
    int n;
    uint32_t next_fid;
    SemaphoreHandle_t notify[CONFIG_CAMTEST_MAX_STREAM_CLIENTS + 2];
    int n_notify;
    uint32_t drops;
};

frame_ring_t *frame_ring_create(int slots, size_t slot_cap)
{
    frame_ring_t *r = calloc(1, sizeof(*r));
    if (!r) return NULL;
    r->lock = xSemaphoreCreateMutex();
    r->slots = heap_caps_calloc(slots, sizeof(frame_slot_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!r->slots || !r->lock) { free(r->slots); free(r); return NULL; }
    r->n = slots;
    for (int i = 0; i < slots; i++) {
        r->slots[i].data = heap_caps_malloc(slot_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        r->slots[i].cap = slot_cap;
        if (!r->slots[i].data) { r->n = i; break; }   /* 缩容降级 */
    }
    if (r->n == 0) { free(r->slots); free(r); return NULL; }
    return r;
}

void frame_ring_destroy(frame_ring_t *r)
{
    if (!r) return;
    for (int i = 0; i < r->n; i++) free(r->slots[i].data);
    free(r->slots);
    free(r);
}

bool frame_ring_publish(frame_ring_t *r, const uint8_t *jpeg, size_t len,
                        uint64_t t_cap, uint64_t t_enc, uint16_t w, uint16_t h, uint8_t q)
{
    const frame_meta_t meta = { .source = FRAME_SRC_DVP, .scaled = 0, .ts_meaning = FRAME_TS_SENSOR_OUT };
    return frame_ring_publish_ex(r, jpeg, len, t_cap, t_enc, w, h, q, &meta);
}

bool frame_ring_publish_ex(frame_ring_t *r, const uint8_t *jpeg, size_t len,
                           uint64_t t_cap, uint64_t t_enc, uint16_t w, uint16_t h, uint8_t q,
                           const frame_meta_t *meta)
{
    /* 1) 选坑：refcnt==0 且不在写入中；优先 inactive，其次最旧 */
    frame_slot_t *victim = NULL;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    frame_slot_t *oldest = NULL;
    for (int i = 0; i < r->n; i++) {
        frame_slot_t *s = &r->slots[i];
        if (s->refcnt != 0 || s->writing) continue;
        if (!s->active) { victim = s; break; }
        if (!oldest || s->fid < oldest->fid) oldest = s;
    }
    if (!victim) victim = oldest;
    if (victim && len <= victim->cap) {
        victim->writing = true;   /* 占坑：消费者跳过 */
    } else {
        victim = NULL;
        r->drops++;
    }
    xSemaphoreGive(r->lock);
    if (!victim) return false;

    /* 2) 锁外拷贝（大块 PSRAM→PSRAM，毫秒级，绝不能持锁） */
    memcpy(victim->data, jpeg, len);

    /* 3) 落账 + 广播 */
    xSemaphoreTake(r->lock, portMAX_DELAY);
    victim->len = len;
    victim->t_capture_us = t_cap;
    victim->t_encode_done_us = t_enc;
    victim->w = w; victim->h = h; victim->quality = q;
    victim->source = meta ? meta->source : FRAME_SRC_DVP;
    victim->scaled = meta ? meta->scaled : 0;
    victim->ts_meaning = meta ? meta->ts_meaning : FRAME_TS_SENSOR_OUT;
    victim->fid = ++r->next_fid;
    victim->active = true;
    victim->writing = false;
    int n = r->n_notify;
    SemaphoreHandle_t notify[CONFIG_CAMTEST_MAX_STREAM_CLIENTS + 2];
    for (int i = 0; i < n; i++) notify[i] = r->notify[i];
    xSemaphoreGive(r->lock);
    for (int i = 0; i < n; i++) xSemaphoreGive(notify[i]);   /* 锁外 Give */
    return true;
}

void frame_ring_register(frame_ring_t *r, SemaphoreHandle_t notify)
{
    xSemaphoreTake(r->lock, portMAX_DELAY);
    if (r->n_notify < (int)(sizeof(r->notify) / sizeof(r->notify[0])))
        r->notify[r->n_notify++] = notify;
    xSemaphoreGive(r->lock);
}

void frame_ring_unregister(frame_ring_t *r, SemaphoreHandle_t notify)
{
    xSemaphoreTake(r->lock, portMAX_DELAY);
    for (int i = 0; i < r->n_notify; i++) {
        if (r->notify[i] == notify) {
            r->notify[i] = r->notify[--r->n_notify];
            break;
        }
    }
    xSemaphoreGive(r->lock);
}

frame_slot_t *frame_ring_acquire(frame_ring_t *r, uint32_t last_fid)
{
    frame_slot_t *best = NULL;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    for (int i = 0; i < r->n; i++) {
        frame_slot_t *s = &r->slots[i];
        if (s->active && !s->writing && (int32_t)(s->fid - last_fid) > 0) {
            if (!best || (int32_t)(s->fid - best->fid) > 0) best = s;
        }
    }
    if (best) best->refcnt++;
    xSemaphoreGive(r->lock);
    return best;
}

void frame_ring_release(frame_ring_t *r, frame_slot_t *s)
{
    xSemaphoreTake(r->lock, portMAX_DELAY);
    s->refcnt--;
    xSemaphoreGive(r->lock);
}

uint32_t frame_ring_last_fid(frame_ring_t *r)
{
    xSemaphoreTake(r->lock, portMAX_DELAY);
    uint32_t f = r->next_fid;
    xSemaphoreGive(r->lock);
    return f;
}
