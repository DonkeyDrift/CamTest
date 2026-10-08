/*
 * web_ui.h — 内嵌单页 Web UI（main/fs_web/index.html，EMBED_FILES 方式，离线可用）
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

int web_ui_get(const uint8_t **start, size_t *len);
