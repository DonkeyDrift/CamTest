/* web_ui.c — EMBED_FILES 产物符号（CMakeLists: EMBED_FILES "fs_web/index.html"） */
#include "web_ui.h"

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

int web_ui_get(const uint8_t **start, size_t *len)
{
    *start = index_html_start;
    *len = index_html_end - index_html_start;
    return 0;
}
