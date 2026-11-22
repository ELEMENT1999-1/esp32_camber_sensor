#pragma once

#include "lvgl.h"
#include "tilt.h"

#ifdef __cplusplus
extern "C" {
#endif

LV_FONT_DECLARE(lv_font_cjk_16)
LV_FONT_DECLARE(lv_font_cjk_20)
LV_FONT_DECLARE(lv_font_num_100)

typedef enum {
    UI_LANG_ZH = 0,
    UI_LANG_EN = 1,
} ui_lang_t;

typedef void (*ui_btn_cb_t)(void);
typedef void (*ui_lang_cb_t)(ui_lang_t lang);

void ui_camber_create(void);
void ui_camber_set_callbacks(ui_btn_cb_t on_zero, ui_btn_cb_t on_invert, ui_lang_cb_t on_lang);
void ui_camber_set_lang(ui_lang_t lang);
ui_lang_t ui_camber_get_lang(void);
void ui_camber_update(const tilt_t *t, bool imu_ok);
/* percent: 0-100，<0 表示未接电池；charging 时显示充电标识 */
void ui_camber_set_battery(int percent, bool charging);
/* 归零/实时按钮状态：true=已归零（再按回到实时） */
void ui_camber_set_zeroed(bool zeroed);
/* 设备标题（同时作为蓝牙设备名） */
void ui_camber_set_title(const char *title);

#ifdef __cplusplus
}
#endif
