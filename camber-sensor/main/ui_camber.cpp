#include "ui_camber.h"

#include <stdio.h>
#include <string.h>

#define SCR_W 480
#define SCR_H 480

#define COL_BG      0x0A0A0C
#define COL_PANEL   0x16161A
#define COL_TEXT    0xF2F2F2
#define COL_DIM     0x8A8A92
#define COL_ACCENT  0xFF7A00
#define COL_OK      0x2ECC71
#define COL_WARN    0xFFB020

static lv_obj_t *lbl_title = NULL;
static lv_obj_t *lbl_angle = NULL;
static lv_obj_t *lbl_offset = NULL;
static lv_obj_t *lbl_status = NULL;
static lv_obj_t *dot_status = NULL;
static lv_obj_t *ruler = NULL;
static lv_obj_t *needle = NULL;
static lv_obj_t *btn_zero = NULL;
static lv_obj_t *btn_invert = NULL;
static lv_obj_t *btn_lang = NULL;
static lv_obj_t *lbl_batt = NULL;
static lv_obj_t *batt_frame = NULL;
static lv_obj_t *batt_fill = NULL;
static lv_obj_t *batt_tip = NULL;
static lv_obj_t *batt_bolt = NULL;

static const lv_point_precise_t bolt_pts[4] = {
    {6, 0}, {1, 7}, {5, 7}, {0, 14},
};

static bool zeroed = false;
static char title_buf[32] = "CAMBER";
static ui_lang_t lang = UI_LANG_ZH;
static float last_angle = 1e9f;
static float last_roll = 1e9f;
static bool last_imu_ok = true;

static void apply_texts(void);
static void btn_event(lv_event_t *e);

static ui_btn_cb_t cb_zero = NULL;
static ui_btn_cb_t cb_invert = NULL;
static ui_lang_cb_t cb_lang = NULL;

static lv_obj_t *make_button(lv_obj_t *parent, int x, ui_btn_cb_t cb)
{
    lv_obj_t *b = lv_button_create(parent);
    lv_obj_set_size(b, 136, 52);
    lv_obj_set_pos(b, x, SCR_H - 24 - 52);
    lv_obj_set_style_bg_color(b, lv_color_hex(COL_PANEL), 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(COL_ACCENT), LV_STATE_PRESSED);
    lv_obj_set_style_border_color(b, lv_color_hex(0x2A2A32), 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_radius(b, 10, 0);
    lv_obj_t *l = lv_label_create(b);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_text_font(l, &lv_font_cjk_20, 0);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, btn_event, LV_EVENT_CLICKED, (void *)cb);
    return b;
}

static void btn_event(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    lv_obj_t *btn = (lv_obj_t *)lv_event_get_current_target(e);
    if (btn == btn_lang) {
        lang = (lang == UI_LANG_ZH) ? UI_LANG_EN : UI_LANG_ZH;
        apply_texts();
        if (cb_lang) cb_lang(lang);
        return;
    }
    ui_btn_cb_t cb = (ui_btn_cb_t)lv_event_get_user_data(e);
    if (cb) cb();
}

void ui_camber_set_callbacks(ui_btn_cb_t on_zero, ui_btn_cb_t on_invert, ui_lang_cb_t on_lang)
{
    cb_zero = on_zero;
    cb_invert = on_invert;
    cb_lang = on_lang;
}

void ui_camber_set_lang(ui_lang_t l)
{
    lang = (l == UI_LANG_EN) ? UI_LANG_EN : UI_LANG_ZH;
    apply_texts();
}

ui_lang_t ui_camber_get_lang(void)
{
    return lang;
}

void ui_camber_set_zeroed(bool z)
{
    zeroed = z;
    apply_texts();
}

void ui_camber_set_title(const char *title)
{
    if (title && title[0]) {
        snprintf(title_buf, sizeof(title_buf), "%s", title);
    }
    if (lbl_title) {
        lv_label_set_text(lbl_title, title_buf);
    }
}

static void apply_texts(void)
{
    bool zh = (lang == UI_LANG_ZH);

    lv_label_set_text(lbl_title, title_buf);
    lv_label_set_text(lbl_status, last_imu_ok ? (zh ? "IMU 正常" : "IMU OK")
                                              : (zh ? "振动滤波" : "VIB FILTER"));
    lv_label_set_text_fmt(lbl_offset, zh ? "归零偏置 %+.2f°" : "ZERO OFFSET %+.2f°", 0.0);
    lv_label_set_text(lv_obj_get_child(btn_zero, 0),
                      zeroed ? (zh ? "实时" : "LIVE") : (zh ? "归零" : "ZERO"));
    lv_label_set_text(lv_obj_get_child(btn_invert, 0), zh ? "反向" : "INVERT");
    lv_label_set_text(lv_obj_get_child(btn_lang, 0), zh ? "中/EN" : "EN/中");
}

void ui_camber_create(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BG), 0);

    lbl_title = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_title, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_cjk_20, 0);
    lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, 16);

    dot_status = lv_obj_create(scr);
    lv_obj_set_size(dot_status, 12, 12);
    lv_obj_set_style_radius(dot_status, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot_status, lv_color_hex(COL_OK), 0);
    lv_obj_set_style_border_width(dot_status, 0, 0);
    lv_obj_align(dot_status, LV_ALIGN_TOP_LEFT, 20, 22);

    lbl_status = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_status, lv_color_hex(COL_OK), 0);
    lv_obj_set_style_text_font(lbl_status, &lv_font_cjk_16, 0);
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 40, 18);

    /* 主角度（100px 大字，含°） */
    lbl_angle = lv_label_create(scr);
    lv_label_set_text(lbl_angle, "0.00°");
    lv_obj_set_style_text_color(lbl_angle, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_text_font(lbl_angle, &lv_font_num_100, 0);
    lv_obj_align(lbl_angle, LV_ALIGN_CENTER, 0, -78);

    /* 轴角度刻度尺 -5° ~ +5°（自绘：lv_scale 线性模式不支持指针） */
    ruler = lv_obj_create(scr);
    lv_obj_set_size(ruler, 420, 84);
    lv_obj_align(ruler, LV_ALIGN_CENTER, 0, 18);
    lv_obj_set_style_bg_color(ruler, lv_color_hex(COL_PANEL), 0);
    lv_obj_set_style_bg_opa(ruler, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ruler, 0, 0);
    lv_obj_set_style_radius(ruler, 10, 0);
    lv_obj_set_style_pad_all(ruler, 0, 0);
    lv_obj_clear_flag(ruler, LV_OBJ_FLAG_SCROLLABLE);

    /* 刻度轨道：x 30..390（11 格，每格 36px） */
    static const int track_x0 = 30, track_w = 360, track_y = 40;
    lv_obj_t *track = lv_obj_create(ruler);
    lv_obj_set_size(track, track_w, 2);
    lv_obj_set_pos(track, track_x0, track_y);
    lv_obj_set_style_bg_color(track, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_border_width(track, 0, 0);
    lv_obj_set_style_radius(track, 1, 0);

    for (int i = 0; i <= 10; i++) {
        int tx = track_x0 + i * (track_w / 10);
        bool major = (i % 2 == 0);
        lv_obj_t *tick = lv_obj_create(ruler);
        lv_obj_set_size(tick, 2, major ? 14 : 8);
        lv_obj_set_pos(tick, tx - 1, track_y + 2);
        lv_obj_set_style_bg_color(tick, lv_color_hex(major ? COL_TEXT : COL_DIM), 0);
        lv_obj_set_style_border_width(tick, 0, 0);
        lv_obj_set_style_radius(tick, 1, 0);
        static const char *scale_txt[11] = {
            "-5", "-4", "-3", "-2", "-1", "0", "1", "2", "3", "4", "5"
        };
        lv_obj_t *lb = lv_label_create(ruler);
        lv_label_set_text(lb, scale_txt[i]);
        lv_obj_set_style_text_color(lb, lv_color_hex(major ? COL_TEXT : COL_DIM), 0);
        lv_obj_set_style_text_font(lb, &lv_font_cjk_16, 0);
        lv_obj_set_width(lb, 28);
        lv_obj_set_style_text_align(lb, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align_to(lb, tick, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    }

    /* 粗绿色指针（显示绕 Z 的左右倾斜） */
    needle = lv_obj_create(ruler);
    lv_obj_set_size(needle, 12, 48);
    lv_obj_set_pos(needle, track_x0 + track_w / 2 - 6, track_y + 1 - 24);
    lv_obj_set_style_bg_color(needle, lv_color_hex(COL_OK), 0);
    lv_obj_set_style_border_width(needle, 0, 0);
    lv_obj_set_style_radius(needle, 6, 0);
    lv_obj_clear_flag(needle, LV_OBJ_FLAG_SCROLLABLE);

    lbl_offset = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_offset, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(lbl_offset, &lv_font_cjk_16, 0);
    lv_obj_align(lbl_offset, LV_ALIGN_CENTER, 0, 88);

    /* 按钮行：归零/实时 | 反向 | 中/EN */
    btn_zero = make_button(scr, 20, cb_zero);
    btn_invert = make_button(scr, 172, cb_invert);
    btn_lang = make_button(scr, 324, NULL);

    /* 右上角电池 */
    lbl_batt = lv_label_create(scr);
    lv_label_set_text(lbl_batt, "--");
    lv_obj_set_style_text_color(lbl_batt, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_text_font(lbl_batt, &lv_font_cjk_16, 0);
    lv_obj_align(lbl_batt, LV_ALIGN_TOP_RIGHT, -58, 12);

    batt_frame = lv_obj_create(scr);
    lv_obj_set_size(batt_frame, 30, 16);
    lv_obj_align(batt_frame, LV_ALIGN_TOP_RIGHT, -22, 13);
    lv_obj_set_style_bg_opa(batt_frame, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(batt_frame, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_border_width(batt_frame, 1, 0);
    lv_obj_set_style_radius(batt_frame, 3, 0);
    lv_obj_set_style_pad_all(batt_frame, 0, 0);

    batt_fill = lv_obj_create(batt_frame);
    lv_obj_set_size(batt_fill, 0, 10);
    lv_obj_align(batt_fill, LV_ALIGN_LEFT_MID, 2, 0);
    lv_obj_set_style_bg_color(batt_fill, lv_color_hex(0xF2F2F2), 0);
    lv_obj_set_style_border_width(batt_fill, 0, 0);
    lv_obj_set_style_radius(batt_fill, 1, 0);

    batt_tip = lv_obj_create(scr);
    lv_obj_set_size(batt_tip, 3, 8);
    lv_obj_align(batt_tip, LV_ALIGN_TOP_RIGHT, -19, 17);
    lv_obj_set_style_bg_color(batt_tip, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_border_width(batt_tip, 0, 0);
    lv_obj_set_style_radius(batt_tip, 1, 0);

    batt_bolt = lv_line_create(scr);
    lv_line_set_points(batt_bolt, bolt_pts, 4);
    lv_obj_set_style_line_width(batt_bolt, 2, 0);
    lv_obj_set_style_line_rounded(batt_bolt, true, 0);
    lv_obj_set_style_line_color(batt_bolt, lv_color_hex(0x0E1116), 0);
    lv_obj_align(batt_bolt, LV_ALIGN_TOP_RIGHT, -34, 14);
    lv_obj_add_flag(batt_bolt, LV_OBJ_FLAG_HIDDEN);

    last_angle = 1e9f;
    last_roll = 1e9f;
    apply_texts();
}

void ui_camber_update(const tilt_t *t, bool imu_ok)
{
    char buf[96];
    bool zh = (lang == UI_LANG_ZH);

    if (imu_ok != last_imu_ok) {
        if (imu_ok) {
            lv_obj_set_style_bg_color(dot_status, lv_color_hex(COL_OK), 0);
            lv_obj_set_style_text_color(lbl_status, lv_color_hex(COL_OK), 0);
        } else {
            lv_obj_set_style_bg_color(dot_status, lv_color_hex(COL_WARN), 0);
            lv_obj_set_style_text_color(lbl_status, lv_color_hex(COL_WARN), 0);
        }
        last_imu_ok = imu_ok;
        lv_label_set_text(lbl_status, imu_ok ? (zh ? "IMU 正常" : "IMU OK")
                                             : (zh ? "振动滤波" : "VIB FILTER"));
    }

    float a = tilt_get_angle(t);
    if (a != last_angle) {
        snprintf(buf, sizeof(buf), "%+.2f°", a);
        lv_label_set_text(lbl_angle, buf);
        last_angle = a;
    }

    /* 刻度尺：绕 Z 的左右倾斜，±5°，粗绿指针 */
    float roll = tilt_get_roll(t);
    if (roll != last_roll) {
        last_roll = roll;
        float v = roll;
        if (v > 5.0f) v = 5.0f;
        if (v < -5.0f) v = -5.0f;
        /* 轨道 x 30..390 映射 -5..+5° */
        int x = 30 + (int)((v + 5.0f) * 36.0f) - 6;
        lv_obj_set_x(needle, x);
    }

    snprintf(buf, sizeof(buf), zh ? "归零偏置 %+.2f°" : "ZERO OFFSET %+.2f°",
             (double)t->offset_deg);
    lv_label_set_text(lbl_offset, buf);
}

void ui_camber_set_battery(int percent, bool charging)
{
    char buf[8];
    if (percent < 0) {
        lv_label_set_text(lbl_batt, "--");
        lv_obj_set_width(batt_fill, 0);
    } else {
        if (percent > 100) percent = 100;
        snprintf(buf, sizeof(buf), "%d%%", percent);
        lv_label_set_text(lbl_batt, buf);
        lv_obj_set_width(batt_fill, 2 + (26 * percent) / 100);
    }
    /* 充电中：绿色填充 + 闪电；否则白色填充 */
    lv_obj_set_style_bg_color(batt_fill,
                              lv_color_hex(charging ? 0x34D399 : 0xF2F2F2), 0);
    if (charging) {
        lv_obj_clear_flag(batt_bolt, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(batt_bolt, LV_OBJ_FLAG_HIDDEN);
    }
}
