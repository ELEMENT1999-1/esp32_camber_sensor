#include <stdio.h>
#include <string.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_mac.h>
#include <driver/gpio.h>
#include <nvs_flash.h>
#include <nvs.h>

#include "i2c_bsp.h"
#include "power_bsp.h"
#include "user_config.h"
#include "qmi8658.h"
#include "tilt.h"
#include "ble_link.h"
#include "lvgl_bsp.h"
#include "ui_camber.h"
#include "logo_img.h"

#define TAG "main"
#define NVS_NAMESPACE "camber"
#define KEY_ZERO_GPIO GPIO_NUM_10   /* 板载 +/KEY 键（Key3，电源键左侧，低有效）= 归零 */
#define KEY_PWR_GPIO  GPIO_NUM_18   /* 电源键软件检测线（Key2，低有效）= 长按 3s 关机 */
#define PWR_OFF_HOLD_US 3000000     /* 长按 3 秒关机 */

I2cMasterBus user_i2cbus(BSP_I2C_SCL, BSP_I2C_SDA, BSP_I2C_NUM);
static DisplayPort *user_display = NULL;

static qmi8658_dev_t qmi8658;
static tilt_t tilt;
static ui_lang_t g_lang = UI_LANG_ZH;
static bool g_zeroed = false;
static char g_title[32] = "CAMBER";
static volatile bool ui_dirty_zero = false;
static volatile bool ui_dirty_title = false;
static volatile bool ui_ready = false;
/* 两点校准采集状态：1=A 采集中, 2=B 采集中 */
static volatile int cal_capture_state = 0;
static volatile float cal_captured_value = 0.0f;

static void nvs_save_cal(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "offset", &tilt.offset_deg, sizeof(float));
    uint8_t inv = tilt.inverted ? 1 : 0;
    nvs_set_u8(h, "invert", inv);
    nvs_set_u8(h, "lang", (uint8_t)ui_camber_get_lang());
    uint8_t z = g_zeroed ? 1 : 0;
    nvs_set_u8(h, "zeroed", z);
    nvs_set_str(h, "title", g_title);
    nvs_set_blob(h, "cal_off", &tilt.cal_offset_deg, sizeof(float));
    nvs_set_blob(h, "cal_scale", &tilt.cal_scale, sizeof(float));
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_load_cal(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    float off = 0;
    size_t len = sizeof(float);
    if (nvs_get_blob(h, "offset", &off, &len) == ESP_OK) {
        tilt_set_offset(&tilt, off);
    }
    uint8_t inv = 0;
    if (nvs_get_u8(h, "invert", &inv) == ESP_OK) {
        tilt_invert(&tilt, inv != 0);
    }
    uint8_t lg = 0;
    if (nvs_get_u8(h, "lang", &lg) == ESP_OK) {
        g_lang = (lg == (uint8_t)UI_LANG_EN) ? UI_LANG_EN : UI_LANG_ZH;
    }
    uint8_t z = 0;
    if (nvs_get_u8(h, "zeroed", &z) == ESP_OK) {
        g_zeroed = (z != 0);
    }
    char title[32] = {0};
    size_t tl = sizeof(title);
    if (nvs_get_str(h, "title", title, &tl) == ESP_OK && title[0]) {
        snprintf(g_title, sizeof(g_title), "%s", title);
    }
    float cal_off = 0, cal_scale = 1.0f;
    size_t cl = sizeof(float);
    if (nvs_get_blob(h, "cal_off", &cal_off, &cl) == ESP_OK) {
        cl = sizeof(float);
        if (nvs_get_blob(h, "cal_scale", &cal_scale, &cl) == ESP_OK) {
            tilt_set_cal(&tilt, cal_off, cal_scale);
        }
    }
    nvs_close(h);
}

static void make_default_title(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(g_title, sizeof(g_title), "CAMBER-%02X%02X", mac[4], mac[5]);
}

static void zero_toggle(void)
{
    if (!g_zeroed) {
        tilt_set_offset(&tilt, tilt_corrected(&tilt));
        g_zeroed = true;
    } else {
        tilt_set_offset(&tilt, 0.0f);
        g_zeroed = false;
    }
    nvs_save_cal();
    ESP_LOGI(TAG, "zero toggle -> %s, offset=%.3f deg",
             g_zeroed ? "ZEROED" : "LIVE", (double)tilt.offset_deg);
}

static void on_btn_zero(void)
{
    zero_toggle();
    ui_camber_set_zeroed(g_zeroed);
}

static void on_btn_invert(void)
{
    tilt_invert(&tilt, !tilt.inverted);
    nvs_save_cal();
    ESP_LOGI(TAG, "invert=%d", (int)tilt.inverted);
}

static void on_ble_cmd(const char *line)
{
    if (strcmp(line, "ZERO") == 0) {
        zero_toggle();
        ui_dirty_zero = true;
    } else if (strcmp(line, "CAL") == 0) {
        tilt_start_bias_cal(&tilt);
    } else if (strcmp(line, "INV") == 0) {
        tilt_invert(&tilt, !tilt.inverted);
        nvs_save_cal();
    } else if (strncmp(line, "NAME,", 5) == 0 && line[5]) {
        snprintf(g_title, sizeof(g_title), "%s", line + 5);
        nvs_save_cal();
        ble_link_set_name(g_title);
        ui_dirty_title = true;
    } else if (strcmp(line, "CAL,RECA") == 0) {
        cal_capture_state = 1;
    } else if (strcmp(line, "CAL,RECB") == 0) {
        cal_capture_state = 2;
    } else if (strncmp(line, "CAL,SET,", 8) == 0) {
        float off = 0.0f, scale = 1.0f;
        if (sscanf(line + 8, "%f,%f", &off, &scale) == 2) {
            tilt_set_cal(&tilt, off, scale);
            nvs_save_cal();
            ESP_LOGI(TAG, "cal set off=%.3f scale=%.4f", (double)off, (double)scale);
        }
    } else if (strcmp(line, "CALCLR") == 0) {
        tilt_set_cal(&tilt, 0.0f, 1.0f);
        nvs_save_cal();
    }
}

static void on_btn_lang(ui_lang_t l)
{
    g_lang = l;
    nvs_save_cal();
    ESP_LOGI(TAG, "lang=%s", (l == UI_LANG_EN) ? "EN" : "ZH");
}

static lv_obj_t *splash_img = NULL;

static void splash_show(void)
{
    if (Lvgl_lock(-1) != ESP_OK) return;
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFFFFFF), 0);
    splash_img = lv_image_create(scr);
    lv_image_set_src(splash_img, &logo_dsc);
    lv_obj_center(splash_img);
    Lvgl_unlock();
    ESP_LOGI(TAG, "logo splash shown (%dx%d)", (int)logo_dsc.header.w, (int)logo_dsc.header.h);
}

static void splash_hide(void)
{
    if (Lvgl_lock(-1) != ESP_OK) return;
    if (splash_img) {
        lv_obj_delete(splash_img);
        splash_img = NULL;
    }
    Lvgl_unlock();
}

static void sensor_task(void *arg)
{
    /* KEY 键（GPIO10）：按下 = 归零/实时切换；电源键（GPIO18）：长按 3s 关机 */
    gpio_config_t key_io = {};
    key_io.pin_bit_mask = (1ULL << KEY_ZERO_GPIO) | (1ULL << KEY_PWR_GPIO);
    key_io.mode = GPIO_MODE_INPUT;
    key_io.pull_up_en = GPIO_PULLUP_ENABLE;
    key_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    key_io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&key_io);
    vTaskDelay(pdMS_TO_TICKS(50));               /* 等上拉/去抖电容稳定 */
    int key_lvl = gpio_get_level(KEY_ZERO_GPIO); /* 以当前电平为基准 */
    int key_cnt = 0;
    int64_t last_key_us = esp_timer_get_time() + 500000; /* 开机 500ms 内忽略 */
    int64_t pwr_down_us = 0;

    qmi8658_data_t data = {};
    int64_t last_us = esp_timer_get_time();
    int64_t last_ui_us = last_us;
    int64_t last_log_us = last_us;
    int64_t last_warn_us = 0;
    int64_t last_batt_us = last_us - 3000000;
    int64_t last_ble_us = last_us;
    int batt_pct = -1;
    bool batt_chg = false;
    bool data_ok = true;

    printf("time_ms,angle_deg,accel_angle_deg,offset_deg,ax_g,ay_g,az_g,gyro_x_dps,gyro_y_dps,bias_x,bias_y,battery_pct,charging\n");

    while (1) {
        /* KEY 消抖（2ms 轮询，20ms 稳定，400ms 连按间隔） */
        {
            int lvl = gpio_get_level(KEY_ZERO_GPIO);
            int64_t kn = esp_timer_get_time();
            if (lvl == key_lvl) {
                key_cnt = 0;
            } else if (++key_cnt >= 10) {
                key_lvl = lvl;
                key_cnt = 0;
                if (lvl == 0 && kn - last_key_us > 400000) {
                    last_key_us = kn;
                    zero_toggle();
                    ui_dirty_zero = true;
                    ESP_LOGI(TAG, "KEY pressed -> zero toggle");
                }
            }
        }

        /* 电源键长按 3s 关机（相对基线判定，与空闲电平极性无关） */
        {
            static int pwr_idle = -1;
            static int pwr_last = -1;
            static int64_t pwr_lvl_since = 0;
            int plvl = gpio_get_level(KEY_PWR_GPIO);
            int64_t pn = esp_timer_get_time();
            if (plvl != pwr_last) {
                pwr_last = plvl;
                pwr_lvl_since = pn;
            }
            /* 开机 3s 后取基线（用户已松开开机键），电平稳定 2s 以上即视为空闲 */
            if (pwr_idle < 0) {
                if (pn - pwr_lvl_since > 2000000 && pn > 3000000) {
                    pwr_idle = plvl;
                }
                pwr_down_us = 0;
            } else if (plvl != pwr_idle) {
                if (pwr_down_us == 0) {
                    pwr_down_us = pn;
                } else if (pn - pwr_down_us > PWR_OFF_HOLD_US) {
                    ESP_LOGW(TAG, "PWR held 3s -> power off");
                    Axp2101_PowerOff();
                }
            } else {
                pwr_down_us = 0;
            }
        }

        bool ready = false;
        if (qmi8658_is_data_ready(&qmi8658, &ready) == ESP_OK && ready) {
            if (qmi8658_read_sensor_data(&qmi8658, &data) == ESP_OK) {
                int64_t now = esp_timer_get_time();
                float dt = (float)(now - last_us) * 1e-6f;
                if (dt <= 0.0f || dt > 0.05f) dt = 0.002f;
                last_us = now;

                float ax = data.accelX * 0.001f;
                float ay = data.accelY * 0.001f;
                float az = data.accelZ * 0.001f;

                tilt_update(&tilt, ax, ay, az, data.gyroX, data.gyroY, dt);
                data_ok = true;

                /* 两点校准采集：500 拍平均后回报 */
                if (cal_capture_state != 0) {
                    static float cal_sum = 0.0f;
                    static int cal_cnt = 0;
                    if (cal_cnt == 0) cal_sum = 0.0f;
                    cal_sum += tilt.angle_deg;
                    cal_cnt++;
                    if (cal_cnt >= 500) {
                        cal_captured_value = cal_sum / (float)cal_cnt;
                        cal_cnt = 0;
                        ble_link_sendf("R,rec%c,%.3f\n",
                                       cal_capture_state == 1 ? 'a' : 'b',
                                       (double)cal_captured_value);
                        cal_capture_state = 0;
                    }
                }

                if (now - last_log_us >= 20000) {
                    last_log_us = now;
                    printf("%.1f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
                           (double)(now / 1000),
                           (double)tilt_get_angle(&tilt),
                           (double)tilt.accel_angle_deg,
                           (double)tilt.offset_deg,
                           (double)ax, (double)ay, (double)az,
                           (double)data.gyroX,
                           (double)data.gyroY,
                           (double)tilt.gyro_bias_x_dps,
                           (double)tilt.gyro_bias_y_dps,
                           batt_pct, (int)batt_chg);
                }

                if (now - last_ui_us >= 40000) {
                    last_ui_us = now;
                    if (ui_ready && Lvgl_lock(50) == ESP_OK) {
                        if (ui_dirty_zero) {
                            ui_dirty_zero = false;
                            ui_camber_set_zeroed(g_zeroed);
                        }
                        if (ui_dirty_title) {
                            ui_dirty_title = false;
                            ui_camber_set_title(g_title);
                        }
                        ui_camber_update(&tilt, tilt.gravity_ok);
                        Lvgl_unlock();
                    }
                }

                if (now - last_ble_us >= 100000) {
                    last_ble_us = now;
                    ble_link_sendf("T,%.2f,%d,%d,%d\n",
                                   (double)tilt_get_angle(&tilt), batt_pct,
                                   batt_chg ? 1 : 0, g_zeroed ? 1 : 0);
                }

                if (now - last_batt_us >= 2000000) {
                    last_batt_us = now;
                    batt_pct = Axp2101_GetBatteryPercent();
                    batt_chg = Axp2101_IsCharging();
                    if (ui_ready && Lvgl_lock(50) == ESP_OK) {
                        ui_camber_set_battery(batt_pct, batt_chg);
                        Lvgl_unlock();
                    }
                }
            }
        } else {
            int64_t now = esp_timer_get_time();
            if (data_ok && now - last_warn_us > 2000000) {
                data_ok = false;
                last_warn_us = now;
                ESP_LOGW(TAG, "QMI8658 data not ready");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    make_default_title();
    tilt_init(&tilt, 0.99f);
    nvs_load_cal();

    Custom_PmicPortInit(&user_i2cbus, 0x34);
    Axp2101_SetPowerOnLongPress();   /* 电源键长按 2s 开机（硬件上限） */

    user_display = new DisplayPort(user_i2cbus, 480, 480);
    user_display->DisplayPort_TouchInit();
    user_display->Set_Backlight(100);
    Lvgl_PortInit(*user_display);

    /* 开机 Logo（显示 1.5s，期间完成 IMU/BLE 初始化与零偏标定） */
    int64_t splash_t0 = esp_timer_get_time();
    splash_show();

    esp_err_t ret = qmi8658_init(&qmi8658, user_i2cbus.Get_I2cBusHandle(), QMI8658_ADDRESS_HIGH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize QMI8658 (error: %d)", ret);
    } else {
        qmi8658_set_accel_range(&qmi8658, QMI8658_ACCEL_RANGE_2G);
        qmi8658_set_accel_odr(&qmi8658, QMI8658_ACCEL_ODR_500HZ);
        qmi8658_set_gyro_range(&qmi8658, QMI8658_GYRO_RANGE_256DPS);
        qmi8658_set_gyro_odr(&qmi8658, QMI8658_GYRO_ODR_500HZ);
        qmi8658_set_display_precision(&qmi8658, 3);
    }

    ble_link_init(g_title);
    ble_link_set_cmd_cb(on_ble_cmd);

    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 6 * 1024, NULL, 5, NULL, 0);

    int64_t splash_ms = (esp_timer_get_time() - splash_t0) / 1000;
    if (splash_ms < 1500) {
        vTaskDelay(pdMS_TO_TICKS(1500 - splash_ms));
    }
    splash_hide();

    if (Lvgl_lock(-1) == ESP_OK) {
        ui_camber_set_callbacks(on_btn_zero, on_btn_invert, on_btn_lang);
        ui_camber_create();
        ui_camber_set_lang(g_lang);
        ui_camber_set_title(g_title);
        ui_camber_set_zeroed(g_zeroed);
        ui_camber_update(&tilt, false);
        Lvgl_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(300));   /* 等 LVGL 首帧完成后再开放 UI 刷新 */
    ui_ready = true;
}
