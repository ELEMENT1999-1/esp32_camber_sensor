idf_component_register(
    SRCS "main.cpp" "ui_camber.cpp" "logo_img.c" "lv_font_cjk_16.c" "lv_font_cjk_20.c" "lv_font_num_100.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash
    PRIV_REQUIRES
    driver
    esp_timer
    pmicpower
    port_bsp
    app_bsp
    qmi8658
    tilt
    ble_link
    lvgl__lvgl
    espressif__esp_lvgl_adapter)
