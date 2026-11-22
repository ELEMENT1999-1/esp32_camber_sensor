#include "ble_link.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_link";

/* 7a1b000X-3f2c-4a8e-9d10-0e5f2a1c0001 （字节序为小端） */
static const ble_uuid128_t SVC_UUID =
    BLE_UUID128_INIT(0x01, 0x00, 0x1c, 0x2a, 0x5f, 0x0e, 0x10, 0x9d,
                     0x8e, 0x4a, 0x2c, 0x3f, 0x01, 0x00, 0x1b, 0x7a);
static const ble_uuid128_t DATA_UUID =
    BLE_UUID128_INIT(0x01, 0x00, 0x1c, 0x2a, 0x5f, 0x0e, 0x10, 0x9d,
                     0x8e, 0x4a, 0x2c, 0x3f, 0x02, 0x00, 0x1b, 0x7a);
static const ble_uuid128_t CMD_UUID =
    BLE_UUID128_INIT(0x01, 0x00, 0x1c, 0x2a, 0x5f, 0x0e, 0x10, 0x9d,
                     0x8e, 0x4a, 0x2c, 0x3f, 0x03, 0x00, 0x1b, 0x7a);

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t data_val_handle = 0;
static char dev_name[32] = "CAMBER";
static ble_cmd_cb_t cmd_cb = NULL;

static char cmd_buf[96];
static size_t cmd_len = 0;

static int gatt_svr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &DATA_UUID.u,
                .access_cb = gatt_svr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &CMD_UUID.u,
                .access_cb = gatt_svr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gap_event(struct ble_gap_event *event, void *arg);

static void adv_now(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN;
    ble_gap_adv_set_fields(&fields);

    struct ble_hs_adv_fields rsp = {0};
    rsp.name = (uint8_t *)dev_name;
    rsp.name_len = strlen(dev_name);
    rsp.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv_params, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv start rc=%d", rc);
    }
}

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_svc_gap_device_name_set(dev_name);
    adv_now();
    ESP_LOGI(TAG, "BLE sync, advertising as \"%s\"", dev_name);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected, handle=%d", conn_handle);
        } else {
            adv_now();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        adv_now();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == data_val_handle && event->subscribe.cur_notify) {
            ble_link_sendf("I,%s\n", dev_name);
        }
        return 0;
    case BLE_GAP_EVENT_CONN_UPDATE:
        return 0;
    default:
        return 0;
    }
}

static void process_cmd(const char *line)
{
    char resp[128];
    if (strncmp(line, "NAME,", 5) == 0) {
        snprintf(resp, sizeof(resp), "R,name,%.80s", line + 5);
    } else {
        snprintf(resp, sizeof(resp), "R,%.90s", line);
    }
    ESP_LOGI(TAG, "cmd: %s", line);
    if (cmd_cb) cmd_cb(line);
    ble_link_sendf("%s,OK\n", resp);
}

static int gatt_svr_access(uint16_t conn, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR &&
        attr_handle == data_val_handle) {
        static const char hello[] = "I,boot\n";
        return os_mbuf_append(ctxt->om, hello, strlen(hello));
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR &&
        attr_handle != data_val_handle) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        char tmp[64];
        if (len > sizeof(tmp) - 1) len = sizeof(tmp) - 1;
        ble_hs_mbuf_to_flat(ctxt->om, tmp, len, NULL);
        tmp[len] = '\0';
        for (uint16_t i = 0; i < len; i++) {
            if (cmd_len < sizeof(cmd_buf) - 1) {
                if (tmp[i] == '\n' || tmp[i] == '\r') {
                    if (cmd_len > 0) {
                        cmd_buf[cmd_len] = '\0';
                        process_cmd(cmd_buf);
                        cmd_len = 0;
                    }
                } else {
                    cmd_buf[cmd_len++] = tmp[i];
                }
            } else {
                cmd_len = 0;
            }
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_REGISTER_OP_CHR) {
        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &DATA_UUID.u) == 0) {
            data_val_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "data char handle=%d", data_val_handle);
        }
    }
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_link_init(const char *name)
{
    snprintf(dev_name, sizeof(dev_name), "%s", name ? name : "CAMBER");

    nimble_port_init();
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    nimble_port_freertos_init(ble_host_task);
}

void ble_link_set_name(const char *name)
{
    if (!name || !name[0]) return;
    snprintf(dev_name, sizeof(dev_name), "%s", name);
    ble_svc_gap_device_name_set(dev_name);
    /* 更新广播里的设备名（已连接时下次广播生效） */
    if (!ble_link_connected()) {
        ble_gap_adv_stop();
        adv_now();
    }
    ble_link_sendf("I,%s\n", dev_name);
}

bool ble_link_connected(void)
{
    return conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

int ble_link_sendf(const char *fmt, ...)
{
    if (!ble_link_connected()) return -1;

    char line[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) return -1;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(line, (size_t)n);
    if (!om) return -1;
    return ble_gatts_notify_custom(conn_handle, data_val_handle, om);
}

void ble_link_set_cmd_cb(ble_cmd_cb_t cb)
{
    cmd_cb = cb;
}
