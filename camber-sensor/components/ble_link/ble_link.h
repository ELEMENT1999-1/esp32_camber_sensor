#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 蓝牙低功耗链路（NimBLE 外设模式）
 *
 * GATT 服务 7a1b0001-3f2c-4a8e-9d10-0e5f2a1c0001
 *   特征 7a1b0002... : 读/通知 —— 遥测文本行
 *     T,<angle>,<batt>,<chg>,<zeroed>\n   @10Hz
 *     I,<title>\n                        连接/改名后
 *     R,<cmd>,<OK|ERR>\n                 命令应答
 *   特征 7a1b0003... : 写 —— 命令文本行
 *     ZERO | CAL | INV | PING | NAME,<title>
 */

typedef void (*ble_cmd_cb_t)(const char *cmd_line);

void ble_link_init(const char *dev_name);
void ble_link_set_name(const char *dev_name);
bool ble_link_connected(void);
int  ble_link_sendf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void ble_link_set_cmd_cb(ble_cmd_cb_t cb);

#ifdef __cplusplus
}
#endif
