#pragma once

#include "i2c_bsp.h"

void Custom_PmicPortInit(I2cMasterBus *i2cbus,uint8_t dev_addr);
void Custom_PmicRegisterInit(void);
void Axp2101_isChargingTask(void *arg);

void Axp2101_SetAldo2(uint8_t vol);
void Axp2101_SetAldo3(uint8_t vol);

/* 电池电量百分比（0-100，未接电池返回 -1）与充电状态 */
int  Axp2101_GetBatteryPercent(void);
bool Axp2101_IsCharging(void);
/* 软件关机（电源键长按 3s 触发） */
void Axp2101_PowerOff(void);
/* 电源键长按 2s 开机（PMIC 硬件最大 2s） */
void Axp2101_SetPowerOnLongPress(void);


