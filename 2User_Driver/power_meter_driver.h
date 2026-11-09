#ifndef DX_ASSIGNMENT5_FREERTOS_POWER_METER_DRIVER_H
#define DX_ASSIGNMENT5_FREERTOS_POWER_METER_DRIVER_H

#include "CAN_bsp.h"

// 真学不会自己写class算了算了
void power_meter_init(CAN_HandleTypeDef *hcan, uint32_t id);
float power_meter_power_w(void);              // 功率单位 W
float power_meter_energy_j(void);             // 缓冲能量单位 J
uint32_t power_meter_msg_cnt(void);           // 一共收到的帧数
bool power_meter_online(uint32_t timeout_ms); //有没有收到帧判断

#endif