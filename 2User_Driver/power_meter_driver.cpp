#include "power_meter_driver.h"
#include "cmsis_os.h"

// 由 CAN 中断刷新的
static volatile int16_t  g_power_raw;    // 原始值，单位 0.1 W
static volatile int16_t  g_energy_raw;   // 原始值，单位 0.1 J
static volatile uint32_t g_msg_cnt;      // 收到帧数
static volatile uint32_t g_last_tick;    // 最后收帧的时间

//  CAN 接收中断调用的
static void power_meter_callback(can_device_receive *dev, uint8_t *data)
{
    (void)dev;
    // 第1字节是低8位，第2字节是高8位
    g_power_raw  = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    g_energy_raw = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
//判断收没收到的
    g_msg_cnt++;
    g_last_tick = osKernelGetTickCount();
}

void power_meter_init(CAN_HandleTypeDef *hcan, uint32_t id)
{
    // static 用局部变量原因：让函数返回后也不会销毁一直到最后
    //注册进入can的栏
    static can_device_receive rx(hcan, id, power_meter_callback);
    (void)rx;//可以避免警告
}

float power_meter_power_w(void)
{
    return g_power_raw * 0.1f;
}

float power_meter_energy_j(void)
{
    return g_energy_raw * 0.1f;
}

uint32_t power_meter_msg_cnt(void)
{
    return g_msg_cnt;
}//返回总帧数之前一直写

bool power_meter_online(uint32_t timeout_ms)
{
    //判断有没有在线的
    return (osKernelGetTickCount() - g_last_tick) <= timeout_ms;
}
