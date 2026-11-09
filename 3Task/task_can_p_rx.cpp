#include "task_can_p_rx.h"
#include "power_meter_driver.h"
#include "cmsis_os.h"

void start_task_p_rx(void *argument)
{
    (void)argument;

    power_meter_init(&hcan1, 0x31);//注册

    uint32_t tick = osKernelGetTickCount();
    uint32_t last_cnt = 0;

    for (;;)
    {
        // 拿到数据
        float power_w  = power_meter_power_w();
        float energy_j = power_meter_energy_j();
        uint32_t cnt   = power_meter_msg_cnt();

        // 刷新成新的一帧
        if (cnt != last_cnt)
        {
            last_cnt = cnt;
        }

       //判断掉线
        bool online = power_meter_online(300);

        tick += 10;          // 10ms 一次
        osDelayUntil(tick);
    }
}
