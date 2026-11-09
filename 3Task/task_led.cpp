//
// Created by admin on 2026/11/9.
//

#include "task_led.h"
#include "bsp_led.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

void start_task_led(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();

    BSP_LED_Set(LED_BLUE, 1);            //蓝亮
    tick += 100;
    osDelayUntil(tick);
    for (;;)
    {
        BSP_LED_Toggle(LED_BLUE);             // 蓝灭
        BSP_LED_Toggle(LED_RED);              // 红亮
        tick += 100;
        osDelayUntil(tick);

        BSP_LED_Toggle(LED_RED);              // 红灭
        BSP_LED_Toggle(LED_GREEN);            // 绿亮
        tick += 100;
        osDelayUntil(tick);

        BSP_LED_Toggle(LED_GREEN);            // 绿灭
        BSP_LED_Toggle(LED_BLUE);             // 蓝亮
        tick += 100;
        osDelayUntil(tick);
    }
}
