//
// Created by admin on 2026/11/9.
//

#include "task_rc.h"
#include "bsp_rc.h"
#include "driver_rc.h"
#include "cmsis_os.h"

void Task_RC_Init(void)
{
    rc_cmd.online   = 0;
    rc_cmd.rollover = 0;
    BSP_RC_Init();          /* 启动 USART3 的 DMA + 空闲中断接收 */
}

void Task_RC_Handle(void)
{
    BSP_RC_Update();        /* 原始值 → 归一化 + 在线判定（BSP） */
    DRV_RC_Update();        /* 归一化值 → rc_cmd（Driver） */
}

void start_task_rc(void *argument)
{
    (void)argument;

    Task_RC_Init();

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        Task_RC_Handle();
        tick += 1;         
        osDelayUntil(tick);
    }
}
