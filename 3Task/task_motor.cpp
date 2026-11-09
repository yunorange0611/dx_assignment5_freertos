#include "task_motor.h"
#include "DJI_motor_driver.h"   // DJI_motor、DJI_M3508
#include "cmsis_os.h"
#include <math.h>               // cosf

void start_task_motor(void *argument)
{
    (void)argument;


    static DJI_motor motor(&hcan1,nullptr,1,DJI_M3508, 0);
    // nullptr的意思是用模板默认的反馈解析回调，注册的函数有了所以不用像刚刚那样写一个了

    const float PI = 3.1415926f;
    const float w  = 2.0f * PI * 0.5f;   // f=0.5 Hz
    const float v_max_rpm = 100.0f;      // 速度峰值100rpm/min

    uint32_t tick = osKernelGetTickCount();
    float t = 0.0f;

    for (;;)
    {
        //v(t) = v_amp * sin(wt)，位置是速度的积分：θ(t) = ∫ (A/60)·sin(w·t) dt= A/(60·w) · (1 − cos(w·t))
        float target_rounds = v_max_rpm / (60.0f * w) * (1.0f - cosf(w * t));

        motor.motor_set_rounds(target_rounds);   // 这个已经pid写好了

        t += 0.001f;
        if (t > 2.0f * PI / w) {
            t -= 2.0f * PI / w;
        }//可以避免t累积的很大

        tick += 1;                               // 1ms 周期的固定回调
        osDelayUntil(tick);
    }
}
