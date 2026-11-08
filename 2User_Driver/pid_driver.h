//
// Created by Tessia on 2023/12/10.
//

#ifndef PEPPER_PICKER_GIMBAL_PID_DRIVER_H
#define PEPPER_PICKER_GIMBAL_PID_DRIVER_H



#include "stdbool.h"
#include "main.h"
#include "cmsis_os.h"
#include "math.h"



// PID 参数包：p/i/d 是增益，max_out 是输出限幅，
// integral_higher/lower_limit 是积分上下限；input_max_err 默认 0 表示不启用。
struct pid_param {
    float p;
    float i;
    float d;
    float input_max_err;//不用
    float max_out;
    float integral_higher_limit; //积分限幅 正
    float integral_lower_limit;  //积分限幅 负
};


// 一个 PID 对象：自己保存参数、历史误差和输出。
// 构造参数顺序是 (max_out, integral_limit, kp, ki, kd)——第一个不是 kp。
class pid{

public:
    // 默认值只是能跑的配置；DJI_motor 里会显式传 velPid/posPid 的参数。
    pid(float max_out_ = 1.0,
        float integral_limit = 0.1,

        float kp = 5.0,
        float ki = 0.001,
        float kd = 0.5);

    // set_ 是目标值，get_ 是反馈值；返回限幅后的输出。
    float pid_calculate(float set_, float get_);
    // 重设增益和限幅；注意它不是“清空历史”，清积分用 pid_clear。
    void pid_reset(float max_out, float integral_limit, float kp, float ki, float kd);
    // 只清积分项 iout，不动增益和限幅。
    void pid_clear();

protected:

    bool enable;

    float set;
    float get;

    float err;
    float last_err;
    float penultimate_err;   // 预留给二阶差分，当前 pid_calculate 没用到

    float pout;
    float iout;
    float dout;


public:
    pid_param param;   // public：方便外部调参/调试
    float out;         // public：最近一次计算的输出，调用方直接读

};

#endif //PEPPER_PICKER_GIMBAL_PID_DRIVER_H
