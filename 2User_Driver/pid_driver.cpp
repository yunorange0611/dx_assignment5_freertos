//
// Created by Tessia on 2023/12/10.
//

#include "pid_driver.h"

// 绝对值限幅：通过指针直接改调用者的变量（*a 是 a 指向的那个 float）。
void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    }
    if (*a < -abs_max) {
        *a = -abs_max;
    }
}

// 上下限不同的限幅：val_max 是上限，val_min 是下限。
void val_limit(float *a, float val_max,float val_min) {
    if (*a > val_max) {
        *a = val_max;
    }
    if (*a < val_min) {
        *a = val_min;
    }
}


// param 的 7 个值按 pid_param 声明顺序填入：p, i, d, input_max_err, max_out, 积分上限, 积分下限。
pid::pid(float max_out_, float integral_limit, float kp, float ki, float kd)
    :param{kp, ki, kd, 0, max_out_, integral_limit, -integral_limit},//结构体初始化格式
     enable(true),
     set(0),get(0),
     err(0),last_err(0),penultimate_err(0),
     pout(0),iout(0),dout(0),out(0){ }

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output   0-outmax
  */ //0-outmax已限幅
// __RAM_FUNC：把这个函数放到 RAM 里跑，避开 Flash 等待周期（链接脚本要支持 .RamFunc）。
__RAM_FUNC
float pid::pid_calculate(float set_, float get_) {
    get = get_;
    set = set_;
    // 误差 = 目标 - 反馈；后面的 P/I/D 都基于它。
    err = set - get;
    if (fabsf(param.input_max_err) > 1e-6)
        abs_limit(&err, param.input_max_err);
    pout = param.p * err;
    // 积分累加：没有乘 dt，同样的 ki 在 1kHz 和 500Hz 下效果不同。
    iout += param.i * err;
//    pid->iout += pid->param.i * abs_zero(pid->err,0.1);
    // 微分：误差变化量；没有除 dt，目标突变时会有微分冲击。
    dout = param.d * (err - last_err);

//    abs_limit(&(pid->iout), pid->param.integral_limit);
    val_limit(&iout, param.integral_higher_limit,param.integral_lower_limit);
    // 三项相加后再做输出限幅；积分项上面已经单独限过。
    out = pout + iout + dout;
    abs_limit(&out, param.max_out);
    last_err = err;

    if (enable == 0) {
        out = 0;
    }
    return(out);
}

// 重设增益/限幅：负数表示“这一项不改”；ki≈0 时清积分。
void pid::pid_reset(float max_out, float integral_limit, float kp, float ki, float kd) {
    if (kp >= 0)param.p = kp;
    if(fabsf(ki) <= 1e-6){
        param.i = 0;
        iout = 0;
    }
    else if (ki > 0)
        param.i = ki;
    if (kd >= 0)param.d = kd;
    if(max_out > 0) param.max_out = max_out;
    if(integral_limit > 0){
        param.integral_higher_limit = integral_limit;
        param.integral_lower_limit = -integral_limit;
    }else{
        param.integral_higher_limit = -integral_limit;
        param.integral_lower_limit = integral_limit;
    }

}

// 只清积分，不动增益和历史误差。
void pid::pid_clear() {
    iout = 0;
}