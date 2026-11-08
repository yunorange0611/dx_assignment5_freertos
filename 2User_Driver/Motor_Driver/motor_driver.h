//
// Created by Tessia on 2023/12/10.
//

// 头文件保护：同一个 .cpp 里重复 include 时，类不会被定义两次。
#ifndef PEPPER_PICKER_GIMBAL_MOTOR_DRIVER_H
#define PEPPER_PICKER_GIMBAL_MOTOR_DRIVER_H


#include "cmsis_os.h"

// 所有电机的公共接口。public 函数不虚，负责统一处理正反转；
// protected 的 *_forward 才是派生类要实现的“物理正方向”版本（NVI 模式）。
class motor_dev {
// protected：本类和派生类能用，外部代码不能直接调这些 *_forward。
protected:
    virtual float motor_get_speed_forward();  //返回电机转子速度 rpm 际实际值 不做归一化
    virtual float motor_get_rounds_forward(); //返回电机转子圈数 实际值 不做归一化

    virtual void motor_set_current_forward(float current);  //范围 -1～1
    virtual void motor_set_speed_forward(float speed);      //范围 -1～1
    virtual void motor_set_rounds_forward(float rounds);    //单位 转子圈数



// public：外部代码只调这一组；具体电机由派生类的 *_forward 实现。
public:
/***----------------------------------实际调用--------------------------------------**/
    float motor_get_speed();                        //返回电机转子速度 rpm 实际值 不做归一化
    float motor_get_rounds();                       //返回电机转子圈数 实际值 不做归一化

    void motor_set_current(float current);  //范围 -1～1
    void motor_set_speed(float speed);      //范围 -1～1
    void motor_set_rounds(float rounds);    //单位 转子圈数

    // 命令接口：派生类可以整体覆盖，不走上面的 public 包装。
    virtual bool motor_reset();
    virtual void motor_control(uint32_t cmd);

protected:
    // 1 = 安装方向与电机正方向相反；派生类在构造函数体里赋值。
    uint32_t reverse;    //是否反转
};



#endif //PEPPER_PICKER_GIMBAL_MOTOR_DRIVER_H
