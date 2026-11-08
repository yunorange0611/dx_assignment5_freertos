//
// Created by Tessia on 2023/12/10.
//

#include "motor_driver.h"

// 公开接口统一处理方向：reverse=1 时把用户给的设定值取反，再交给派生类的 forward 版本。
void motor_dev::motor_set_rounds(float rounds) {
    if(reverse == 1){
        motor_set_rounds_forward(-rounds);
    } else{
        motor_set_rounds_forward(rounds);
    }
}

// 和 motor_set_rounds 同一个模式：reverse=1 时取反，再调 forward；
// 下面的 motor_set_current 也一样。
void motor_dev::motor_set_speed(float speed) {
    if(reverse == 1){
        motor_set_speed_forward(-speed);
    } else{
        motor_set_speed_forward(speed);
    }
}

void motor_dev::motor_set_current(float current) {
    if(reverse == 1){
        motor_set_current_forward(-current);
    } else{
        motor_set_current_forward(current);
    }
}

// getter 同理：reverse=1 时把派生类返回的物理值取反，保证外部看到的符号一致。
float motor_dev::motor_get_rounds() {
    if(reverse == 1){
        return(-motor_get_rounds_forward());
    } else{
        return(motor_get_rounds_forward());
    }
}

// getter 也一样：forward 返回物理值，这里统一按 reverse 取反。
float motor_dev::motor_get_speed() {
    if(reverse == 1){
        return(-motor_get_speed_forward());
    } else{
        return(motor_get_speed_forward());
    }
}


/**--------------------------以下虚函数没有定义会报错-----------------------------**/
// 这些虚函数必须有定义，否则链接 vtable 时会报 undefined reference；
// 空实现让派生类可以只覆盖自己支持的接口，基类也能实例化。
float motor_dev::motor_get_speed_forward(){
    return(0);
}

float motor_dev::motor_get_rounds_forward(){
    return(0);
}

void motor_dev::motor_set_current_forward(float current){
    ;
}

// 类型是 float，不影响函数签名。
void motor_dev::motor_set_speed_forward(float current){
    ;
}

void motor_dev::motor_set_rounds_forward(float current){
    ;
}

// 默认实现：不支持复位，返回 false；DJI_motor 会重写。
bool motor_dev::motor_reset(){
    return(false);
}

// 默认空实现；DJI_motor 会按 DJI_CMD 处理。
void motor_dev::motor_control(uint32_t cmd){
    ;
}