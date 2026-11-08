/**
  ******************************************************************************
  * @file           : DJI_motor_driver.cpp
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
  */



#include "DJI_motor_driver.h"



typedef struct dji_can_tx {
    CAN_HandleTypeDef *can_handle;
    uint16_t id;
    uint8_t enable;
    uint8_t buf[8];
}dji_motor_can_tx_t;

// 6 个发送组：CAN1/CAN2 × 0x200/0x1FF/0x2FF。
// 每组 8 字节，最多装 4 个电机；enable 由电机构造函数置 1。
dji_motor_can_tx_t can_tx_list[] = {{&hcan1, 0x200, 0,},
                                    {&hcan1, 0x1ff, 0,},
                                    {&hcan1, 0x2ff, 0,},
                                    {&hcan2, 0x200, 0,},
                                    {&hcan2, 0x1ff, 0,},
                                    {&hcan2, 0x2ff, 0,}
};

void dji_motor_data_update_callback(can_device_receive *can_receive, uint8_t *data);

// 构造顺序按成员声明，不按初始化列表：can_rx 先注册过滤器，can_tx 再选发送组，PIDs 最后构造。
DJI_motor::DJI_motor(CAN_HandleTypeDef *hcan_, can_rx_callback *callback_,
                     uint32_t id_motor, DJI_MOTOR_TYPE type_, uint8_t is_reverse)
                     :can_rx_data{},motor_data{},
                      type(type_),current_to_send(0),bias_ordinal(0),is_use_external_speed(0),
                      cmd(DJI_MOTOR_RESET_OFFSET),init_offset_f(false),is_id_false(false),

                      // 反馈 ID：3508/2006 = 0x200+id；6020 = 0x204+id。
                      can_rx(hcan_,(type_==DJI_M3508||type_==DJI_M2006)?0x200+id_motor:0x204+id_motor,callback_),
                      // 发送组：3508/2006 的 ID1~4→0x200、ID5~8→0x1FF；6020 的 ID1~4→0x1FF、ID5~7→0x2FF。
                      can_tx(hcan_,(type_==DJI_M3508||type_==DJI_M2006)?(id_motor<=4?0x200:0x1ff):(id_motor<=4?0x1ff:0x2ff)),
                      // velPid(max_out, integral_limit, kp, ki, kd)
                      velPid(0.5f, 0.1f, 10, 0.002f, 0.1f),
                      // posPid(max_out, integral_limit, kp, ki, kd)
                      posPid(0.2f,0.1f,0.3f,0.f,0.f) {

    // reverse 是基类 protected 成员，不能在派生类初始化列表里初始化，只能在函数体赋值。
    reverse = is_reverse; //父类继承的protected似乎无法在列表中初始化

    // 只接受 ID 1~8；GM6020 的 ID8 非法。
    id_motor>0&&id_motor<9 ? id = id_motor : is_id_false = 1;
    type==DJI_GM6020?(id_motor==8?is_id_false=1:id=id_motor):id=id_motor; //判断6020id是否为8

    callback_ == nullptr ? can_rx.can_set_callback(dji_motor_data_update_callback) : can_rx.can_set_callback(callback_);


    // 等 50 帧反馈后才继续构造；如果电机没接，这里会一直 osDelay(1)。
    while (init_offset_f!=1){
        osDelay(1);//在这里停止
    }

    dji_motor_can_tx_t *tx;
    if(hcan_==&hcan1){
        if(can_tx.member.id == 0x200){
            tx = &can_tx_list[0];
            can_tx_data = can_tx_list[0].buf;
        } else if(can_tx.member.id == 0x1ff){
            tx = &can_tx_list[1];
            can_tx_data = can_tx_list[1].buf;
        }else{
            tx = &can_tx_list[2];
            can_tx_data = can_tx_list[2].buf;
        }
    }else{
        if(can_tx.member.id == 0x200){
            tx = &can_tx_list[3];
            can_tx_data = can_tx_list[3].buf;
        } else if(can_tx.member.id == 0x1ff){
            tx = &can_tx_list[4];
            can_tx_data = can_tx_list[4].buf;
        }else{
            tx = &can_tx_list[5];
            can_tx_data = can_tx_list[5].buf;
        }
    }
    tx->enable = 1;
}



/*
 * 获取解算电机数据
 */
// 从 can_rx 成员地址反推 DJI_motor 对象，再把 8 字节解析成编码器/转速/电流。
void dji_motor_data_update_callback(can_device_receive *can_receive, uint8_t *data) {
    DJI_motor *dji_motor = container_of(can_receive, DJI_motor, can_rx);
    memcpy(dji_motor->can_rx_data,data,8);
    dji_motor->motor_data.msg_cnt++;
    if (dji_motor->motor_data.msg_cnt > 50)
    {
        dji_motor->init_offset_f = 1;
    }
    /* initial value */
    if (dji_motor->init_offset_f == 0)
    {
        // 大端：高字节在前；角度用 uint16，转速/电流用 int16。
        dji_motor->motor_data.ecd = (uint16_t)(dji_motor->can_rx.rx_data[0] << 8 | dji_motor->can_rx.rx_data[1]);
        dji_motor->motor_data.offset_ecd = dji_motor->motor_data.ecd;
        return;
    }

    dji_motor->motor_data.last_ecd = dji_motor->motor_data.ecd;
    dji_motor->motor_data.ecd = (uint16_t)(dji_motor->can_rx.rx_data[0] << 8 | dji_motor->can_rx.rx_data[1]);

    // 过零点：两次采样差超过半圈，就认为跨了 0/8191，圈数相应加/减。
    if (dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd > 4096)
    {
        dji_motor->motor_data.round_cnt--;
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd - 8192;
    }
    else if (dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd < -4096)
    {
        dji_motor->motor_data.round_cnt++;
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd + 8192;
    }
    else
    {
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd;
    }

    dji_motor->motor_data.total_ecd = dji_motor->motor_data.round_cnt * 8192 + dji_motor->motor_data.ecd - dji_motor->motor_data.offset_ecd;
    dji_motor->motor_data.total_round = (float)(dji_motor->motor_data.total_ecd) * ENCODER_TO_ROUND;
    dji_motor->motor_data.speed_rpm = (int16_t)(dji_motor->can_rx.rx_data[2] << 8 | dji_motor->can_rx.rx_data[3]);
    dji_motor->motor_data.given_current = (int16_t)(dji_motor->can_rx.rx_data[4] << 8 | dji_motor->can_rx.rx_data[5]);
}


// 把 current_to_send 写到组缓冲的 [index-1, index] 两字节，高字节在前（大端）。
void DJI_motor::dji_motor_can_tx_write() {
    // ID1~4 和 ID5~8 映射到同一帧的 4 个两字节槽位：1/5→[0,1]，2/6→[2,3]...
    uint8_t index = (id < 5 ? id : id - 4) * 2 - 1;
    taskENTER_CRITICAL();
    // 大端：index-1 放高 8 位，index 放低 8 位
    can_tx_data[index] = (uint8_t)(current_to_send&0xFF);
    can_tx_data[index - 1] = (uint8_t)(current_to_send>>8);
    taskEXIT_CRITICAL();
}


/**
 * 获取电机速度
 * @param void
 * @return 电机实际速度 单位rpm
 */
float DJI_motor::motor_get_speed_forward() {
    if(is_use_external_speed == 1){
        return *motor_data.external_speed;
    } else{
        return (float)motor_data.speed_rpm;
    }
}

void DJI_motor::motor_change_speed_source(float *speed) {
    is_use_external_speed = 1;
    motor_data.external_speed = speed;
}


/**
 * 获取电机位子
 * @param motor
 * @return 电机计入过零点检测的圈数
 */
float DJI_motor::motor_get_rounds_forward() {
    return motor_data.total_round;
}


/**
 * 设置电机电流
 * @param motor
 * @param current_normal 范围 -1～1
 */
void DJI_motor::motor_set_current_forward(float current) {
    // 输入是 -1~1 的归一化电流，先限幅，防止乘最大电流后溢出 int16
    current = current > 1.0f ? 1.0f : (current < -1.0f ? -1.0f : current);
    switch(type){
        case DJI_M3508:
            current_to_send=(int16_t)(current * MOTOR_3508_MAX_CURRENT);
            break;
        case DJI_M2006:
            current_to_send=(int16_t)(current * MOTOR_2006_MAX_CURRENT);
            break;
        case DJI_GM6020:
            current_to_send=(int16_t)(current * MOTOR_6020_MAX_CURRENT);
            break;
    }
    dji_motor_can_tx_write();
}


//speed=0.05
// 速度环：目标 speed 和反馈都归一化到 ±1，PID 输出再作为电流指令。
void DJI_motor::motor_set_speed_forward(float speed) {
    switch(type){
        case DJI_M3508:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_3508_MAX_SPEED);
            break;
        case DJI_M2006:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_2006_MAX_SPEED);
            break;
        case DJI_GM6020:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_6020_MAX_SPEED);
            break;
    }
    motor_set_current_forward(velPid.out);
}


// 位置环：目标圈数和 total_round 比较，输出归一化速度，再交给速度环。
void DJI_motor::motor_set_rounds_forward(float rounds) {
    posPid.pid_calculate(rounds, (float)(motor_get_rounds_forward()));
    motor_set_speed_forward(posPid.out);
}


// 把当前编码器位置设为新的零点，并清空圈数。
bool DJI_motor::motor_reset() {
    motor_data.offset_ecd = motor_data.ecd;
    motor_data.total_ecd = 0;
    motor_data.total_round = 0;
    motor_data.round_cnt = 0;
    return(true);
}


// 统一命令入口：清 PID、复位偏置、6020 的绝对零位调整、加减一圈偏置。
void DJI_motor::motor_control(uint32_t cmd_) {
    cmd = (DJI_CMD)cmd_;
    switch (cmd) {
        case DJI_MOTOR_CLEAR_PID:
            velPid.pid_clear();
            posPid.pid_clear();
            break;
        case DJI_MOTOR_RESET_OFFSET:
            motor_reset();
            break;
        case DJI_MOTOR_DISABLE_OFFSET:
            motor_data.offset_ecd = 0;
            break;
        case DJI_MOTOR_ADD_ONE_LAP_OFFSET:
            motor_data.offset_ecd = motor_data.offset_ecd + ENCODER_MAX;
            break;
        case DJI_MOTOR_SUB_ONE_LAP_OFFSET:
            motor_data.offset_ecd = motor_data.offset_ecd - ENCODER_MAX;
            break;
    }
}


// 1 ms 周期把 6 个使能组的缓冲各入队一次。
// 队列按值拷贝，局部 msg 可以复用；buf_data 指向全局的 can_tx_list[i].buf。
void DJI_motor_service(void *argument) {
    uint32_t tick = osKernelGetTickCount();
    can_device_transmit msg;
    for(;;){
        for(int i = 0;i<6;i++){
            if(can_tx_list[i].enable){
                msg.member.hcan = can_tx_list[i].can_handle;
                msg.member.id = can_tx_list[i].id;
                msg.member.len = 8;
                msg.member.buf_data = can_tx_list[i].buf;
                msg.can_send_msg();
            }
        }
        tick+=1;
        osDelayUntil(tick);
    }

}
