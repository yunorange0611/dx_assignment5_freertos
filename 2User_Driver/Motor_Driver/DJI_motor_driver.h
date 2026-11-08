/**
  ******************************************************************************
  * @file           : DJI_motor_driver.h
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
  */

#ifndef PEPPER_PICKER_GIMBAL_DJI_MOTOR_DRIVER_H
#define PEPPER_PICKER_GIMBAL_DJI_MOTOR_DRIVER_H

//C

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "motor_driver.h"
#include "CAN_bsp.h"
#include "pid_driver.h"
#include "string.h"
#include "main.h"


const float MOTOR_6020_MAX_SPEED = 320.0f;
const float MOTOR_3508_MAX_SPEED = 9600.0f;
const float MOTOR_2006_MAX_SPEED = 21000.0f;
const float ENCODER_ANGLE_RATIO = 22.755555555555556f;     //(8192.0f / 360.0f)
const float ENCODER_TO_ANGLE_RATIO = 0.0439453125f;        // 360/8192.0
const float ENCODER_MAX = 8192.0f;
const float ENCODER_TO_ROUND = 1.0f/8192.0f;
const float MOTOR_6020_MAX_CURRENT = 25000;                //25000 for 6020
const float MOTOR_3508_MAX_CURRENT = 16384;                //16384 for M3508
const float MOTOR_2006_MAX_CURRENT = 10000;

// 电机类型决定反馈 ID、发送组、最大转速和最大电流。
enum DJI_MOTOR_TYPE {
    DJI_M3508 = 0, //默认为
    DJI_M2006 = 1,
    DJI_GM6020 = 2, ///推荐id为5、6、7
};
// 统一命令码：motor_control(cmd) 传进来，DJI_motor 在 .cpp 里 switch。
enum DJI_CMD {
    DJI_MOTOR_CLEAR_PID = 200,
    DJI_MOTOR_RESET_OFFSET,
    DJI_MOTOR_DISABLE_OFFSET,   //适用于6020，因为其为绝对值编码器
    DJI_MOTOR_ADD_ONE_LAP_OFFSET,    //适用于初始化过零点问题，加一圈偏置
    DJI_MOTOR_SUB_ONE_LAP_OFFSET,    //适用于初始化过零点问题，减一圈偏置
};

// 一个电机的全部反馈状态：ecd 是单圈编码器值，round_cnt 累计过零点圈数，
// total_round 是相对 offset_ecd 的转子总圈数，speed_rpm/current 直接来自反馈帧。
struct dji_motor_data {
    uint16_t last_ecd;
    uint16_t ecd;           //转子机械角度 编码器值
    int16_t speed_rpm;      //转速
    int16_t given_current;  //实际电流

    float *external_speed;  //外置速度  该速度需用户自己更新

    int32_t round_cnt;      //圈数计算
    int32_t total_ecd;      //总机械角

    float total_round;      //总圈数
    int32_t ecd_raw_rate;   //差值
    uint32_t msg_cnt;       //计数用于
    int32_t offset_ecd;    //校准偏置
};


// DJI 电机实现：继承 motor_dev 的统一接口，内部组合 CAN 收发对象和两个 PID。
// 外部只调 motor_set_* / motor_get_*，不需要知道编码器、电流缩放和 CAN 帧。
class DJI_motor : public motor_dev {

public:
    /***--------------------------初始化时用------------------------------***/
    // 构造参数：CAN 句柄、回调、电机 ID、类型、是否反转；后四个有默认值。
    DJI_motor(CAN_HandleTypeDef *hcan_=&hcan1, can_rx_callback *callback_ = nullptr,
            uint32_t id_motor = 1, DJI_MOTOR_TYPE type_ = DJI_M3508, uint8_t is_reverse = 0);

    /***--------------------------指令------------------------------***/
    bool motor_reset() override;                             //复位
    void motor_control(uint32_t cmd) override;

    // 把速度反馈源换成外部指针（例如陀螺仪），不换就用电机的 speed_rpm。
    void motor_change_speed_source(float *speed);

protected:
    /****------------------------------电机运行时用---------------------------------***/
    float motor_get_speed_forward() override;  //返回电机转子速度 rpm 际实际值 不做归一化
    float motor_get_rounds_forward() override; //返回电机转子圈数 实际值 不做归一化

    void motor_set_current_forward(float current) override;  //范围 -1～1
    void motor_set_speed_forward(float speed) override;      //范围 -1～1
    void motor_set_rounds_forward(float rounds) override;    //单位 转子圈数


    /****------------------------------回执函数及其他---------------------------------***/
    // 回调是全局函数，需要读 protected 的 motor_data，所以声明为友元。
    friend void dji_motor_data_update_callback(can_device_receive *can_receive, uint8_t *data);
    // 把 current_to_send 按大端写进组缓冲的两字节槽位。
    void dji_motor_can_tx_write();

protected:
    enum DJI_MOTOR_TYPE type;
    uint32_t id;            //电机id
    dji_motor_data motor_data;  // 本电机自己的反馈状态，解析回调通过 container_of 找到它
    bool init_offset_f;     //校准标志
    bool is_id_false;       //id是否发生错误
    int16_t current_to_send;        //要发送的电流
    can_device_receive can_rx;   // 接收对象：构造时按电机 ID 注册过滤器，回调写 motor_data
    // 本电机的发送对象：构造时按 ID 选 0x200 / 0x1FF / 0x2FF 发送组。
    // can_tx_data 指向该组的 8 字节缓冲，4 个电机共用一帧，各写各的两字节。
    can_device_transmit can_tx;
    uint8_t bias_ordinal;
    DJI_CMD cmd;
    uint8_t is_use_external_speed;   //是否使用外置速度

public:
    // 原始接收数据和组缓冲指针，调试/回调用；正常控制不需要碰。
    uint8_t can_rx_data[8];
    uint8_t *can_tx_data;
    pid velPid;   // 速度环：输入归一化速度，输出归一化电流（max_out=0.5）
    pid posPid;   // 位置环：输入转子圈数，输出归一化速度（max_out=0.2）
};

#ifdef __cplusplus
extern "C" {
#endif

// 任务入口，C 生成的 freertos.c 按 C 符号名调用。
void DJI_motor_service(void *argument);

#ifdef __cplusplus
}
#endif
//C++

#endif //PEPPER_PICKER_GIMBAL_DJI_MOTOR_DRIVER_H
