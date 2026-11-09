//
// Created by admin on 2026/11/9.
//

#ifndef DX_ASSIGNMENT5_FREERTOS_DRIVER_RC_H
#define DX_ASSIGNMENT5_FREERTOS_DRIVER_RC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 摇杆 → 目标角速度上限（单位：deg/s，与 ins_data 欧拉角单位一致） */
#define RC_YAW_MAX_RATE    300.0f
#define RC_PITCH_MAX_RATE  200.0f

/* 左摇杆 → 底盘平移速度上限（mm/s）。小陀螺自转速度由底盘板自己定，不经通信下发 */
#define RC_XY_MAX_SPEED    4000.0f

/* 底盘模式（由 S1 映射） */
typedef enum {
    MODE_FOLLOW = 0,   /* 跟随 */
    MODE_SPIN   = 1    /* 小陀螺 */
} Chassis_Mode_t;

/* 大 Pitch 目标挡位（由 S2 映射）：只有低点 / 高点两个状态 */
typedef enum {
    BP_CMD_LOW  = 0,   /* 降到低点（上电默认） */
    BP_CMD_HIGH = 1    /* 升到高点 */
} BigPitch_Cmd_t;

/* 具体 target：由 BSP 归一化值翻译而来，供 Task 直接使用 */
typedef struct {
    float yaw_rate_cmd;          /* 右摇杆横 → 目标角速度 (deg/s) */
    float pitch_rate_cmd;        /* 右摇杆纵 → 目标角速度 (deg/s) */
    Chassis_Mode_t chassis_mode; /* S1：跟随 / 小陀螺 */
    uint8_t trigger_enable;      /* S1：拨弹开 / 关 */
    BigPitch_Cmd_t big_pitch_cmd;/* S2：上=抬升 / 下=下降 / 中=保持上一次 */
    int16_t vx;                  /* 底盘 X 平移速度 (mm/s)，左摇杆纵（前+） */
    int16_t vy;                  /* 底盘 Y 平移速度 (mm/s)，左摇杆横 */
    uint8_t online;              /* 1 在线，0 断联 */
    uint8_t rollover;            /* 1 已翻车，0 正常（由 task_rc 写） */
} RC_Cmd_t;

extern RC_Cmd_t rc_cmd;

/* BSP 归一化值 → rc_cmd（含断联清零） */
void DRV_RC_Update(void);

#ifdef __cplusplus
}
#endif

#endif
