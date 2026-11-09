
#ifndef __TASK_INS_H
#define __TASK_INS_H

#include "stdint.h"
#include "bsp_imu.h"
#include "QuaternionEKF.h"
#include "ins_data.h"   /* Euler_t / INS_t / ins_data（下沉到 Driver 层） */

#ifdef __cplusplus
extern "C" {
#endif

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

/**
 * @brief 用于修正安装误差的参数,demo中可无视
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

void INS_Init(void);
void INS_Task(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);


void start_task_ins(void *argument);

#ifdef __cplusplus
}
#endif

#endif
