#ifndef __INS_DATA_H__
#define __INS_DATA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 欧拉角（姿态角） */
typedef struct
{
    float yaw;        // 偏航角
    float pitch;      // 俯仰角
    float roll;       // 横滚角
    float yaw_total;  // yaw 连续累加角（小陀螺用）
} Euler_t;

typedef struct
{
    float q[4];             // 四元数估计值

    float Gyro[3];          // 角速度
    float Accel[3];         // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF;         // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    Euler_t euler;          // 位姿（欧拉角）
} INS_t;

extern INS_t ins_data;

#ifdef __cplusplus
}
#endif

#endif
