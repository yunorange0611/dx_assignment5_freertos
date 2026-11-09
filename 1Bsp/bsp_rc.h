//
// Created by admin on 2026/11/9.
//

#ifndef DX_ASSIGNMENT5_FREERTOS_BSP_RC_H
#define DX_ASSIGNMENT5_FREERTOS_BSP_RC_H


#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif


/*缓冲区长度*/
#define RX_BUFFER_SIZE 72

/*通道值的最小、中位、最大*/
#define RC_CH_VALUE_MIN    ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX    ((uint16_t)1684)


/* 拨杆位置 */
#define RC_SW_UP    ((uint8_t)1)
#define RC_SW_MID   ((uint8_t)3)
#define RC_SW_DOWN  ((uint8_t)2)

/* 摇杆死区：归一化后 ±0.05 内归零 */
#define RC_DEADBAND 0.05f

/*关控阈值*/
#define RC_OFFLINE_TIME 30


/*归一化后的遥控数据 */
typedef struct {
    float   right_x, right_y;   /* 右摇杆，归一化 [-1,1]（含死区） */
    float   left_x,  left_y;    /* 左摇杆，归一化 [-1,1]（含死区） */
    int8_t  s1, s2;             /* 拨杆位置：-1 下 / 0 中 / +1 上 */
    uint8_t online;             /* 1 在线，0 断联 */
    uint32_t last_update_time;  /* 最近成功解析时间戳 */
} BSP_RC_t;

extern BSP_RC_t bsp_rc;


void BSP_RC_Init(void);
uint8_t BSP_RC_IsOnline(void);   /* 中断时间戳判断在线 */
void BSP_RC_Update(void);   /* 原始 → 归一化 + 在线判定 */

#ifdef __cplusplus
}
#endif








#endif //DX_ASSIGNMENT5_FREERTOS_BSP_RC_H