//
// Created by admin on 2026/11/9.
//

#include "bsp_rc.h"
#include "usart.h"
#include <string.h>
#include <math.h>

#include "stm32f4xx_hal.h"

/* ============ BSP 内部：原始帧（未归一化） ============ */
static struct {
    uint16_t right_x, right_y;
    uint16_t left_x, left_y;
    uint8_t  s1, s2;
    volatile uint32_t last_update_time;   /* 在串口中断里写，任务里读,用这个v */
} rc_raw;

/* ============ BSP 对外：归一化后的物理量 ============ */
BSP_RC_t bsp_rc;

static uint8_t rx_buffer[RX_BUFFER_SIZE];      /* DMA 环形写入*/
static uint8_t frame_buffer[RX_BUFFER_SIZE];   /* 临时缓冲区 */
static uint16_t old_pos = 0;                   /* 上次 DMA 位置 */

/* 启动DBUS 的 DMA + 空闲中断接收*/
static void BSP_RC_StartRx(void)
{
    old_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void BSP_RC_Init(void)
{
    BSP_RC_StartRx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    BSP_RC_StartRx();
}

/* 解析 DBUS 帧：18 字节整数倍，只取最后一帧最新数据 */
static void BSP_RC_Unpack(uint8_t *data, uint16_t len)
{
    if (len == 0 || len % 18 != 0) return;
    /*位运算把这 18 字节的比特流按田字格切出来*/
    //什么shit东西//
    uint8_t *latest = data + len - 18;   /* 指针偏移到最后一帧开头 */

    uint16_t right_x = (latest[0] | (latest[1] << 8)) & 0x07ff;
    uint16_t right_y = ((latest[1] >> 3) | (latest[2] << 5)) & 0x07ff;
    uint16_t left_x  = ((latest[2] >> 6) | (latest[3] << 2) | (latest[4] << 10)) & 0x07ff;
    uint16_t left_y  = ((latest[4] >> 1) | (latest[5] << 7)) & 0x07ff;
    uint8_t  s1 = ((latest[5] >> 4) & 0x000C) >> 2;
    uint8_t  s2 = (latest[5] >> 4) & 0x0003;

    /* 合法性检查：摇杆应在 364~1684，拨杆应为 1/2/3。*/
    if (right_x < RC_CH_VALUE_MIN || right_x > RC_CH_VALUE_MAX) return;
    if (right_y < RC_CH_VALUE_MIN || right_y > RC_CH_VALUE_MAX) return;
    if (left_x  < RC_CH_VALUE_MIN || left_x  > RC_CH_VALUE_MAX) return;
    if (left_y  < RC_CH_VALUE_MIN || left_y  > RC_CH_VALUE_MAX) return;
    if (s1 < RC_SW_UP || s1 > RC_SW_MID) return;   /* 1 上 / 2 下 / 3 中 */
    if (s2 < RC_SW_UP || s2 > RC_SW_MID) return;

    rc_raw.right_x = right_x;
    rc_raw.right_y = right_y;
    rc_raw.left_x  = left_x;
    rc_raw.left_y  = left_y;
    rc_raw.s1 = s1;
    rc_raw.s2 = s2;

    rc_raw.last_update_time = HAL_GetTick();//用来判断的时间戳
}

/* 空闲中断回调：拼出完整帧后交给本层解包 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART3) return;

    uint16_t current_pos = Size;
    uint16_t len = 0;

    if (current_pos == old_pos) return;

    if (current_pos > old_pos)                  /* 线性：未绕回 */
    {
        len = current_pos - old_pos;
        memcpy(frame_buffer, &rx_buffer[old_pos], len);
    }
    else                                        /* 环形：已绕回 */
    {
        len = RX_BUFFER_SIZE - old_pos;
        memcpy(frame_buffer, &rx_buffer[old_pos], len);
        if (current_pos > 0)
        {
            memcpy(&frame_buffer[len], rx_buffer, current_pos);
            len += current_pos;
        }
    }

    old_pos = current_pos;

    BSP_RC_Unpack(frame_buffer, len);
}

/* 单通道摇杆：原始 → 归一化 [-1,1]，±死区内归零（有跳变） */
static float BSP_RC_Map_Stick(uint16_t raw)
{
    float x = ((float)raw - (float)RC_CH_VALUE_OFFSET) / 660.0f;   /* 660 = 半程 */

    if (x > 1.0f)       x = 1.0f;
    else if (x < -1.0f) x = -1.0f;

    if (fabsf(x) < RC_DEADBAND) return 0.0f;
    return x;
}

/* 拨杆原始值 → 三态位置 -1 / 0 / +1 */
static int8_t BSP_RC_Map_Switch(uint8_t sw)
{
    if (sw == RC_SW_UP)   return 1;
    if (sw == RC_SW_DOWN) return -1;
    return 0;
}

/* 在线判定*/
uint8_t BSP_RC_IsOnline(void)
{
    uint32_t last = rc_raw.last_update_time;
    if (last == 0) return 0;                       /* 上电后还没收到过有效帧 */
    return ((HAL_GetTick() - last) > RC_OFFLINE_TIME) ? 0 : 1;
}

/* 周期调用：原始 → 归一化 + 在线判定 */
void BSP_RC_Update(void)
{
    bsp_rc.last_update_time = rc_raw.last_update_time;
    bsp_rc.online = ((HAL_GetTick() - rc_raw.last_update_time) > RC_OFFLINE_TIME) ? 0 : 1;
//判断是否在线
//? 0 : 1 是三元运算符。条件成立取 0，否则取 1
    if (!bsp_rc.online)   /* 断联：物理量全部归中 */
    {
        bsp_rc.right_x = bsp_rc.right_y = 0.0f;
        bsp_rc.left_x  = bsp_rc.left_y  = 0.0f;
        bsp_rc.s1 = 0;
        bsp_rc.s2 = 0;
        return;
    }

    bsp_rc.right_x = BSP_RC_Map_Stick(rc_raw.right_x);
    bsp_rc.right_y = BSP_RC_Map_Stick(rc_raw.right_y);
    bsp_rc.left_x  = BSP_RC_Map_Stick(rc_raw.left_x);
    bsp_rc.left_y  = BSP_RC_Map_Stick(rc_raw.left_y);
    bsp_rc.s1 = BSP_RC_Map_Switch(rc_raw.s1);
    bsp_rc.s2 = BSP_RC_Map_Switch(rc_raw.s2);
}
