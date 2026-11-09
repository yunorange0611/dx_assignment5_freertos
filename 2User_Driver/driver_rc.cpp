//
// Created by admin on 2026/11/9.
//

#include "driver_rc.h"
#include "bsp_rc.h"
RC_Cmd_t rc_cmd;   /* 统一 target，供 Task 读取 */

/* BSP 归一化值 → rc_cmd；断联时全部清零 */
void DRV_RC_Update(void)
{
    rc_cmd.online = bsp_rc.online;

    if (!bsp_rc.online)
    {
        rc_cmd.yaw_rate_cmd   = 0.0f;
        rc_cmd.pitch_rate_cmd = 0.0f;
        rc_cmd.chassis_mode   = MODE_FOLLOW;
        rc_cmd.trigger_enable = 0;
        rc_cmd.vx = 0;
        rc_cmd.vy = 0;
        /* big_pitch_cmd 不清：它是“挡位”不是速度，断联时电机本来就全部失能 */
        return;
    }

    /* 摇杆 → 目标角速度（deg/s） */
    rc_cmd.yaw_rate_cmd   = -bsp_rc.right_x * RC_YAW_MAX_RATE;   /* 右推 = 右转（yaw 角减小）；实测原取正时左右反了 */
    rc_cmd.pitch_rate_cmd =  bsp_rc.right_y * RC_PITCH_MAX_RATE;   /* 前推 = 抬头（实测原取负时前推低头，已改正） */

    /* S1 → 底盘模式 + 拨弹：上=跟随+开 / 中=跟随+关 / 下=小陀螺+关 */
    switch (bsp_rc.s1)
    {
    case 1:
        rc_cmd.chassis_mode = MODE_FOLLOW; rc_cmd.trigger_enable = 1; break;
    case -1:
        rc_cmd.chassis_mode = MODE_SPIN;   rc_cmd.trigger_enable = 0; break;
    case 0:
    default:
        rc_cmd.chassis_mode = MODE_FOLLOW; rc_cmd.trigger_enable = 0; break;
    }

    /* S2 → 大 Pitch 挡位：上=高点 / 下=低点 / 中=不改（保持上一次的挡位） */
    if (bsp_rc.s2 == 1)
    {
        rc_cmd.big_pitch_cmd = BP_CMD_HIGH;
    }
    else if (bsp_rc.s2 == -1)
    {
        rc_cmd.big_pitch_cmd = BP_CMD_LOW;
    }

    /* 左摇杆 → 底盘平移速度（mm/s）—— 符号实测不对就调这两行 */
    rc_cmd.vx = (int16_t)( bsp_rc.left_y * RC_XY_MAX_SPEED);   /* 左摇杆纵 → 前后 */
    rc_cmd.vy = (int16_t)(-bsp_rc.left_x * RC_XY_MAX_SPEED);   /* 左摇杆横 → 左右 */
}
