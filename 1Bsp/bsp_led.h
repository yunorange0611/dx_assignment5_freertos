//
// Created by admin on 2026/11/9.
//

#ifndef DX_ASSIGNMENT5_FREERTOS_BSP_LED_H
#define DX_ASSIGNMENT5_FREERTOS_BSP_LED_H

#include "main.h"
//原本写一堆定义太大奋
//enum把一段话变成值然后typedef给这类值起名字作变量类型
typedef enum {
    LED_RED = 1,
    LED_GREEN = 2,
    LED_BLUE = 3,
} led_color_t;

#ifdef __cplusplus
extern "C" {
#endif


void BSP_LED_Toggle(led_color_t color);    //翻转
void BSP_LED_Set(led_color_t color, uint8_t state);    // on: 1 亮 / 0 灭
#ifdef __cplusplus
}
#endif

#endif