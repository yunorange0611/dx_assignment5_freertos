
#include "bsp_led.h"

#define LED_ON_LEVEL    GPIO_PIN_SET
#define LED_OFF_LEVEL   GPIO_PIN_RESET

//翻转用的
void BSP_LED_Toggle(led_color_t color)
{
    switch (color) {
    case LED_RED:   HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin); break;
    case LED_GREEN: HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin); break;
    case LED_BLUE:  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin); break;
    default: break;
    }
}

//啥时候要亮了灭了用的  state: 1=亮 0=灭
void BSP_LED_Set(led_color_t color, uint8_t state)
{
    GPIO_PinState level;//state亮和灭 level拉高拉低 在解决一下位数问题
    if (state) level = LED_ON_LEVEL;
    else       level = LED_OFF_LEVEL;

    switch (color) {
    case LED_RED:   HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, level); break;
    case LED_GREEN: HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, level); break;
    case LED_BLUE:  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, level); break;
    default: break;
    }
}
