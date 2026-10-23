/* 文件 led.c，用途 LED和延时 */

#include "led.h"

void LED_Init(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_PIN);  /* 上电默认熄灭 */
}

void LED_On(void)
{
    DL_GPIO_setPins(LED_PORT, LED_PIN);
}

void LED_Off(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

void LED_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);  /* 1毫秒延时，32兆赫兹 */
    }
}
