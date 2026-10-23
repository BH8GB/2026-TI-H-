/* 文件 led.h，用途 LED接口 */

#ifndef LED_H
#define LED_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define LED_PORT  GPIO_CTRL_PORT
#define LED_PIN   GPIO_CTRL_LED_PIN

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void delay_ms(uint32_t ms);

#endif
