/* 文件 oled.h，用途 OLED接口 */

#ifndef OLED_H
#define OLED_H

#include <stdint.h>

#define OLED_ADDR  0x3C     /* SSD1306地址，7位 */
#define OLED_W     128
#define OLED_HEIGHT 64
#define OLED_PAGES 8

int  OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t page, uint8_t col, const char *str);
void OLED_ShowChar(uint8_t page, uint8_t col, char c);

#endif
