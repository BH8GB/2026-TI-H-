/* 文件 sensor.h，用途 灰度循迹接口 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

void    Sensor_Init(void);           /* 初始化UART3并开启数字帧 */
void    Sensor_Read(void);           /* 处理缓冲区并更新原始值 */
int8_t  Sensor_CalcError(void);      /* 加权位置误差，范围负72到正72 */

void    Track_Start(void);
void    Track_Stop(void);
uint8_t Track_IsRunning(void);
uint8_t Track_ComputeWheelTargets(uint32_t now, int16_t *left, int16_t *right);
void    Track_SetBaseSpeed(int16_t rpm);
void    Track_SetKp(float kp);
int16_t Track_GetBaseSpeed(void);
uint8_t Track_GetLineOk(void);
uint32_t Track_GetFrameAge(uint32_t now);
uint16_t Track_GetValidFrames(void);
uint16_t Track_GetBadFrames(void);
uint16_t Track_GetOverflowCount(void);
uint8_t Track_GetSensorMask(void);

extern uint8_t  g_sensor_raw[8];
extern uint8_t  g_last_rx_byte;
extern uint16_t g_rx_total;

const char *Sensor_GetLast4(void);

#endif
