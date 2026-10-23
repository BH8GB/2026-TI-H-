/* 文件 motor.h，用途 电机驱动接口 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* PWM参数由SysConfig配置 */
#define PWM_MAX      2133U
#define PWM_DEAD     30U

void Motor_Init(void);
void Motor_Stop(void);
void Motor_LeftCoast(void);
void Motor_RightCoast(void);

/* duty大于0前进，小于0后退 */
void Motor_Left(int16_t duty);
void Motor_Right(int16_t duty);

/* 原地左转和右转 */
void Motor_TurnLeft(uint16_t duty);
void Motor_TurnRight(uint16_t duty);

#endif
