/* 文件 motor.c，用途 TB6612电机驱动 */

#include "motor.h"

static uint16_t limit(int16_t d) {
    int16_t m = (int16_t)(PWM_MAX - PWM_DEAD);
    if (d < 0) d = -d; if (d > m) d = m;
    return (uint16_t)d;
}

void Motor_Init(void) { DL_TimerG_startCounter(MOTOR_PWM_INST); Motor_Stop(); }

void Motor_Stop(void) {
    /* 短接刹车，IN1和IN2都为高 */
    DL_GPIO_setPins(DIR_R_PORT, DIR_R_AIN1_PIN | DIR_R_AIN2_PIN);
    DL_GPIO_setPins(DIR_L_PORT, DIR_L_BIN1_PIN | DIR_L_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, PWM_MAX, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, PWM_MAX, DL_TIMER_CC_1_INDEX);
}

/* 撤力时滑行，停车时短接制动 */
void Motor_RightCoast(void)
{
    DL_GPIO_clearPins(DIR_R_PORT, DIR_R_AIN1_PIN | DIR_R_AIN2_PIN);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_MAX, DL_TIMER_CC_0_INDEX);
}

void Motor_LeftCoast(void)
{
    DL_GPIO_clearPins(DIR_L_PORT, DIR_L_BIN1_PIN | DIR_L_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_MAX, DL_TIMER_CC_1_INDEX);
}

void Motor_Right(int16_t d) {
    if (d == 0) {
        DL_GPIO_setPins(DIR_R_PORT, DIR_R_AIN1_PIN | DIR_R_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
            PWM_MAX, DL_TIMER_CC_0_INDEX);
        return;
    }
    if (d > 0) { /* 前进，IN1低，IN2高 */
        DL_GPIO_clearPins(DIR_R_PORT, DIR_R_AIN1_PIN);
        DL_GPIO_setPins(DIR_R_PORT, DIR_R_AIN2_PIN);
    } else {     /* 后退，IN1高，IN2低 */
        DL_GPIO_setPins(DIR_R_PORT, DIR_R_AIN1_PIN);
        DL_GPIO_clearPins(DIR_R_PORT, DIR_R_AIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_MAX - limit(d), DL_TIMER_CC_0_INDEX);
}

/* A为右轮，B为左轮，左轮线序相反 */
void Motor_Left(int16_t d) {
    if (d == 0) {
        DL_GPIO_setPins(DIR_L_PORT, DIR_L_BIN1_PIN | DIR_L_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
            PWM_MAX, DL_TIMER_CC_1_INDEX);
        return;
    }
    if (d > 0) { /* 左轮前进，BIN1高，BIN2低 */
        DL_GPIO_setPins(DIR_L_PORT, DIR_L_BIN1_PIN);
        DL_GPIO_clearPins(DIR_L_PORT, DIR_L_BIN2_PIN);
    } else {     /* 左轮后退，BIN1低，BIN2高 */
        DL_GPIO_clearPins(DIR_L_PORT, DIR_L_BIN1_PIN);
        DL_GPIO_setPins(DIR_L_PORT, DIR_L_BIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_MAX - limit(d), DL_TIMER_CC_1_INDEX);
}

void Motor_TurnLeft(uint16_t d)
{
    Motor_Left(-(int16_t)d);
    Motor_Right((int16_t)d);
}

void Motor_TurnRight(uint16_t d)
{
    Motor_Left((int16_t)d);
    Motor_Right(-(int16_t)d);
}
