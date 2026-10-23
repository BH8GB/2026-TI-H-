#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>

/* 每圈约30000计数，20毫秒计数等于RPM乘10 */
#define WHEEL_SPEED_PERIOD_MS       20U
#define WHEEL_COUNTS_PER_REV        30000L
#define WHEEL_COUNTS_PER_RPM_20MS   10
#define WHEEL_SPEED_MAX_RPM         300

typedef struct {
    int16_t target_left;       /* 单位RPM */
    int16_t target_right;      /* 单位RPM */
    int16_t measured_left;     /* 单位RPM */
    int16_t measured_right;    /* 单位RPM */
    int16_t output_left;
    int16_t output_right;
    uint16_t elapsed_ms;
    uint8_t enabled;
    uint8_t stale;
} WheelSpeedStatus;

void WheelSpeed_Init(uint32_t now);
void WheelSpeed_Enable(uint32_t now);
void WheelSpeed_Disable(uint8_t brake);
void WheelSpeed_SetTargets(int16_t left, int16_t right);
void WheelSpeed_SetGains(float kp, float ki);
void WheelSpeed_ResetGains(void);
uint8_t WheelSpeed_Update(uint32_t now);
void WheelSpeed_GetStatus(WheelSpeedStatus *status);

#endif
