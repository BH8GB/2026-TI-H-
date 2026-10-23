/* 文件 ball_task4.h，用途 第四问接口 */

#ifndef BALL_TASK4_H
#define BALL_TASK4_H

#include <stdint.h>

typedef enum {
    BALL_TASK4_IDLE = 0,
    BALL_TASK4_ACCEL,
    BALL_TASK4_CRUISE,
    BALL_TASK4_DECEL,
    BALL_TASK4_SETTLE,
    BALL_TASK4_DONE,
    BALL_TASK4_ABORTED,
    BALL_TASK4_FAULT_NO_VISION,
    BALL_TASK4_FAULT_EDGE,
    BALL_TASK4_FAULT_TIMEOUT
} BallTask4State;

typedef enum {
    BALL_TASK4_START_OK = 0,
    BALL_TASK4_START_NO_VISION,
    BALL_TASK4_START_NOT_CENTERED
} BallTask4StartResult;

typedef struct {
    BallTask4State state;
    uint32_t elapsed_ms;
    int16_t distance_mm;
    int16_t target_rpm;
    int16_t measured_rpm;
    int16_t wheel_error_rpm;
    int16_t acceleration_mm_s2;
    int16_t ball_position_mm;
    int16_t ball_velocity_mm_s;
    int16_t feedforward_us;
    int16_t left_target_rpm;
    int16_t right_target_rpm;
} BallTask4Status;

typedef struct {
    uint16_t max_rpm;
    uint16_t ramp_rpm_per_s;
    uint16_t decel_rpm_per_s;
    uint16_t wheel_diameter_hundredth_mm;
    int16_t feedforward_milli_us_per_mm_s2;
    int16_t decel_feedforward_milli_us_per_mm_s2;
} BallTask4Config;

void BallTask4_Init(uint32_t now_ms);
BallTask4StartResult BallTask4_Start(uint32_t now_ms);
void BallTask4_Process(uint32_t now_ms);
void BallTask4_Abort(uint32_t now_ms);
uint8_t BallTask4_IsRunning(void);
void BallTask4_GetStatus(BallTask4Status *status);
void BallTask4_GetConfig(BallTask4Config *config);
uint8_t BallTask4_SetConfig(const BallTask4Config *config);
uint8_t BallTask4_ResetConfig(void);
uint8_t BallTask4_SetMaxRpm(uint16_t max_rpm);
uint8_t BallTask4_SetRampRpmPerS(uint16_t ramp_rpm_per_s);
uint8_t BallTask4_SetDecelRpmPerS(uint16_t decel_rpm_per_s);
uint8_t BallTask4_SetWheelDiameterHundredthMm(uint16_t diameter_hundredth_mm);
uint8_t BallTask4_SetFeedforwardMilli(int16_t gain_milli);
uint8_t BallTask4_SetDecelFeedforwardMilli(int16_t gain_milli);

#endif
