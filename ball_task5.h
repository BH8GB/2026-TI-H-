/* 文件 ball_task5.h，用途 第五问接口 */

#ifndef BALL_TASK5_H
#define BALL_TASK5_H

#include <stdint.h>

typedef enum {
    BALL_TASK5_IDLE = 0,
    BALL_TASK5_ACCEL,
    BALL_TASK5_TRACK,
    BALL_TASK5_DECEL,
    BALL_TASK5_SETTLE,
    BALL_TASK5_DONE,
    BALL_TASK5_ABORTED,
    BALL_TASK5_FAULT_NO_VISION,
    BALL_TASK5_FAULT_NOT_CENTERED,
    BALL_TASK5_FAULT_NO_LINE,
    BALL_TASK5_FAULT_EDGE,
    BALL_TASK5_FAULT_TIMEOUT
} BallTask5State;

typedef enum {
    BALL_TASK5_START_OK = 0,
    BALL_TASK5_START_NO_VISION,
    BALL_TASK5_START_NOT_CENTERED
} BallTask5StartResult;

typedef struct {
    BallTask5State state;
    uint32_t elapsed_ms;
    int16_t distance_mm;
    int16_t target_rpm;
    int16_t measured_rpm;
    int16_t acceleration_mm_s2;
    int16_t line_error;
    uint16_t line_age_ms;
    int16_t target_position_mm;
    int16_t ball_position_mm;
    int16_t ball_velocity_mm_s;
    int16_t feedforward_us;
} BallTask5Status;

void BallTask5_Init(uint32_t now_ms);
BallTask5StartResult BallTask5_Start(uint32_t now_ms);
void BallTask5_Process(uint32_t now_ms);
void BallTask5_Abort(uint32_t now_ms);
uint8_t BallTask5_IsRunning(void);
void BallTask5_GetStatus(BallTask5Status *status);

#endif
