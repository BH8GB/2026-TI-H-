/* 文件 ball_task6.h，用途 第六问接口 */

#ifndef BALL_TASK6_H
#define BALL_TASK6_H

#include <stdint.h>

/* 状态值与第五问一致，便于共用执行器 */
typedef enum {
    BALL_TASK6_IDLE = 0,
    BALL_TASK6_ACCEL,
    BALL_TASK6_TRACK,
    BALL_TASK6_DECEL,
    BALL_TASK6_SETTLE,
    BALL_TASK6_DONE,
    BALL_TASK6_ABORTED,
    BALL_TASK6_FAULT_NO_VISION,
    BALL_TASK6_FAULT_START_POSITION,
    BALL_TASK6_FAULT_NO_LINE,
    BALL_TASK6_FAULT_EDGE,
    BALL_TASK6_FAULT_TIMEOUT
} BallTask6State;

typedef enum {
    BALL_TASK6_START_OK = 0,
    BALL_TASK6_START_NO_VISION,
    BALL_TASK6_START_POSITION_UNSAFE
} BallTask6StartResult;

/* 第六问预定位结果 */
typedef enum {
    BALL_TASK6_SETUP_OK = 0,
    BALL_TASK6_SETUP_NO_VISION,
    BALL_TASK6_SETUP_POSITION_UNSAFE,
    BALL_TASK6_SETUP_BALL_MOVING,
    BALL_TASK6_SETUP_NO_TARGET
} BallTask6SetupResult;

typedef struct {
    BallTask6State state;
    uint32_t elapsed_ms;
    int16_t distance_mm;
    int16_t target_rpm;
    int16_t measured_rpm;
    int16_t acceleration_mm_s2;
    int16_t line_error;
    int16_t target_position_mm;
    int16_t ball_position_mm;
    int16_t ball_velocity_mm_s;
    int16_t feedforward_us;
} BallTask6Status;

void BallTask6_Init(uint32_t now_ms);
BallTask6SetupResult BallTask6_CaptureTarget(uint32_t now_ms);
BallTask6SetupResult BallTask6_BeginPositioning(uint32_t now_ms);
void BallTask6_PrepareProcess(uint32_t now_ms);
void BallTask6_ClearSetup(void);
uint8_t BallTask6_HasCapturedTarget(void);
uint8_t BallTask6_IsPositioning(void);
uint8_t BallTask6_IsPositionReady(void);
int16_t BallTask6_GetCapturedTargetMm(void);
BallTask6StartResult BallTask6_Start(uint32_t now_ms);
void BallTask6_Process(uint32_t now_ms);
void BallTask6_Abort(uint32_t now_ms);
uint8_t BallTask6_IsRunning(void);
void BallTask6_GetStatus(BallTask6Status *status);

#endif
