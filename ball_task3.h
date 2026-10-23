/* 文件 ball_task3.h，用途 第三问接口 */

#ifndef BALL_TASK3_H
#define BALL_TASK3_H

#include <stdint.h>

/* 第三问状态 */
typedef enum {
    BALL_TASK3_IDLE = 0,
    BALL_TASK3_MOVE_PLUS_5,
    BALL_TASK3_MOVE_MINUS_5,
    BALL_TASK3_DONE,
    BALL_TASK3_ABORTED,
    BALL_TASK3_FAULT_NO_VISION,
    BALL_TASK3_FAULT_NOT_CENTERED,
    BALL_TASK3_FAULT_TIMEOUT,
    BALL_TASK3_FAULT_EDGE
} BallTask3State;

/* 启动结果 */
typedef enum {
    BALL_TASK3_START_OK = 0,
    BALL_TASK3_START_NO_VISION,
    BALL_TASK3_START_NOT_CENTERED
} BallTask3StartResult;

/* 第三问状态快照 */
typedef struct {
    BallTask3State state;
    uint32_t elapsed_ms;
    int16_t position_mm;
    int16_t velocity_mm_s;
    int16_t plus_position_mm;
    int16_t final_position_mm;
    uint8_t vision_waiting;
    uint8_t settle_count;
} BallTask3Status;

/* 初始化第三问 */
void BallTask3_Init(void);

/* 从中心启动往返测试 */
BallTask3StartResult BallTask3_Start(uint32_t now_ms);

/* 主循环调用，按新视觉帧判到位 */
void BallTask3_Process(uint32_t now_ms);

/* 中止第三问并回中 */
void BallTask3_Abort(uint32_t now_ms);

/* 是否处于运动阶段 */
uint8_t BallTask3_IsRunning(void);

/* 读取第三问状态 */
void BallTask3_GetStatus(BallTask3Status *status);

#endif
