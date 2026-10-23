/* 文件 ball_task3.c，用途 第三问往返控制 */

#include "ball_task3.h"
#include "ball_control.h"

/* 视觉丢失超时暂停，恢复后继续 */
#define BALL_TASK3_VISION_GRACE_MS           250U

/* 启动前钢球需在中心15毫米内 */
#define BALL_TASK3_START_CENTER_LIMIT_MM      15

/* 只给位置目标，速度由控制器规划 */
#define BALL_TASK3_PLUS_TARGET_MM              50
#define BALL_TASK3_MINUS_TARGET_MM            (-50)
#define BALL_TASK3_POSITION_TOLERANCE_MM       10
#define BALL_TASK3_PLUS_TURN_SPEED_MM_S         45
#define BALL_TASK3_PLUS_CONFIRM_FRAMES           2U
#define BALL_TASK3_EDGE_LIMIT_MM               105

/* 视觉确认停滞后才允许短助推 */
#define BALL_TASK3_STALL_MIN_ERROR_MM            10
#define BALL_TASK3_STALL_MAX_SPEED_MM_S           8
#define BALL_TASK3_STALL_POSITION_SPREAD_MM       1
#define BALL_TASK3_STALL_CONFIRM_FRAMES           5U
#define BALL_TASK3_STALL_COOLDOWN_FRAMES          8U

/* 负5厘米需位置速度满足并连续5帧 */
#define BALL_TASK3_SETTLE_SPEED_MM_S           40
#define BALL_TASK3_MINUS_SETTLE_FRAMES          5U

static BallTask3Status g_task3;
static uint32_t g_start_tick;
static uint32_t g_last_valid_tick;
static uint32_t g_last_frame_count;
static uint8_t g_plus_confirm_count;
static int16_t g_plus_stall_anchor_mm;
static uint8_t g_plus_stall_count;
static uint8_t g_plus_stall_cooldown;

/* 16位有符号绝对值 */
static int16_t task3_abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

/* 故障时摆杆回机械水平 */
static void task3_enter_fault(BallTask3State state, uint32_t now_ms)
{
    BallControlSafetyReason reason = BALL_CONTROL_SAFETY_NONE;
    g_task3.state = state;
    g_task3.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    g_task3.vision_waiting = 0U;
    if (state == BALL_TASK3_FAULT_EDGE) {
        reason = BALL_CONTROL_SAFETY_EDGE;
    } else if (state == BALL_TASK3_FAULT_NO_VISION) {
        reason = BALL_CONTROL_SAFETY_VISION;
    }
    BallControl_SetFineHold(0U);
    BallControl_EStop(reason);
}

void BallTask3_Init(void)
{
    BallControl_SetFineHold(0U);
    g_task3.state = BALL_TASK3_IDLE;
    g_task3.elapsed_ms = 0U;
    g_task3.position_mm = 0;
    g_task3.velocity_mm_s = 0;
    g_task3.plus_position_mm = 0;
    g_task3.final_position_mm = 0;
    g_task3.vision_waiting = 0U;
    g_task3.settle_count = 0U;
    g_start_tick = 0U;
    g_last_valid_tick = 0U;
    g_last_frame_count = 0U;
    g_plus_confirm_count = 0U;
    g_plus_stall_anchor_mm = 0;
    g_plus_stall_count = 0U;
    g_plus_stall_cooldown = 0U;
}

BallTask3StartResult BallTask3_Start(uint32_t now_ms)
{
    BallControlStatus ball;
    BallControl_SetFineHold(0U);
    BallControl_GetStatus(&ball, now_ms);

    g_start_tick = now_ms;
    g_last_valid_tick = now_ms;
    g_last_frame_count = ball.frame_count;
    g_plus_confirm_count = 0U;
    g_plus_stall_anchor_mm = ball.position_mm;
    g_plus_stall_count = 0U;
    g_plus_stall_cooldown = 0U;
    g_task3.elapsed_ms = 0U;
    g_task3.position_mm = ball.position_mm;
    g_task3.velocity_mm_s = ball.velocity_mm_s;
    g_task3.plus_position_mm = 0;
    g_task3.final_position_mm = 0;
    g_task3.vision_waiting = 0U;
    g_task3.settle_count = 0U;

    if (ball.link_active == 0U || ball.ball_valid == 0U ||
        ball.confidence == 0U) {
        g_task3.state = BALL_TASK3_FAULT_NO_VISION;
        BallControl_SetEnabled(0U);
        return BALL_TASK3_START_NO_VISION;
    }

    if (task3_abs_i16(ball.position_mm) >
        BALL_TASK3_START_CENTER_LIMIT_MM) {
        g_task3.state = BALL_TASK3_FAULT_NOT_CENTERED;
        BallControl_SetTargetMm(0);
        BallControl_SetEnabled(0U);
        return BALL_TASK3_START_NOT_CENTERED;
    }

    BallControl_SetTargetMm(BALL_TASK3_PLUS_TARGET_MM);
    BallControl_SetEnabled(1U);
    g_task3.state = BALL_TASK3_MOVE_PLUS_5;
    return BALL_TASK3_START_OK;
}

void BallTask3_Process(uint32_t now_ms)
{
    BallControlStatus ball;
    int16_t error_mm;
    uint8_t new_frame;

    if (BallTask3_IsRunning() == 0U) return;

    BallControl_GetStatus(&ball, now_ms);
    g_task3.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    g_task3.position_mm = ball.position_mm;
    g_task3.velocity_mm_s = ball.velocity_mm_s;
    new_frame = (ball.frame_count != g_last_frame_count) ? 1U : 0U;

    /* 先执行控制核心，优先传播安全停机 */
    if (ball.safety_reason == (uint8_t)BALL_CONTROL_SAFETY_EDGE) {
        task3_enter_fault(BALL_TASK3_FAULT_EDGE, now_ms);
        return;
    }
    /* 只有新视觉帧才刷新失联计时 */
    if (new_frame != 0U && ball.link_active != 0U &&
        ball.ball_valid != 0U && ball.confidence != 0U) {
        g_last_valid_tick = now_ms;
        if (g_task3.vision_waiting != 0U) {
            g_task3.vision_waiting = 0U;
        }
    }
    if ((uint32_t)(now_ms - g_last_valid_tick) >
        BALL_TASK3_VISION_GRACE_MS) {
        if (g_task3.vision_waiting == 0U) {
            g_task3.vision_waiting = 1U;
            g_plus_confirm_count = 0U;
            g_task3.settle_count = 0U;
        }
        return;
    }

    if (task3_abs_i16(ball.position_mm) >= BALL_TASK3_EDGE_LIMIT_MM) {
        task3_enter_fault(BALL_TASK3_FAULT_EDGE, now_ms);
        return;
    }

    /* 同一视觉帧只累计一次到位 */
    if (new_frame == 0U) return;
    g_last_frame_count = ball.frame_count;
    if (ball.confidence == 0U) return;

    if (g_task3.state == BALL_TASK3_MOVE_PLUS_5) {
        error_mm = (int16_t)(BALL_TASK3_PLUS_TARGET_MM - ball.position_mm);

        if (g_plus_stall_cooldown > 0U) {
            g_plus_stall_cooldown--;
        }
        if (task3_abs_i16(error_mm) > BALL_TASK3_STALL_MIN_ERROR_MM &&
            task3_abs_i16(ball.velocity_mm_s) <=
                BALL_TASK3_STALL_MAX_SPEED_MM_S &&
            g_plus_stall_cooldown == 0U) {
            int16_t stall_delta_mm =
                (int16_t)(ball.position_mm - g_plus_stall_anchor_mm);
            if (task3_abs_i16(stall_delta_mm) <=
                BALL_TASK3_STALL_POSITION_SPREAD_MM) {
                if (g_plus_stall_count < BALL_TASK3_STALL_CONFIRM_FRAMES) {
                    g_plus_stall_count++;
                }
            } else {
                g_plus_stall_anchor_mm = ball.position_mm;
                g_plus_stall_count = 1U;
            }

            if (g_plus_stall_count >= BALL_TASK3_STALL_CONFIRM_FRAMES) {
                if (BallControl_RequestStallNudge() != 0U) {
                    g_plus_stall_cooldown =
                        BALL_TASK3_STALL_COOLDOWN_FRAMES;
                }
                g_plus_stall_anchor_mm = ball.position_mm;
                g_plus_stall_count = 0U;
            }
        } else {
            g_plus_stall_anchor_mm = ball.position_mm;
            g_plus_stall_count = 0U;
        }

        if (task3_abs_i16(error_mm) <= BALL_TASK3_POSITION_TOLERANCE_MM &&
            task3_abs_i16(ball.velocity_mm_s) <=
                BALL_TASK3_PLUS_TURN_SPEED_MM_S) {
            if (++g_plus_confirm_count >= BALL_TASK3_PLUS_CONFIRM_FRAMES) {
                g_task3.plus_position_mm = ball.position_mm;
                g_task3.settle_count = 0U;
                g_plus_confirm_count = 0U;
                g_plus_stall_count = 0U;
                g_plus_stall_cooldown = 0U;
                BallControl_SetFineHold(1U);
                BallControl_SetTargetMm(BALL_TASK3_MINUS_TARGET_MM);
                g_task3.state = BALL_TASK3_MOVE_MINUS_5;
            }
        } else {
            g_plus_confirm_count = 0U;
        }
    } else {
        error_mm = (int16_t)(BALL_TASK3_MINUS_TARGET_MM - ball.position_mm);
        if (task3_abs_i16(error_mm) <= BALL_TASK3_POSITION_TOLERANCE_MM &&
            task3_abs_i16(ball.velocity_mm_s) <=
                BALL_TASK3_SETTLE_SPEED_MM_S) {
            if (g_task3.settle_count < BALL_TASK3_MINUS_SETTLE_FRAMES) {
                g_task3.settle_count++;
            }
            if (g_task3.settle_count >= BALL_TASK3_MINUS_SETTLE_FRAMES) {
                g_task3.final_position_mm = ball.position_mm;
                g_task3.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
                /* 完成后保持负50毫米目标 */
                g_task3.state = BALL_TASK3_DONE;
                g_task3.vision_waiting = 0U;
            }
        } else {
            g_task3.settle_count = 0U;
        }
    }
}

void BallTask3_Abort(uint32_t now_ms)
{
    BallControl_SetFineHold(0U);
    if (BallTask3_IsRunning() == 0U) return;
    g_task3.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    g_task3.state = BALL_TASK3_ABORTED;
    g_task3.vision_waiting = 0U;
    g_task3.settle_count = 0U;
    g_plus_confirm_count = 0U;
    g_plus_stall_count = 0U;
    g_plus_stall_cooldown = 0U;
    BallControl_SetTargetMm(0);
    BallControl_SetEnabled(0U);
}

uint8_t BallTask3_IsRunning(void)
{
    return (g_task3.state == BALL_TASK3_MOVE_PLUS_5 ||
            g_task3.state == BALL_TASK3_MOVE_MINUS_5) ? 1U : 0U;
}

void BallTask3_GetStatus(BallTask3Status *status)
{
    if (status != 0) *status = g_task3;
}
