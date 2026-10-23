/* 文件 ball_task4.c，用途 第四问直线行驶并保持钢球 */

#include "ball_task4.h"
#include "ball_control.h"
#include "encoder.h"
#include "wheel_speed.h"

#define TASK4_CONTROL_PERIOD_MS              20U
#define TASK4_DISTANCE_MM                  1500.0f
#define TASK4_START_CENTER_LIMIT_MM           8
#define TASK4_BALL_EDGE_LIMIT_MM             95
#define TASK4_VISION_GRACE_MS               250U
#define TASK4_TIMEOUT_MS                   8000U
#define TASK4_STOP_MARGIN_MM                 90.0f
#define TASK4_STOP_SPEED_RPM                  4
#define TASK4_STOP_SETTLE_TICKS               8U
#define TASK4_FINE_HOLD_POSITION_MM            4
#define TASK4_FINE_HOLD_SPEED_MM_S             8
#define TASK4_FINE_HOLD_SETTLE_TICKS           5U
#define TASK4_FINE_HOLD_RELEASE_POSITION_MM     6
#define TASK4_FINE_HOLD_RELEASE_SPEED_MM_S      12
#define TASK4_SYNC_COUNTS_PER_RPM           600L
#define TASK4_SYNC_MAX_RPM                    8
#define TASK4_SPEED_FILTER_ALPHA              0.25f
#define TASK4_ACCEL_FILTER_ALPHA              0.20f
#define TASK4_ACCEL_LIMIT_MM_S2            1200.0f
#define TASK4_FEEDFORWARD_APPLY_STEP_US       20
#define TASK4_FEEDFORWARD_RELEASE_STEP_US     40
#define TASK4_FEEDFORWARD_OVERRIDE_MM          6
#define TASK4_FEEDFORWARD_RETURN_SPEED_MM_S   20
#define TASK4_SLOPE_HOLD_LEARN_US_PER_MM       6.0f
#define TASK4_SLOPE_HOLD_LEARN_MIN_SPEED_MM_S  2
#define TASK4_SLOPE_HOLD_LEARN_MAX_SPEED_MM_S 12
#define TASK4_SLOPE_HOLD_LEARN_POSITION_MM      6
#define TASK4_SLOPE_HOLD_BIAS_LIMIT_US         60.0f
#define TASK4_SLOPE_HOLD_APPLY_STEP_US         12
#define TASK4_SLOPE_HOLD_RELEASE_STEP_US       24
#define TASK4_SLOPE_LEARN_ACCEL_LIMIT_MM_S2    40
#define TASK4_STALL_ERROR_MM                   12
#define TASK4_STALL_SPEED_MM_S                  4
#define TASK4_STALL_POSITION_STEP_MM            1
#define TASK4_STALL_CONFIRM_FRAMES              5U
#define TASK4_STALL_COOLDOWN_MS                800U
#define TASK4_PI                              3.1415926f

#define TASK4_DEFAULT_MAX_RPM                100U
/* 最后一段加速放缓，减小到速过冲 */
#define TASK4_DEFAULT_RAMP_RPM_PER_S          40U
/* 减速度保持25RPM每秒，避免超过8秒 */
#define TASK4_DEFAULT_DECEL_RPM_PER_S         25U
#define TASK4_DEFAULT_WHEEL_DIAMETER_HUNDREDTH_MM 6657U
/* 实测负向漂移，前馈增益从0.50降到0.35 */
#define TASK4_DEFAULT_FF_MILLI               350
/* 减速前馈1.00倍，对应约负70微秒 */
#define TASK4_DEFAULT_DECEL_FF_MILLI        1000

static BallTask4Status g_task4;
static BallTask4Config g_config = {
    TASK4_DEFAULT_MAX_RPM,
    TASK4_DEFAULT_RAMP_RPM_PER_S,
    TASK4_DEFAULT_DECEL_RPM_PER_S,
    TASK4_DEFAULT_WHEEL_DIAMETER_HUNDREDTH_MM,
    TASK4_DEFAULT_FF_MILLI,
    TASK4_DEFAULT_DECEL_FF_MILLI
};
static uint32_t g_start_tick;
static uint32_t g_last_process_tick;
static uint32_t g_last_valid_tick;
static int32_t g_start_left_count;
static int32_t g_start_right_count;
static float g_command_rpm;
static float g_filtered_speed_mm_s;
static float g_filtered_accel_mm_s2;
static int16_t g_applied_feedforward_us;
static float g_slope_hold_bias_us;
static int16_t g_slope_hold_feedforward_us;
static uint32_t g_slope_hold_last_frame_count;
static int16_t g_slope_hold_last_position_mm;
static uint8_t g_slope_hold_have_sample;
static uint8_t g_stop_settle_count;
static uint8_t g_ball_center_settle_count;
static uint8_t g_fine_hold_engaged;
static uint32_t g_fine_hold_last_frame_count;
static uint32_t g_stall_last_frame_count;
static int16_t g_stall_last_position_mm;
static uint8_t g_stall_have_sample;
static uint8_t g_stall_confirm_frames;
static uint32_t g_stall_last_nudge_tick;

static int16_t task4_abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static float task4_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 前馈施加慢、撤销快，减少摆杆冲击 */
static int16_t task4_slew_feedforward(int16_t current, int16_t target)
{
    int16_t step = TASK4_FEEDFORWARD_APPLY_STEP_US;
    if (task4_abs_i16(target) < task4_abs_i16(current) ||
        (current > 0 && target <= 0) ||
        (current < 0 && target >= 0)) {
        step = TASK4_FEEDFORWARD_RELEASE_STEP_US;
    }
    if (target > (int16_t)(current + step)) return (int16_t)(current + step);
    if (target < (int16_t)(current - step)) return (int16_t)(current - step);
    return target;
}

/* 限制停车坡度偏置，保留控制余量 */
static float task4_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/* 坡度偏置变化更慢，换向时快速撤销 */
static int16_t task4_slew_slope_hold(int16_t current, int16_t target)
{
    int16_t step = TASK4_SLOPE_HOLD_APPLY_STEP_US;
    if (task4_abs_i16(target) < task4_abs_i16(current) ||
        (current > 0 && target <= 0) ||
        (current < 0 && target >= 0)) {
        step = TASK4_SLOPE_HOLD_RELEASE_STEP_US;
    }
    if (target > (int16_t)(current + step)) return (int16_t)(current + step);
    if (target < (int16_t)(current - step)) return (int16_t)(current - step);
    return target;
}

/* 清除第四问坡度估计和舵机偏置 */
static void task4_reset_slope_hold(void)
{
    g_slope_hold_bias_us = 0.0f;
    g_slope_hold_feedforward_us = 0;
    g_slope_hold_last_frame_count = 0U;
    g_slope_hold_last_position_mm = 0;
    g_slope_hold_have_sample = 0U;
}

/* 停车后只学习恒定坡度，不叠加第二套PID */
static int16_t task4_update_slope_hold(const BallControlStatus *ball,
                                       uint8_t learn_enabled)
{
    float target_us = 0.0f;

    if (ball == 0 || ball->enabled == 0U) {
        task4_reset_slope_hold();
    } else if (ball->link_active != 0U && ball->ball_valid != 0U) {
        target_us = g_slope_hold_bias_us;

        if (ball->confidence == 0U) {
            /* 漏检保持期不学习旧速度 */
            g_slope_hold_last_frame_count = ball->frame_count;
            g_slope_hold_have_sample = 0U;
        } else if (ball->frame_count != g_slope_hold_last_frame_count) {
            int16_t previous_abs_mm =
                task4_abs_i16(g_slope_hold_last_position_mm);
            int16_t current_abs_mm = task4_abs_i16(ball->position_mm);

            g_slope_hold_last_frame_count = ball->frame_count;
            if (learn_enabled != 0U &&
                g_slope_hold_have_sample != 0U &&
                current_abs_mm > previous_abs_mm &&
                current_abs_mm <= TASK4_SLOPE_HOLD_LEARN_POSITION_MM &&
                task4_abs_i16(ball->velocity_mm_s) >
                    TASK4_SLOPE_HOLD_LEARN_MIN_SPEED_MM_S &&
                task4_abs_i16(ball->velocity_mm_s) <=
                    TASK4_SLOPE_HOLD_LEARN_MAX_SPEED_MM_S) {
                int16_t outward_mm =
                    (int16_t)(current_abs_mm - previous_abs_mm);
                float direction = (ball->position_mm >= 0) ? 1.0f : -1.0f;

                g_slope_hold_bias_us += direction *
                    (float)outward_mm * TASK4_SLOPE_HOLD_LEARN_US_PER_MM;
                g_slope_hold_bias_us = task4_clampf(
                    g_slope_hold_bias_us,
                    -TASK4_SLOPE_HOLD_BIAS_LIMIT_US,
                    TASK4_SLOPE_HOLD_BIAS_LIMIT_US);
                target_us = g_slope_hold_bias_us;
            }

            g_slope_hold_last_position_mm = ball->position_mm;
            g_slope_hold_have_sample = learn_enabled;
        }
    } else {
        /* 短时失联撤销偏置但保留估计 */
        g_slope_hold_have_sample = 0U;
    }

    g_slope_hold_feedforward_us = task4_slew_slope_hold(
        g_slope_hold_feedforward_us, (int16_t)target_us);
    return g_slope_hold_feedforward_us;
}

/* 钢球卡在偏心时使用一次短助推 */
static void task4_update_stall_recovery(const BallControlStatus *ball,
                                        uint32_t now_ms,
                                        uint8_t enabled)
{
    uint8_t new_valid_frame = 0U;

    if (ball == 0) return;
    if (ball->link_active != 0U && ball->ball_valid != 0U &&
        ball->confidence != 0U &&
        ball->frame_count != g_stall_last_frame_count) {
        g_stall_last_frame_count = ball->frame_count;
        new_valid_frame = 1U;
    }

    if (enabled != 0U && g_fine_hold_engaged == 0U &&
        new_valid_frame != 0U &&
        task4_abs_i16(ball->position_mm) >= TASK4_STALL_ERROR_MM &&
        task4_abs_i16(ball->velocity_mm_s) <= TASK4_STALL_SPEED_MM_S) {
        if (g_stall_have_sample != 0U &&
            task4_abs_i16((int16_t)(ball->position_mm -
                                    g_stall_last_position_mm)) <=
                TASK4_STALL_POSITION_STEP_MM) {
            if (g_stall_confirm_frames < TASK4_STALL_CONFIRM_FRAMES) {
                g_stall_confirm_frames++;
            }
        } else {
            g_stall_confirm_frames = 1U;
        }

        if (g_stall_confirm_frames >= TASK4_STALL_CONFIRM_FRAMES &&
            (uint32_t)(now_ms - g_stall_last_nudge_tick) >=
                TASK4_STALL_COOLDOWN_MS) {
            if (BallControl_RequestStallNudge() != 0U) {
                g_stall_last_nudge_tick = now_ms;
            }
            g_stall_confirm_frames = 0U;
        }
        g_stall_last_position_mm = ball->position_mm;
        g_stall_have_sample = 1U;
    } else if (new_valid_frame != 0U) {
        g_stall_confirm_frames = 0U;
        g_stall_last_position_mm = ball->position_mm;
        g_stall_have_sample = 1U;
    }
}

/* 前馈不能压过闭环纠偏 */
static int16_t task4_guard_feedforward(int16_t requested,
                                       const BallControlStatus *ball)
{
    int32_t position_velocity;
    int32_t feedforward_position;
    int32_t feedforward_velocity;
    if (ball == 0 || ball->link_active == 0U || ball->ball_valid == 0U) {
        return 0;
    }

    position_velocity = (int32_t)ball->position_mm *
                        (int32_t)ball->velocity_mm_s;
    feedforward_velocity = (int32_t)requested *
                           (int32_t)ball->velocity_mm_s;
    if (position_velocity < 0 &&
        feedforward_velocity < 0 &&
        task4_abs_i16(ball->velocity_mm_s) >=
            TASK4_FEEDFORWARD_RETURN_SPEED_MM_S) {
        return 0;
    }

    if (task4_abs_i16(ball->position_mm) <= TASK4_FEEDFORWARD_OVERRIDE_MM) {
        return requested;
    }

    feedforward_position = (int32_t)requested * (int32_t)ball->position_mm;
    if (feedforward_position <= 0) return 0;
    return requested;
}

/* 钢球回中且低速后才进入精细保持 */
static void task4_update_final_hold(const BallControlStatus *ball)
{
    uint8_t new_valid_frame = 0U;
    if (ball == 0) return;

    /* 稳定计数只接受新视觉帧 */
    if (ball->link_active != 0U && ball->ball_valid != 0U &&
        ball->confidence != 0U &&
        ball->frame_count != g_fine_hold_last_frame_count) {
        g_fine_hold_last_frame_count = ball->frame_count;
        new_valid_frame = 1U;
    }

    /* 精细保持降低低速补偿，越界立即恢复完整控制 */
    if (g_fine_hold_engaged != 0U) {
        if (ball->link_active == 0U || ball->ball_valid == 0U) {
            g_ball_center_settle_count = 0U;
            return;
        }
        if (task4_abs_i16(ball->position_mm) >
                TASK4_FINE_HOLD_RELEASE_POSITION_MM ||
            task4_abs_i16(ball->velocity_mm_s) >
                TASK4_FINE_HOLD_RELEASE_SPEED_MM_S) {
            BallControl_SetFineHold(0U);
            g_fine_hold_engaged = 0U;
            g_ball_center_settle_count = 0U;
        } else {
            return;
        }
    }

    if (ball->link_active != 0U && ball->ball_valid != 0U &&
        ball->confidence != 0U &&
        task4_abs_i16(ball->position_mm) <= TASK4_FINE_HOLD_POSITION_MM &&
        task4_abs_i16(ball->velocity_mm_s) <= TASK4_FINE_HOLD_SPEED_MM_S &&
        new_valid_frame != 0U) {
        if (g_ball_center_settle_count < TASK4_FINE_HOLD_SETTLE_TICKS) {
            g_ball_center_settle_count++;
        }
    } else if (ball->link_active == 0U || ball->ball_valid == 0U ||
               ball->confidence == 0U ||
               task4_abs_i16(ball->position_mm) >
                   TASK4_FINE_HOLD_POSITION_MM ||
               task4_abs_i16(ball->velocity_mm_s) >
                   TASK4_FINE_HOLD_SPEED_MM_S) {
        g_ball_center_settle_count = 0U;
    }

    if (g_ball_center_settle_count >= TASK4_FINE_HOLD_SETTLE_TICKS) {
        BallControl_SetFineHold(1U);
        g_fine_hold_engaged = 1U;
    }
}

static float task4_wheel_circumference_mm(void)
{
    return TASK4_PI * (float)g_config.wheel_diameter_hundredth_mm / 100.0f;
}

static float task4_counts_per_mm(void)
{
    return (float)ENCODER_COUNTS_PER_REV / task4_wheel_circumference_mm();
}

static int32_t task4_average_travel_counts(int32_t left, int32_t right)
{
    int64_t left_delta = (int64_t)left - (int64_t)g_start_left_count;
    int64_t right_delta = (int64_t)right - (int64_t)g_start_right_count;
    int64_t average = (left_delta + right_delta) / 2;
    if (average > 2147483647LL) average = 2147483647LL;
    if (average < -2147483647LL) average = -2147483647LL;
    return (int32_t)average;
}

static void task4_stop_drive(void)
{
    WheelSpeed_SetTargets(0, 0);
    WheelSpeed_Disable(1U);
    g_filtered_speed_mm_s = 0.0f;
    g_filtered_accel_mm_s2 = 0.0f;
    g_applied_feedforward_us = 0;
    g_task4.target_rpm = 0;
    g_task4.measured_rpm = 0;
    g_task4.acceleration_mm_s2 = 0;
    g_task4.left_target_rpm = 0;
    g_task4.right_target_rpm = 0;
    BallControl_SetMotionFeedforwardUs(g_slope_hold_feedforward_us);
    g_task4.feedforward_us = g_slope_hold_feedforward_us;
}

/* 只在启动、停止或故障时清除坡度学习 */
static void task4_reset_final_hold(void)
{
    task4_reset_slope_hold();
    BallControl_SetFineHold(0U);
    BallControl_SetMotionFeedforwardUs(0);
    g_fine_hold_engaged = 0U;
    g_ball_center_settle_count = 0U;
    g_fine_hold_last_frame_count = 0U;
    g_stall_last_frame_count = 0U;
    g_stall_last_position_mm = 0;
    g_stall_have_sample = 0U;
    g_stall_confirm_frames = 0U;
    g_stall_last_nudge_tick = 0U;
    g_task4.feedforward_us = 0;
}

static void task4_enter_fault(BallTask4State state, uint32_t now_ms)
{
    g_task4.state = state;
    g_task4.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    task4_stop_drive();
    task4_reset_final_hold();
    if (state == BALL_TASK4_FAULT_NO_VISION) {
        BallControl_EStop(BALL_CONTROL_SAFETY_VISION);
    } else if (state == BALL_TASK4_FAULT_EDGE) {
        BallControl_EStop(BALL_CONTROL_SAFETY_EDGE);
    } else {
        BallControl_EStop(BALL_CONTROL_SAFETY_NONE);
    }
}

void BallTask4_Init(uint32_t now_ms)
{
    g_task4.state = BALL_TASK4_IDLE;
    g_task4.elapsed_ms = 0U;
    g_task4.distance_mm = 0;
    g_task4.target_rpm = 0;
    g_task4.measured_rpm = 0;
    g_task4.wheel_error_rpm = 0;
    g_task4.acceleration_mm_s2 = 0;
    g_task4.ball_position_mm = 0;
    g_task4.ball_velocity_mm_s = 0;
    g_task4.feedforward_us = 0;
    g_task4.left_target_rpm = 0;
    g_task4.right_target_rpm = 0;
    g_start_tick = now_ms;
    g_last_process_tick = now_ms;
    g_last_valid_tick = now_ms;
    g_start_left_count = 0;
    g_start_right_count = 0;
    g_command_rpm = 0.0f;
    g_filtered_speed_mm_s = 0.0f;
    g_filtered_accel_mm_s2 = 0.0f;
    g_applied_feedforward_us = 0;
    g_stop_settle_count = 0U;
    task4_reset_final_hold();
}

BallTask4StartResult BallTask4_Start(uint32_t now_ms)
{
    BallControlStatus ball;
    task4_reset_final_hold();
    BallControl_GetStatus(&ball, now_ms);
    if (ball.link_active == 0U || ball.ball_valid == 0U ||
        ball.confidence == 0U) {
        g_task4.state = BALL_TASK4_FAULT_NO_VISION;
        return BALL_TASK4_START_NO_VISION;
    }
    if (task4_abs_i16(ball.position_mm) > TASK4_START_CENTER_LIMIT_MM) {
        g_task4.state = BALL_TASK4_IDLE;
        return BALL_TASK4_START_NOT_CENTERED;
    }

    Encoder_GetCounts(&g_start_left_count, &g_start_right_count);
    g_start_tick = now_ms;
    g_last_process_tick = now_ms;
    g_last_valid_tick = now_ms;
    g_command_rpm = 0.0f;
    g_filtered_speed_mm_s = 0.0f;
    g_filtered_accel_mm_s2 = 0.0f;
    g_applied_feedforward_us = 0;
    g_stop_settle_count = 0U;
    g_stall_last_frame_count = ball.frame_count;
    g_stall_last_position_mm = ball.position_mm;
    g_stall_have_sample = 1U;
    g_stall_last_nudge_tick = now_ms - TASK4_STALL_COOLDOWN_MS;

    g_task4.state = BALL_TASK4_ACCEL;
    g_task4.elapsed_ms = 0U;
    g_task4.distance_mm = 0;
    g_task4.target_rpm = 0;
    g_task4.measured_rpm = 0;
    g_task4.wheel_error_rpm = 0;
    g_task4.acceleration_mm_s2 = 0;
    g_task4.ball_position_mm = ball.position_mm;
    g_task4.ball_velocity_mm_s = ball.velocity_mm_s;
    g_task4.feedforward_us = 0;

    /* 行驶时使用正常保持和完整静摩擦补偿 */
    BallControl_SetFineHold(0U);
    BallControl_SetTargetMm(0);
    BallControl_SetMotionFeedforwardUs(0);
    BallControl_SetEnabled(1U);
    WheelSpeed_Enable(now_ms);
    WheelSpeed_SetTargets(0, 0);
    return BALL_TASK4_START_OK;
}

void BallTask4_Process(uint32_t now_ms)
{
    BallControlStatus ball;
    WheelSpeedStatus wheels;
    int32_t left_count;
    int32_t right_count;
    uint32_t elapsed_ms;
    float distance_mm;
    float remaining_mm;
    float circumference_mm;
    float measured_speed_mm_s;
    float previous_filtered_speed;
    float raw_accel_mm_s2;
    float feedforward_accel_mm_s2;
    float accel_step_rpm;
    float decel_step_rpm;
    float decel_mm_s2;
    float reference_rpm;
    float reference_speed_mm_s;
    float braking_distance_mm;
    int16_t average_rpm;
    int16_t sync_rpm;
    int16_t base_rpm;
    int16_t left_target;
    int16_t right_target;
    int16_t feedforward_us;
    int16_t feedforward_gain_milli;
    int64_t left_delta;
    int64_t right_delta;

    if (BallTask4_IsRunning() == 0U) {
        /* 完成后继续20毫秒监测钢球和坡度 */
        if (g_task4.state == BALL_TASK4_DONE &&
            (uint32_t)(now_ms - g_last_process_tick) >=
                TASK4_CONTROL_PERIOD_MS) {
            g_last_process_tick = now_ms;
            BallControl_GetStatus(&ball, now_ms);
            g_task4.ball_position_mm = ball.position_mm;
            g_task4.ball_velocity_mm_s = ball.velocity_mm_s;
            task4_update_final_hold(&ball);
            task4_update_slope_hold(&ball, 1U);
            BallControl_SetMotionFeedforwardUs(g_slope_hold_feedforward_us);
            g_task4.feedforward_us = g_slope_hold_feedforward_us;
            task4_update_stall_recovery(&ball, now_ms, 0U);
        }
        return;
    }

    WheelSpeed_Update(now_ms);
    if ((uint32_t)(now_ms - g_last_process_tick) < TASK4_CONTROL_PERIOD_MS) {
        return;
    }
    elapsed_ms = (uint32_t)(now_ms - g_last_process_tick);
    g_last_process_tick = now_ms;

    BallControl_GetStatus(&ball, now_ms);
    WheelSpeed_GetStatus(&wheels);
    Encoder_GetCounts(&left_count, &right_count);
    g_task4.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    g_task4.ball_position_mm = ball.position_mm;
    g_task4.ball_velocity_mm_s = ball.velocity_mm_s;

    if (ball.link_active != 0U && ball.ball_valid != 0U &&
        ball.confidence != 0U) {
        g_last_valid_tick = now_ms;
    } else if ((uint32_t)(now_ms - g_last_valid_tick) >
               TASK4_VISION_GRACE_MS) {
        task4_enter_fault(BALL_TASK4_FAULT_NO_VISION, now_ms);
        return;
    }
    if (task4_abs_i16(ball.position_mm) >= TASK4_BALL_EDGE_LIMIT_MM ||
        ball.safety_reason == (uint8_t)BALL_CONTROL_SAFETY_EDGE) {
        task4_enter_fault(BALL_TASK4_FAULT_EDGE, now_ms);
        return;
    }
    if (g_task4.elapsed_ms >= TASK4_TIMEOUT_MS) {
        task4_enter_fault(BALL_TASK4_FAULT_TIMEOUT, now_ms);
        return;
    }

    distance_mm = (float)task4_average_travel_counts(left_count, right_count) /
                  task4_counts_per_mm();
    if (distance_mm < 0.0f) distance_mm = 0.0f;
    if (distance_mm > 32767.0f) distance_mm = 32767.0f;
    g_task4.distance_mm = (int16_t)distance_mm;
    remaining_mm = TASK4_DISTANCE_MM - distance_mm;

    average_rpm = (int16_t)(((int32_t)wheels.measured_left +
                             (int32_t)wheels.measured_right) / 2);
    g_task4.measured_rpm = average_rpm;
    /* 正值表示左轮更快，用差值观察直线偏差 */
    g_task4.wheel_error_rpm =
        (int16_t)((int32_t)wheels.measured_left -
                  (int32_t)wheels.measured_right);
    circumference_mm = task4_wheel_circumference_mm();
    decel_mm_s2 = (float)g_config.decel_rpm_per_s *
                  circumference_mm / 60.0f;
    measured_speed_mm_s = (float)average_rpm * circumference_mm / 60.0f;
    previous_filtered_speed = g_filtered_speed_mm_s;
    g_filtered_speed_mm_s += TASK4_SPEED_FILTER_ALPHA *
        (measured_speed_mm_s - g_filtered_speed_mm_s);
    raw_accel_mm_s2 = (g_filtered_speed_mm_s - previous_filtered_speed) *
                      1000.0f / (float)elapsed_ms;
    if (raw_accel_mm_s2 > TASK4_ACCEL_LIMIT_MM_S2) {
        raw_accel_mm_s2 = TASK4_ACCEL_LIMIT_MM_S2;
    } else if (raw_accel_mm_s2 < -TASK4_ACCEL_LIMIT_MM_S2) {
        raw_accel_mm_s2 = -TASK4_ACCEL_LIMIT_MM_S2;
    }
    g_filtered_accel_mm_s2 += TASK4_ACCEL_FILTER_ALPHA *
        (raw_accel_mm_s2 - g_filtered_accel_mm_s2);
    g_task4.acceleration_mm_s2 = (int16_t)g_filtered_accel_mm_s2;

    /* 先更新运动状态再计算前馈 */
    if (g_task4.state != BALL_TASK4_SETTLE) {
        accel_step_rpm = (float)g_config.ramp_rpm_per_s *
                         (float)elapsed_ms / 1000.0f;
        decel_step_rpm = (float)g_config.decel_rpm_per_s *
                         (float)elapsed_ms / 1000.0f;
        reference_rpm = task4_absf((float)average_rpm);
        if (g_command_rpm > reference_rpm) reference_rpm = g_command_rpm;
        reference_speed_mm_s = reference_rpm * circumference_mm / 60.0f;
        braking_distance_mm = reference_speed_mm_s * reference_speed_mm_s /
                              (2.0f * decel_mm_s2) + TASK4_STOP_MARGIN_MM;

        if (remaining_mm <= braking_distance_mm) {
            g_task4.state = BALL_TASK4_DECEL;
        }

        if (g_task4.state == BALL_TASK4_DECEL) {
            g_command_rpm -= decel_step_rpm;
            if (g_command_rpm <= 0.5f) {
                g_command_rpm = 0.0f;
                g_task4.state = BALL_TASK4_SETTLE;
                g_ball_center_settle_count = 0U;
            }
        } else {
            g_command_rpm += accel_step_rpm;
            if (g_command_rpm >= (float)g_config.max_rpm) {
                g_command_rpm = (float)g_config.max_rpm;
                g_task4.state = BALL_TASK4_CRUISE;
            }
        }
    }

    /* 巡航和停车阶段学习恒定坡度偏置 */
    task4_update_slope_hold(
        &ball,
        (g_task4.state == BALL_TASK4_CRUISE ||
         g_task4.state == BALL_TASK4_SETTLE) ? 1U : 0U);
    task4_update_stall_recovery(
        &ball, now_ms,
        (g_task4.state == BALL_TASK4_CRUISE ||
         g_task4.state == BALL_TASK4_SETTLE) ? 1U : 0U);

    /* 进入减速立即使用规划减速度预倾摆杆 */
    feedforward_accel_mm_s2 = g_filtered_accel_mm_s2;
    feedforward_gain_milli =
        g_config.feedforward_milli_us_per_mm_s2;
    if (g_task4.state == BALL_TASK4_DECEL) {
        if (feedforward_accel_mm_s2 > -decel_mm_s2) {
            feedforward_accel_mm_s2 = -decel_mm_s2;
        }
        feedforward_gain_milli =
            g_config.decel_feedforward_milli_us_per_mm_s2;
    } else if (g_task4.state == BALL_TASK4_CRUISE ||
               g_task4.state == BALL_TASK4_SETTLE) {
        /* 巡航和停车命令加速度为零，撤销动态前馈 */
        feedforward_accel_mm_s2 = 0.0f;
        feedforward_gain_milli = 0;
    }
    feedforward_us = (int16_t)(feedforward_accel_mm_s2 *
        (float)feedforward_gain_milli / 1000.0f);
    if (feedforward_us > BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US) {
        feedforward_us = BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US;
    } else if (feedforward_us <
               -BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US) {
        feedforward_us = -BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US;
    }
    feedforward_us = task4_guard_feedforward(feedforward_us, &ball);
    g_applied_feedforward_us = task4_slew_feedforward(
        g_applied_feedforward_us, feedforward_us);
    feedforward_us = (int16_t)(g_applied_feedforward_us +
                               g_slope_hold_feedforward_us);
    if (feedforward_us > BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US) {
        feedforward_us = BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US;
    } else if (feedforward_us <
               -BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US) {
        feedforward_us = -BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US;
    }
    BallControl_SetMotionFeedforwardUs(feedforward_us);
    g_task4.feedforward_us = feedforward_us;

    if (g_task4.state == BALL_TASK4_SETTLE) {
        WheelSpeed_SetTargets(0, 0);
        g_task4.target_rpm = 0;
        g_task4.left_target_rpm = 0;
        g_task4.right_target_rpm = 0;
        if (task4_abs_i16(average_rpm) <= TASK4_STOP_SPEED_RPM) {
            if (g_stop_settle_count < TASK4_STOP_SETTLE_TICKS) {
                g_stop_settle_count++;
            }
        } else {
            g_stop_settle_count = 0U;
        }
        if (g_stop_settle_count >= TASK4_STOP_SETTLE_TICKS) {
            task4_stop_drive();
            g_task4.state = BALL_TASK4_DONE;
            g_task4.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
        }
        return;
    }

    base_rpm = (int16_t)(g_command_rpm + 0.5f);
    left_delta = (int64_t)left_count - (int64_t)g_start_left_count;
    right_delta = (int64_t)right_count - (int64_t)g_start_right_count;
    sync_rpm = (int16_t)((left_delta - right_delta) /
                         TASK4_SYNC_COUNTS_PER_RPM);
    if (sync_rpm > TASK4_SYNC_MAX_RPM) sync_rpm = TASK4_SYNC_MAX_RPM;
    if (sync_rpm < -TASK4_SYNC_MAX_RPM) sync_rpm = -TASK4_SYNC_MAX_RPM;
    left_target = (int16_t)(base_rpm - sync_rpm);
    right_target = (int16_t)(base_rpm + sync_rpm);
    if (left_target < 0) left_target = 0;
    if (right_target < 0) right_target = 0;
    WheelSpeed_SetTargets(left_target, right_target);
    g_task4.target_rpm = base_rpm;
    g_task4.left_target_rpm = left_target;
    g_task4.right_target_rpm = right_target;
}

void BallTask4_Abort(uint32_t now_ms)
{
    if (g_task4.state == BALL_TASK4_IDLE ||
        g_task4.state == BALL_TASK4_ABORTED) return;
    g_task4.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    g_task4.state = BALL_TASK4_ABORTED;
    task4_stop_drive();
    task4_reset_final_hold();
    BallControl_SetEnabled(0U);
}

uint8_t BallTask4_IsRunning(void)
{
    return (g_task4.state == BALL_TASK4_ACCEL ||
            g_task4.state == BALL_TASK4_CRUISE ||
            g_task4.state == BALL_TASK4_DECEL ||
            g_task4.state == BALL_TASK4_SETTLE) ? 1U : 0U;
}

void BallTask4_GetStatus(BallTask4Status *status)
{
    if (status != 0) *status = g_task4;
}

void BallTask4_GetConfig(BallTask4Config *config)
{
    if (config != 0) *config = g_config;
}

/* 仅在任务停止时应用完整配置 */
uint8_t BallTask4_SetConfig(const BallTask4Config *config)
{
    if (config == 0 || BallTask4_IsRunning() != 0U ||
        config->max_rpm < 60U || config->max_rpm > 160U ||
        config->ramp_rpm_per_s < 20U || config->ramp_rpm_per_s > 100U ||
        config->decel_rpm_per_s < 20U || config->decel_rpm_per_s > 80U ||
        config->wheel_diameter_hundredth_mm < 5000U ||
        config->wheel_diameter_hundredth_mm > 10000U ||
        config->feedforward_milli_us_per_mm_s2 < -1000 ||
        config->feedforward_milli_us_per_mm_s2 > 1000 ||
        config->decel_feedforward_milli_us_per_mm_s2 < 0 ||
        config->decel_feedforward_milli_us_per_mm_s2 > 2000) {
        return 0U;
    }

    g_config = *config;
    return 1U;
}

/* 恢复上次稳定运行配置 */
uint8_t BallTask4_ResetConfig(void)
{
    const BallTask4Config defaults = {
        TASK4_DEFAULT_MAX_RPM,
        TASK4_DEFAULT_RAMP_RPM_PER_S,
        TASK4_DEFAULT_DECEL_RPM_PER_S,
        TASK4_DEFAULT_WHEEL_DIAMETER_HUNDREDTH_MM,
        TASK4_DEFAULT_FF_MILLI,
        TASK4_DEFAULT_DECEL_FF_MILLI
    };
    return BallTask4_SetConfig(&defaults);
}

uint8_t BallTask4_SetMaxRpm(uint16_t max_rpm)
{
    if (BallTask4_IsRunning() != 0U || max_rpm < 60U || max_rpm > 160U) {
        return 0U;
    }
    g_config.max_rpm = max_rpm;
    return 1U;
}

uint8_t BallTask4_SetRampRpmPerS(uint16_t ramp_rpm_per_s)
{
    if (BallTask4_IsRunning() != 0U || ramp_rpm_per_s < 20U ||
        ramp_rpm_per_s > 100U) {
        return 0U;
    }
    g_config.ramp_rpm_per_s = ramp_rpm_per_s;
    return 1U;
}

uint8_t BallTask4_SetDecelRpmPerS(uint16_t decel_rpm_per_s)
{
    if (BallTask4_IsRunning() != 0U || decel_rpm_per_s < 20U ||
        decel_rpm_per_s > 80U) {
        return 0U;
    }
    g_config.decel_rpm_per_s = decel_rpm_per_s;
    return 1U;
}

uint8_t BallTask4_SetWheelDiameterHundredthMm(uint16_t diameter_hundredth_mm)
{
    if (BallTask4_IsRunning() != 0U || diameter_hundredth_mm < 5000U ||
        diameter_hundredth_mm > 10000U) {
        return 0U;
    }
    g_config.wheel_diameter_hundredth_mm = diameter_hundredth_mm;
    return 1U;
}

uint8_t BallTask4_SetFeedforwardMilli(int16_t gain_milli)
{
    if (BallTask4_IsRunning() != 0U || gain_milli < -1000 ||
        gain_milli > 1000) {
        return 0U;
    }
    g_config.feedforward_milli_us_per_mm_s2 = gain_milli;
    return 1U;
}

uint8_t BallTask4_SetDecelFeedforwardMilli(int16_t gain_milli)
{
    if (BallTask4_IsRunning() != 0U || gain_milli < 0 ||
        gain_milli > 2000) {
        return 0U;
    }
    g_config.decel_feedforward_milli_us_per_mm_s2 = gain_milli;
    return 1U;
}
