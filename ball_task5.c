/* 文件 ball_task5.c，用途 第五和第六问一圈行驶 */

#include "ball_task5.h"
#include "ball_task6.h"
#include "ball_control.h"
#include "encoder.h"
#include "sensor.h"
#include "wheel_speed.h"

#define TASK5_CONTROL_PERIOD_MS             20U
#define TASK5_BASE_LAP_DISTANCE_MM         6089.0f
#define TASK5_EXTRA_STOP_DISTANCE_MM        500.0f
#define TASK5_LAP_DISTANCE_MM              (TASK5_BASE_LAP_DISTANCE_MM + \
                                             TASK5_EXTRA_STOP_DISTANCE_MM)
#define TASK6_LAP_DISTANCE_MM              (TASK5_BASE_LAP_DISTANCE_MM + \
                                             TASK5_EXTRA_STOP_DISTANCE_MM)
#define TASK5_START_CENTER_LIMIT_MM           8
#define TASK6_TARGET_LIMIT_MM               115
#define TASK6_START_EDGE_LIMIT_MM           115
#define TASK6_CAPTURE_SPEED_LIMIT_MM_S        15
#define TASK6_READY_POSITION_LIMIT_MM          5
#define TASK6_READY_SPEED_LIMIT_MM_S           15
#define TASK6_READY_HOLD_MS                  2000U
#define TASK5_BALL_EDGE_LIMIT_MM            105
#define TASK6_BALL_EDGE_LIMIT_MM            119
#define TASK5_VISION_GRACE_MS              300U
#define TASK5_LINE_GRACE_MS                300U
#define TASK5_LINE_FRESH_MS                100U
#define TASK5_SOFT_TIMEOUT_MS             30000U
#define TASK5_HARD_TIMEOUT_MS             45000U
#define TASK6_HARD_TIMEOUT_MS             45000U
#define TASK5_STOP_SPEED_RPM                  4
#define TASK5_STOP_SETTLE_TICKS              8U
#define TASK6_STOP_SPEED_RPM                  4
#define TASK6_STOP_SETTLE_TICKS              8U
/* 使用平缓速度曲线，第五问30秒，第六问45秒保护 */
#define TASK5_START_RPM                      0U
#define TASK5_DEFAULT_MAX_RPM               71U
#define TASK5_DEFAULT_ACCEL_RPM_PER_S       20U
#define TASK5_DEFAULT_DECEL_RPM_PER_S       25U
#define TASK5_STOP_MARGIN_MM                90.0f
#define TASK6_START_RPM                      0U
#define TASK6_DEFAULT_MAX_RPM               65U
#define TASK6_DEFAULT_ACCEL_RPM_PER_S       12U
#define TASK6_DEFAULT_DECEL_RPM_PER_S       25U
#define TASK6_STOP_MARGIN_MM                55.0f
#define TASK6_RUN_POSITIVE_TRIM_US          (-60)
#define TASK5_SYNC_COUNTS_PER_RPM          600L
#define TASK5_SYNC_MAX_RPM                   8
#define TASK5_SPEED_FILTER_ALPHA          0.25f
#define TASK5_ACCEL_FILTER_ALPHA           0.20f
#define TASK5_ACCEL_LIMIT_MM_S2           900.0f
#define TASK5_PI                            3.1415926f
#define TASK5_WHEEL_DIAMETER_HUNDREDTH_MM  6657U
#define TASK5_ACCEL_FF_MILLI               350
#define TASK5_DECEL_FF_MILLI              1000
#define TASK6_FF_MILLI                     500
#define TASK5_FEEDFORWARD_APPLY_STEP_US      20
#define TASK5_FEEDFORWARD_RELEASE_STEP_US     8
#define TASK5_ACCEL_CATCHUP_GAP_RPM          2.0f
#define TASK5_ACCEL_CATCHUP_LIMIT_MM_S2      80.0f
#define TASK5_TURN_FF_START_MM             1300.0f
#define TASK5_TURN_FF_FILTER_ALPHA            0.25f
#define TASK5_TURN_FF_US_PER_RPM              1.25f
#define TASK5_TURN_FF_LIMIT_US                30.0f
#define TASK5_LOW_SPEED_TRACK_RATIO          0.80f
#define TASK5_FIRST_CURVE_START_MM         1450.0f
#define TASK5_FIRST_CURVE_APPROACH_MM       1000.0f
#define TASK5_FIRST_CURVE_HOLD_END_MM      2000.0f
#define TASK5_FIRST_CURVE_RELEASE_END_MM   2400.0f
#define TASK5_FIRST_CURVE_SPEED_RPM           66.0f
#define TASK5_SECOND_CURVE_REGION_START_MM 3650.0f
#define TASK5_WHEEL_TARGET_TURN_SLEW_RPM_PER_S 240.0f
#define TASK5_WHEEL_TARGET_RELEASE_SLEW_RPM_PER_S 180.0f
#define TASK5_STRAIGHT_FINE_ERROR             13
#define TASK5_STRAIGHT_FULL_ERROR             23
#define TASK5_STRAIGHT_FINE_RATIO           0.08f
#define TASK5_STRAIGHT_FILTER_ALPHA         0.30f
#define TASK5_STRAIGHT_TRACK_CORR_LIMIT_RPM  12
#define TASK5_FIRST_TRACK_CORR_LIMIT_RPM     18
#define TASK5_SECOND_TRACK_CORR_LIMIT_RPM    18
#define TASK_LAP_FEEDFORWARD_OVERRIDE_MM      6
#define TASK_LAP_FEEDFORWARD_RETURN_SPEED_MM_S 20

typedef struct {
    float distance_mm;
    float stop_margin_mm;
    uint32_t soft_timeout_ms;
    uint32_t hard_timeout_ms;
    uint16_t start_rpm;
    uint16_t max_rpm;
    uint16_t accel_rpm_per_s;
    uint16_t decel_rpm_per_s;
    uint16_t stop_speed_rpm;
    uint8_t stop_settle_ticks;
    int16_t accel_ff_milli;
    int16_t decel_ff_milli;
    uint8_t use_planned_feedforward;
} TaskLapMotionProfile;

/* 第五问使用软起停，一圈后多走50厘米 */
static const TaskLapMotionProfile g_task5_motion = {
    TASK5_LAP_DISTANCE_MM,
    TASK5_STOP_MARGIN_MM,
    TASK5_SOFT_TIMEOUT_MS,
    TASK5_HARD_TIMEOUT_MS,
    TASK5_START_RPM,
    TASK5_DEFAULT_MAX_RPM,
    TASK5_DEFAULT_ACCEL_RPM_PER_S,
    TASK5_DEFAULT_DECEL_RPM_PER_S,
    TASK5_STOP_SPEED_RPM,
    TASK5_STOP_SETTLE_TICKS,
    TASK5_ACCEL_FF_MILLI,
    TASK5_DECEL_FF_MILLI,
    1U
};

/* 第六问使用独立加减速参数，同样多走50厘米 */
static const TaskLapMotionProfile g_task6_motion = {
    TASK6_LAP_DISTANCE_MM,
    TASK6_STOP_MARGIN_MM,
    0U,
    TASK6_HARD_TIMEOUT_MS,
    TASK6_START_RPM,
    TASK6_DEFAULT_MAX_RPM,
    TASK6_DEFAULT_ACCEL_RPM_PER_S,
    TASK6_DEFAULT_DECEL_RPM_PER_S,
    TASK6_STOP_SPEED_RPM,
    TASK6_STOP_SETTLE_TICKS,
    TASK6_FF_MILLI,
    TASK6_FF_MILLI,
    1U
};

static BallTask5Status g_task5;
/* 当前一圈任务，0为无，5为第五问，6为第六问 */
static uint8_t g_active_lap_task;
static uint32_t g_start_tick;
static uint32_t g_last_process_tick;
static uint32_t g_last_valid_tick;
static uint32_t g_last_line_tick;
static int32_t g_start_left_count;
static int32_t g_start_right_count;
static float g_command_rpm;
static float g_filtered_speed_mm_s;
static float g_filtered_accel_mm_s2;
static int16_t g_applied_feedforward_us;
static uint8_t g_stop_settle_count;
static uint8_t g_timeout_pending;
static int16_t g_saved_track_base_rpm;
static int16_t g_last_track_correction_rpm;
static uint8_t g_have_track_correction;
static float g_applied_left_target_rpm;
static float g_applied_right_target_rpm;
static float g_task5_drive_center_rpm;
static float g_task5_filtered_track_correction_rpm;
static float g_task5_filtered_turn_correction_rpm;
/* 第六问锁存目标位置 */
static int16_t g_task6_captured_target_mm;
static uint8_t g_task6_target_captured;
static uint8_t g_task6_positioning_active;
static uint8_t g_task6_position_ready;
static uint8_t g_task6_ready_tracking;
static uint32_t g_task6_ready_since_tick;

static int16_t task5_abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static float task5_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float task5_move_toward(float current, float target, float step)
{
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

/* 第一弯借用第六问较低巡航速度，并按里程平滑过渡 */
static float task5_first_curve_speed_cap(float distance_mm, float commanded_rpm)
{
    float cap_rpm = commanded_rpm;

    if (distance_mm >= TASK5_FIRST_CURVE_APPROACH_MM &&
        distance_mm < TASK5_FIRST_CURVE_START_MM) {
        cap_rpm = (float)TASK5_DEFAULT_MAX_RPM -
            ((float)TASK5_DEFAULT_MAX_RPM - TASK5_FIRST_CURVE_SPEED_RPM) *
            (distance_mm - TASK5_FIRST_CURVE_APPROACH_MM) /
            (TASK5_FIRST_CURVE_START_MM -
             TASK5_FIRST_CURVE_APPROACH_MM);
    } else if (distance_mm >= TASK5_FIRST_CURVE_START_MM &&
               distance_mm < TASK5_FIRST_CURVE_HOLD_END_MM) {
        cap_rpm = TASK5_FIRST_CURVE_SPEED_RPM;
    } else if (distance_mm >= TASK5_FIRST_CURVE_HOLD_END_MM &&
               distance_mm < TASK5_FIRST_CURVE_RELEASE_END_MM) {
        cap_rpm = TASK5_FIRST_CURVE_SPEED_RPM +
            ((float)TASK5_DEFAULT_MAX_RPM - TASK5_FIRST_CURVE_SPEED_RPM) *
            (distance_mm - TASK5_FIRST_CURVE_HOLD_END_MM) /
            (TASK5_FIRST_CURVE_RELEASE_END_MM -
             TASK5_FIRST_CURVE_HOLD_END_MM);
    }

    return (cap_rpm < commanded_rpm) ? cap_rpm : commanded_rpm;
}

static const TaskLapMotionProfile *task_lap_motion(uint8_t task_id)
{
    return (task_id == 5U) ? &g_task5_motion : &g_task6_motion;
}

/* 前馈施加慢、撤销快，避免摆杆阶跃 */
static int16_t task_lap_slew_feedforward(int16_t current, int16_t target)
{
    int16_t step = TASK5_FEEDFORWARD_APPLY_STEP_US;
    if (task5_abs_i16(target) < task5_abs_i16(current) ||
        (current > 0 && target <= 0) ||
        (current < 0 && target >= 0)) {
        step = TASK5_FEEDFORWARD_RELEASE_STEP_US;
    }
    if (target > (int16_t)(current + step)) return (int16_t)(current + step);
    if (target < (int16_t)(current - step)) return (int16_t)(current - step);
    return target;
}

/* 低速起步时限制循迹差速，随转速升高释放 */
static void task5_limit_low_speed_track(float base_rpm,
                                         float distance_mm,
                                         int16_t line_error,
                                         int16_t *left,
                                         int16_t *right)
{
    int32_t center;
    int32_t correction;
    int32_t maximum;
    float raw_correction;
    float softened_correction;
    float abs_error;
    float transition;

    if (g_active_lap_task != 5U || left == 0 || right == 0) return;
    center = ((int32_t)*left + (int32_t)*right) / 2;
    raw_correction = ((float)*left - (float)*right) * 0.5f;
    softened_correction = raw_correction;

    /* 直线保留小幅低通修正，误差增大后恢复完整控制 */
    if (distance_mm < TASK5_FIRST_CURVE_START_MM) {
        g_task5_filtered_track_correction_rpm +=
            TASK5_STRAIGHT_FILTER_ALPHA *
            (raw_correction - g_task5_filtered_track_correction_rpm);
        abs_error = (float)task5_abs_i16(line_error);
        if (line_error == -128 || abs_error <= TASK5_STRAIGHT_FINE_ERROR) {
            softened_correction =
                g_task5_filtered_track_correction_rpm *
                TASK5_STRAIGHT_FINE_RATIO;
        } else if (abs_error < TASK5_STRAIGHT_FULL_ERROR) {
            transition =
                (abs_error - (float)TASK5_STRAIGHT_FINE_ERROR) /
                (float)(TASK5_STRAIGHT_FULL_ERROR -
                        TASK5_STRAIGHT_FINE_ERROR);
            softened_correction =
                g_task5_filtered_track_correction_rpm *
                TASK5_STRAIGHT_FINE_RATIO;
            softened_correction += transition *
                (raw_correction - softened_correction);
        } else {
            g_task5_filtered_track_correction_rpm = raw_correction;
            softened_correction = raw_correction;
        }
    } else {
        g_task5_filtered_track_correction_rpm = raw_correction;
    }
    correction = (int32_t)((softened_correction >= 0.0f) ?
        (softened_correction + 0.5f) : (softened_correction - 0.5f));
    maximum = (int32_t)(base_rpm * TASK5_LOW_SPEED_TRACK_RATIO);
    /* 按里程限制循迹差速，弯道不超过18RPM */
    if (distance_mm < TASK5_FIRST_CURVE_START_MM) {
        if (maximum > TASK5_STRAIGHT_TRACK_CORR_LIMIT_RPM) {
            maximum = TASK5_STRAIGHT_TRACK_CORR_LIMIT_RPM;
        }
    } else if (distance_mm < TASK5_SECOND_CURVE_REGION_START_MM) {
        if (maximum > TASK5_FIRST_TRACK_CORR_LIMIT_RPM) {
            maximum = TASK5_FIRST_TRACK_CORR_LIMIT_RPM;
        }
    } else if (maximum > TASK5_SECOND_TRACK_CORR_LIMIT_RPM) {
        maximum = TASK5_SECOND_TRACK_CORR_LIMIT_RPM;
    }
    if (correction > maximum) correction = maximum;
    if (correction < -maximum) correction = -maximum;
    *left = (int16_t)(center + correction);
    *right = (int16_t)(center - correction);
}

/* 循迹短时断流时保持上一帧修正，超过300毫秒报故障 */
static uint8_t task5_compute_track_targets(uint32_t now_ms,
                                           float base_rpm,
                                           int16_t *left,
                                           int16_t *right)
{
    uint32_t frame_age_ms;
    int32_t center;
    int32_t correction;
    uint8_t valid;

    if (left == 0 || right == 0) return 0U;
    if (g_active_lap_task != 5U) {
        return Track_ComputeWheelTargets(now_ms, left, right);
    }

    frame_age_ms = Track_GetFrameAge(now_ms);
    if (frame_age_ms <= TASK5_LINE_FRESH_MS) {
        valid = Track_ComputeWheelTargets(now_ms, left, right);
        if (valid != 0U) {
            g_last_track_correction_rpm =
                (int16_t)(((int32_t)*left - (int32_t)*right) / 2);
            g_have_track_correction = 1U;
            return 1U;
        }
    }

    if (frame_age_ms <= TASK5_LINE_GRACE_MS) {
        center = (int32_t)(base_rpm + 0.5f);
        correction = (g_have_track_correction != 0U) ?
            (int32_t)g_last_track_correction_rpm : 0;
        *left = (int16_t)(center + correction);
        *right = (int16_t)(center - correction);
        return 1U;
    }
    return 0U;
}

/* 左右轮目标分别限斜率，避免单轮突然卸力 */
static void task5_slew_wheel_targets(uint32_t elapsed_ms,
                                     int16_t requested_left,
                                     int16_t requested_right,
                                     int16_t *applied_left,
                                     int16_t *applied_right)
{
    float step;
    float target;
    float requested_correction;
    float applied_correction;
    float slew_rpm_per_s;

    if (applied_left == 0 || applied_right == 0) return;
    if (g_active_lap_task != 5U) {
        *applied_left = requested_left;
        *applied_right = requested_right;
        return;
    }

    requested_correction = task5_absf(
        ((float)requested_left - (float)requested_right) * 0.5f);
    applied_correction = task5_absf(
        (g_applied_left_target_rpm - g_applied_right_target_rpm) * 0.5f);
    slew_rpm_per_s =
        (requested_correction > applied_correction + 1.0f) ?
        TASK5_WHEEL_TARGET_TURN_SLEW_RPM_PER_S :
        TASK5_WHEEL_TARGET_RELEASE_SLEW_RPM_PER_S;
    step = slew_rpm_per_s * (float)elapsed_ms / 1000.0f;
    if (step < 1.0f) step = 1.0f;

    target = (float)requested_left;
    if (target > g_applied_left_target_rpm + step) {
        g_applied_left_target_rpm += step;
    } else if (target < g_applied_left_target_rpm - step) {
        g_applied_left_target_rpm -= step;
    } else {
        g_applied_left_target_rpm = target;
    }

    target = (float)requested_right;
    if (target > g_applied_right_target_rpm + step) {
        g_applied_right_target_rpm += step;
    } else if (target < g_applied_right_target_rpm - step) {
        g_applied_right_target_rpm -= step;
    } else {
        g_applied_right_target_rpm = target;
    }

    *applied_left = (int16_t)(g_applied_left_target_rpm + 0.5f);
    *applied_right = (int16_t)(g_applied_right_target_rpm + 0.5f);
}

/* 车身前馈不能压过滚球闭环 */
static int16_t task_lap_guard_feedforward(int16_t requested,
                                          const BallControlStatus *ball)
{
    int16_t deviation_mm;
    int32_t position_velocity;
    int32_t feedforward_position;
    int32_t feedforward_velocity;

    if (ball == 0 || ball->link_active == 0U || ball->ball_valid == 0U) {
        return 0;
    }

    deviation_mm = (int16_t)(ball->position_mm -
                             g_task5.target_position_mm);
    position_velocity = (int32_t)deviation_mm *
                        (int32_t)ball->velocity_mm_s;
    feedforward_velocity = (int32_t)requested *
                           (int32_t)ball->velocity_mm_s;
    if (position_velocity < 0 && feedforward_velocity < 0 &&
        task5_abs_i16(ball->velocity_mm_s) >=
            TASK_LAP_FEEDFORWARD_RETURN_SPEED_MM_S) {
        return 0;
    }

    if (task5_abs_i16(deviation_mm) <= TASK_LAP_FEEDFORWARD_OVERRIDE_MM) {
        return requested;
    }

    feedforward_position = (int32_t)requested * (int32_t)deviation_mm;
    if (feedforward_position <= 0) return 0;
    return requested;
}

static float task5_wheel_circumference_mm(void)
{
    return TASK5_PI * (float)TASK5_WHEEL_DIAMETER_HUNDREDTH_MM / 100.0f;
}

static float task5_counts_per_mm(void)
{
    return (float)ENCODER_COUNTS_PER_REV / task5_wheel_circumference_mm();
}

static int32_t task5_average_travel_counts(int32_t left, int32_t right)
{
    int64_t left_delta = (int64_t)left - (int64_t)g_start_left_count;
    int64_t right_delta = (int64_t)right - (int64_t)g_start_right_count;
    int64_t average = (left_delta + right_delta) / 2;
    if (average > 2147483647LL) average = 2147483647LL;
    if (average < -2147483647LL) average = -2147483647LL;
    return (int32_t)average;
}

static void task5_stop_drive(void)
{
    Track_Stop();
    Track_SetBaseSpeed(g_saved_track_base_rpm);
    WheelSpeed_SetTargets(0, 0);
    WheelSpeed_Disable(1U);
    BallControl_SetMotionFeedforwardUs(0);
    g_filtered_speed_mm_s = 0.0f;
    g_filtered_accel_mm_s2 = 0.0f;
    g_applied_feedforward_us = 0;
    g_task5.target_rpm = 0;
    g_task5.feedforward_us = 0;
    g_task5_drive_center_rpm = 0.0f;
    g_task5_filtered_track_correction_rpm = 0.0f;
    g_task5_filtered_turn_correction_rpm = 0.0f;
}

static void task5_enter_fault(BallTask5State state, uint32_t now_ms)
{
    g_task5.state = state;
    g_task5.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    task5_stop_drive();
    if (state == BALL_TASK5_FAULT_NO_VISION) {
        BallControl_EStop(BALL_CONTROL_SAFETY_VISION);
    } else if (state == BALL_TASK5_FAULT_EDGE) {
        BallControl_EStop(BALL_CONTROL_SAFETY_EDGE);
    }
}

static void task_lap_init(uint32_t now_ms)
{
    g_task5.state = BALL_TASK5_IDLE;
    g_task5.elapsed_ms = 0U;
    g_task5.distance_mm = 0;
    g_task5.target_rpm = 0;
    g_task5.measured_rpm = 0;
    g_task5.acceleration_mm_s2 = 0;
    g_task5.line_error = 0;
    g_task5.line_age_ms = 0xFFFFU;
    g_task5.target_position_mm = 0;
    g_task5.ball_position_mm = 0;
    g_task5.ball_velocity_mm_s = 0;
    g_task5.feedforward_us = 0;
    g_start_tick = now_ms;
    g_last_process_tick = now_ms;
    g_last_valid_tick = now_ms;
    g_last_line_tick = now_ms;
    g_start_left_count = 0;
    g_start_right_count = 0;
    g_command_rpm = 0.0f;
    g_filtered_speed_mm_s = 0.0f;
    g_filtered_accel_mm_s2 = 0.0f;
    g_applied_feedforward_us = 0;
    g_stop_settle_count = 0U;
    g_timeout_pending = 0U;
    g_saved_track_base_rpm = Track_GetBaseSpeed();
    g_last_track_correction_rpm = 0;
    g_have_track_correction = 0U;
    g_applied_left_target_rpm = 0.0f;
    g_applied_right_target_rpm = 0.0f;
    g_task5_drive_center_rpm = 0.0f;
    g_task5_filtered_track_correction_rpm = 0.0f;
    g_task5_filtered_turn_correction_rpm = 0.0f;
    g_active_lap_task = 0U;
}

void BallTask5_Init(uint32_t now_ms)
{
    task_lap_init(now_ms);
}

void BallTask6_Init(uint32_t now_ms)
{
    task_lap_init(now_ms);
    g_task6_captured_target_mm = 0;
    g_task6_target_captured = 0U;
    g_task6_positioning_active = 0U;
    g_task6_position_ready = 0U;
    g_task6_ready_tracking = 0U;
    g_task6_ready_since_tick = now_ms;
}

/* 取消第六问赛前步骤，必要时关闭闭环 */
void BallTask6_ClearSetup(void)
{
    uint8_t was_positioning = g_task6_positioning_active;

    g_task6_captured_target_mm = 0;
    g_task6_target_captured = 0U;
    g_task6_positioning_active = 0U;
    g_task6_position_ready = 0U;
    g_task6_ready_tracking = 0U;
    BallControl_SetTask6PositioningAssist(0U);
    if (was_positioning != 0U) {
        BallControl_SetMotionFeedforwardUs(0);
        BallControl_SetFineHold(0U);
        BallControl_SetEnabled(0U);
    }
}

/* 第一次按PB14记录静止钢球位置 */
BallTask6SetupResult BallTask6_CaptureTarget(uint32_t now_ms)
{
    BallControlStatus ball;

    BallControl_GetStatus(&ball, now_ms);
    if (ball.link_active == 0U || ball.ball_valid == 0U ||
        ball.confidence == 0U) {
        return BALL_TASK6_SETUP_NO_VISION;
    }
    if (task5_abs_i16(ball.position_mm) > TASK6_TARGET_LIMIT_MM) {
        return BALL_TASK6_SETUP_POSITION_UNSAFE;
    }
    if (task5_abs_i16(ball.velocity_mm_s) >
        TASK6_CAPTURE_SPEED_LIMIT_MM_S) {
        return BALL_TASK6_SETUP_BALL_MOVING;
    }

    BallTask6_ClearSetup();
    BallControl_SetMotionFeedforwardUs(0);
    BallControl_SetFineHold(0U);
    BallControl_SetEnabled(0U);
    g_active_lap_task = 0U;
    g_task5.state = BALL_TASK5_IDLE;
    g_task5.elapsed_ms = 0U;
    g_task5.distance_mm = 0;
    g_task6_captured_target_mm = ball.position_mm;
    g_task6_target_captured = 1U;
    g_task5.target_position_mm = ball.position_mm;
    g_task5.ball_position_mm = ball.position_mm;
    g_task5.ball_velocity_mm_s = ball.velocity_mm_s;
    return BALL_TASK6_SETUP_OK;
}

/* 第二次按PB14只驱动摆杆到目标 */
BallTask6SetupResult BallTask6_BeginPositioning(uint32_t now_ms)
{
    BallControlStatus ball;

    if (g_task6_target_captured == 0U) {
        return BALL_TASK6_SETUP_NO_TARGET;
    }
    BallControl_GetStatus(&ball, now_ms);
    if (ball.link_active == 0U || ball.ball_valid == 0U ||
        ball.confidence == 0U) {
        return BALL_TASK6_SETUP_NO_VISION;
    }
    if (task5_abs_i16(ball.position_mm) > TASK6_START_EDGE_LIMIT_MM) {
        return BALL_TASK6_SETUP_POSITION_UNSAFE;
    }

    BallControl_SetMotionFeedforwardUs(0);
    BallControl_SetFineHold(1U);
    BallControl_SetTargetMm(g_task6_captured_target_mm);
    BallControl_SetTask6PositioningAssist(1U);
    BallControl_SetEnabled(1U);
    g_task6_positioning_active = 1U;
    g_task6_position_ready = 0U;
    g_task6_ready_tracking = 0U;
    g_task6_ready_since_tick = now_ms;
    return BALL_TASK6_SETUP_OK;
}

/* 连续稳定2秒后才允许第三次按键启动 */
void BallTask6_PrepareProcess(uint32_t now_ms)
{
    BallControlStatus ball;
    int16_t position_error_mm;

    if (g_task6_positioning_active == 0U ||
        g_task6_target_captured == 0U) return;
    BallControl_GetStatus(&ball, now_ms);
    position_error_mm = (int16_t)(ball.position_mm -
                                  g_task6_captured_target_mm);
    if (ball.link_active == 0U || ball.ball_valid == 0U ||
        ball.confidence == 0U ||
        task5_abs_i16(position_error_mm) >
            TASK6_READY_POSITION_LIMIT_MM ||
        task5_abs_i16(ball.velocity_mm_s) >
            TASK6_READY_SPEED_LIMIT_MM_S) {
        g_task6_position_ready = 0U;
        g_task6_ready_tracking = 0U;
        return;
    }

    if (g_task6_ready_tracking == 0U) {
        g_task6_ready_tracking = 1U;
        g_task6_ready_since_tick = now_ms;
    } else if ((uint32_t)(now_ms - g_task6_ready_since_tick) >=
               TASK6_READY_HOLD_MS) {
        g_task6_position_ready = 1U;
    }
}

uint8_t BallTask6_HasCapturedTarget(void)
{
    return g_task6_target_captured;
}

uint8_t BallTask6_IsPositioning(void)
{
    return g_task6_positioning_active;
}

uint8_t BallTask6_IsPositionReady(void)
{
    return g_task6_position_ready;
}

int16_t BallTask6_GetCapturedTargetMm(void)
{
    return g_task6_captured_target_mm;
}

/* 公共启动流程做安全检查并清零状态 */
static uint8_t task_lap_start(uint32_t now_ms, uint8_t task_id)
{
    BallControlStatus ball;
    const TaskLapMotionProfile *motion;
    int16_t target_position_mm;

    if (task_id != 5U && task_id != 6U) return 2U;
    motion = task_lap_motion(task_id);
    g_active_lap_task = task_id;
    g_task5_filtered_track_correction_rpm = 0.0f;
    g_task5_filtered_turn_correction_rpm = 0.0f;
    /* 失败启动也清零，避免沿用上次数据 */
    g_task5.elapsed_ms = 0U;
    g_task5.distance_mm = 0;
    g_task5.target_rpm = 0;
    g_task5.measured_rpm = 0;
    g_task5.acceleration_mm_s2 = 0;
    g_task5.line_error = 0;
    g_task5.line_age_ms = 0xFFFFU;
    g_task5.target_position_mm = 0;
    g_task5.ball_position_mm = 0;
    g_task5.ball_velocity_mm_s = 0;
    g_task5.feedforward_us = 0;
    BallControl_GetStatus(&ball, now_ms);
    if (ball.link_active == 0U || ball.ball_valid == 0U ||
        ball.confidence == 0U) {
        g_task5.state = BALL_TASK5_FAULT_NO_VISION;
        return 1U;
    }

    if (task_id == 5U) {
        /* 第五问必须从中心O点启动 */
        if (task5_abs_i16(ball.position_mm) > TASK5_START_CENTER_LIMIT_MM) {
            g_task5.state = BALL_TASK5_FAULT_NOT_CENTERED;
            g_task5.elapsed_ms = 0U;
            g_task5.target_position_mm = 0;
            g_task5.ball_position_mm = ball.position_mm;
            g_task5.ball_velocity_mm_s = ball.velocity_mm_s;
            return 2U;
        }
        target_position_mm = 0;
    } else {
        /* 第六问允许任意位置，但要留急停安全区 */
        if (task5_abs_i16(ball.position_mm) > TASK6_START_EDGE_LIMIT_MM) {
            g_task5.state = BALL_TASK5_FAULT_NOT_CENTERED;
            g_task5.elapsed_ms = 0U;
            g_task5.target_position_mm = ball.position_mm;
            g_task5.ball_position_mm = ball.position_mm;
            g_task5.ball_velocity_mm_s = ball.velocity_mm_s;
            return 2U;
        }
        if (g_task6_target_captured != 0U) {
            target_position_mm = g_task6_captured_target_mm;
            if (task5_abs_i16((int16_t)(ball.position_mm -
                                        target_position_mm)) >
                    TASK6_READY_POSITION_LIMIT_MM ||
                task5_abs_i16(ball.velocity_mm_s) >
                    TASK6_READY_SPEED_LIMIT_MM_S) {
                g_task5.state = BALL_TASK5_FAULT_NOT_CENTERED;
                g_task5.elapsed_ms = 0U;
                g_task5.target_position_mm = target_position_mm;
                g_task5.ball_position_mm = ball.position_mm;
                g_task5.ball_velocity_mm_s = ball.velocity_mm_s;
                return 2U;
            }
        } else {
            /* 串口启动第六问时以当前位置为目标 */
            target_position_mm = ball.position_mm;
        }
    }

    Encoder_GetCounts(&g_start_left_count, &g_start_right_count);
    g_start_tick = now_ms;
    g_last_process_tick = now_ms;
    g_last_valid_tick = now_ms;
    g_last_line_tick = now_ms;
    g_command_rpm = (float)motion->start_rpm;
    g_filtered_speed_mm_s = 0.0f;
    g_filtered_accel_mm_s2 = 0.0f;
    g_applied_feedforward_us = 0;
    g_stop_settle_count = 0U;
    g_timeout_pending = 0U;
    g_last_track_correction_rpm = 0;
    g_have_track_correction = 0U;
    g_applied_left_target_rpm = 0.0f;
    g_applied_right_target_rpm = 0.0f;
    g_task5_drive_center_rpm = (float)motion->start_rpm;
    g_task5_filtered_turn_correction_rpm = 0.0f;
    g_task5.state = BALL_TASK5_ACCEL;
    g_task5.elapsed_ms = 0U;
    g_task5.distance_mm = 0;
    g_task5.target_rpm = (int16_t)motion->start_rpm;
    g_task5.measured_rpm = 0;
    g_task5.acceleration_mm_s2 = 0;
    g_task5.line_error = 0;
    g_task5.line_age_ms = 0U;
    g_task5.target_position_mm = target_position_mm;
    g_task5.ball_position_mm = ball.position_mm;
    g_task5.ball_velocity_mm_s = ball.velocity_mm_s;
    g_task5.feedforward_us = 0;

    /* 第五问全程精细保持，减小中心静摩擦补偿 */
    BallControl_SetFineHold(1U);
    BallControl_SetTargetMm(target_position_mm);
    BallControl_SetTask6PositioningAssist((task_id == 6U) ? 1U : 0U);
    BallControl_SetMotionFeedforwardUs(0);
    BallControl_SetEnabled(1U);
    if (task_id == 6U) {
        /* 第六问正式运行使用较小正目标偏置，避免端点冲击 */
        BallControl_SetTask6PositionTrimUs(
            (target_position_mm > 0) ? TASK6_RUN_POSITIVE_TRIM_US : 0);
    }
    g_saved_track_base_rpm = Track_GetBaseSpeed();
    Track_SetBaseSpeed((int16_t)motion->start_rpm);
    Track_Start();
    WheelSpeed_Enable(now_ms);
    WheelSpeed_SetTargets(0, 0);
    if (task_id == 6U) {
        /* 正式计时后目标已复制，不再使用赛前锁存 */
        g_task6_target_captured = 0U;
        g_task6_positioning_active = 0U;
        g_task6_position_ready = 0U;
        g_task6_ready_tracking = 0U;
    }
    return 0U;
}

BallTask5StartResult BallTask5_Start(uint32_t now_ms)
{
    return (BallTask5StartResult)task_lap_start(now_ms, 5U);
}

BallTask6StartResult BallTask6_Start(uint32_t now_ms)
{
    return (BallTask6StartResult)task_lap_start(now_ms, 6U);
}

static uint8_t task_lap_is_running(void)
{
    return (g_task5.state == BALL_TASK5_ACCEL ||
            g_task5.state == BALL_TASK5_TRACK ||
            g_task5.state == BALL_TASK5_DECEL ||
            g_task5.state == BALL_TASK5_SETTLE) ? 1U : 0U;
}

static void task_lap_process(uint32_t now_ms)
{
    BallControlStatus ball;
    WheelSpeedStatus wheels;
    const TaskLapMotionProfile *motion;
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
    float feedforward_command_us;
    float catchup_accel_mm_s2;
    float turn_correction_rpm;
    float turn_feedforward_us;
    float decel_mm_s2;
    float drive_center_rpm;
    float previous_drive_center_rpm;
    float reference_rpm;
    float reference_speed_mm_s;
    float braking_distance_mm;
    float accel_step_rpm;
    float decel_step_rpm;
    int16_t average_rpm;
    int16_t left_target;
    int16_t right_target;
    int16_t line_error = 0;
    int16_t feedforward_us;
    int16_t feedforward_gain_milli;
    int16_t sync_rpm;
    int64_t left_delta;
    int64_t right_delta;
    uint8_t targets_valid;
    uint32_t line_frame_age_ms;

    if (task_lap_is_running() == 0U) return;
    motion = task_lap_motion(g_active_lap_task);

    WheelSpeed_Update(now_ms);
    if ((uint32_t)(now_ms - g_last_process_tick) < TASK5_CONTROL_PERIOD_MS) {
        return;
    }
    elapsed_ms = (uint32_t)(now_ms - g_last_process_tick);
    g_last_process_tick = now_ms;
    BallControl_GetStatus(&ball, now_ms);
    WheelSpeed_GetStatus(&wheels);
    Encoder_GetCounts(&left_count, &right_count);
    g_task5.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    g_task5.ball_position_mm = ball.position_mm;
    g_task5.ball_velocity_mm_s = ball.velocity_mm_s;
    line_frame_age_ms = Track_GetFrameAge(now_ms);
    g_task5.line_age_ms = (line_frame_age_ms > 0xFFFFU) ?
        0xFFFFU : (uint16_t)line_frame_age_ms;

    if (ball.link_active != 0U && ball.ball_valid != 0U &&
        ball.confidence != 0U) {
        g_last_valid_tick = now_ms;
    } else if ((uint32_t)(now_ms - g_last_valid_tick) > TASK5_VISION_GRACE_MS) {
        task5_enter_fault(BALL_TASK5_FAULT_NO_VISION, now_ms);
        return;
    }
    if (task5_abs_i16(ball.position_mm) >=
            ((g_active_lap_task == 6U) ? TASK6_BALL_EDGE_LIMIT_MM :
                                        TASK5_BALL_EDGE_LIMIT_MM) ||
        ball.safety_reason == (uint8_t)BALL_CONTROL_SAFETY_EDGE) {
        task5_enter_fault(BALL_TASK5_FAULT_EDGE, now_ms);
        return;
    }
    if (g_task5.elapsed_ms >= motion->hard_timeout_ms) {
        task5_enter_fault(BALL_TASK5_FAULT_TIMEOUT, now_ms);
        return;
    }
    if (motion->soft_timeout_ms != 0U &&
        g_task5.elapsed_ms >= motion->soft_timeout_ms) {
        /* 30秒是计分线，仍完成6589毫米后再报告超时 */
        g_timeout_pending = 1U;
    }

    distance_mm = (float)task5_average_travel_counts(left_count, right_count) /
                  task5_counts_per_mm();
    if (distance_mm < 0.0f) distance_mm = 0.0f;
    if (distance_mm > 32767.0f) distance_mm = 32767.0f;
    g_task5.distance_mm = (int16_t)distance_mm;
    remaining_mm = motion->distance_mm - distance_mm;

    average_rpm = (int16_t)(((int32_t)wheels.measured_left +
                             (int32_t)wheels.measured_right) / 2);
    g_task5.measured_rpm = average_rpm;
    circumference_mm = task5_wheel_circumference_mm();
    measured_speed_mm_s = (float)average_rpm * circumference_mm / 60.0f;
    previous_filtered_speed = g_filtered_speed_mm_s;
    g_filtered_speed_mm_s += TASK5_SPEED_FILTER_ALPHA *
        (measured_speed_mm_s - g_filtered_speed_mm_s);
    raw_accel_mm_s2 = (g_filtered_speed_mm_s - previous_filtered_speed) *
                      1000.0f / (float)elapsed_ms;
    if (raw_accel_mm_s2 > TASK5_ACCEL_LIMIT_MM_S2) {
        raw_accel_mm_s2 = TASK5_ACCEL_LIMIT_MM_S2;
    } else if (raw_accel_mm_s2 < -TASK5_ACCEL_LIMIT_MM_S2) {
        raw_accel_mm_s2 = -TASK5_ACCEL_LIMIT_MM_S2;
    }
    g_filtered_accel_mm_s2 += TASK5_ACCEL_FILTER_ALPHA *
        (raw_accel_mm_s2 - g_filtered_accel_mm_s2);
    g_task5.acceleration_mm_s2 = (int16_t)g_filtered_accel_mm_s2;

    if (Track_GetLineOk() != 0U &&
        line_frame_age_ms <= TASK5_LINE_GRACE_MS) {
        line_error = Sensor_CalcError();
        if (line_error != -128 &&
            (g_active_lap_task != 5U ||
             line_frame_age_ms <= TASK5_LINE_FRESH_MS)) {
            g_last_line_tick = now_ms;
        }
    }
    if ((uint32_t)(now_ms - g_last_line_tick) > TASK5_LINE_GRACE_MS &&
        g_task5.state != BALL_TASK5_SETTLE) {
        task5_enter_fault(BALL_TASK5_FAULT_NO_LINE, now_ms);
        return;
    }
    g_task5.line_error = (line_error == -128) ? 0 : line_error;

    if (g_task5.state == BALL_TASK5_SETTLE) {
        Track_Stop();
        WheelSpeed_SetTargets(0, 0);
        if (motion->use_planned_feedforward != 0U) {
            g_applied_feedforward_us = task_lap_slew_feedforward(
                g_applied_feedforward_us, 0);
            BallControl_SetMotionFeedforwardUs(g_applied_feedforward_us);
        } else {
            g_applied_feedforward_us = 0;
            BallControl_SetMotionFeedforwardUs(0);
        }
        g_task5.target_rpm = 0;
        g_task5.feedforward_us = g_applied_feedforward_us;
        WheelSpeed_Update(now_ms);
        if (task5_abs_i16(average_rpm) <=
            (int16_t)motion->stop_speed_rpm) {
            if (g_stop_settle_count < motion->stop_settle_ticks) {
                g_stop_settle_count++;
            }
        } else {
            g_stop_settle_count = 0U;
        }
        if (g_stop_settle_count >= motion->stop_settle_ticks) {
            task5_stop_drive();
            g_task5.state = (g_timeout_pending != 0U) ?
                BALL_TASK5_FAULT_TIMEOUT : BALL_TASK5_DONE;
            g_task5.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
        }
        return;
    }

    decel_mm_s2 = (float)motion->decel_rpm_per_s *
                  circumference_mm / 60.0f;
    reference_rpm = task5_absf((float)average_rpm);
    if (g_command_rpm > reference_rpm) reference_rpm = g_command_rpm;
    reference_speed_mm_s = reference_rpm * circumference_mm / 60.0f;
    braking_distance_mm = reference_speed_mm_s * reference_speed_mm_s /
                          (2.0f * decel_mm_s2) + motion->stop_margin_mm;
    if (remaining_mm <= braking_distance_mm) {
        g_task5.state = BALL_TASK5_DECEL;
    } else if (g_task5.state == BALL_TASK5_ACCEL &&
               g_command_rpm >= (float)motion->max_rpm) {
        g_task5.state = BALL_TASK5_TRACK;
    }

    accel_step_rpm = (float)motion->accel_rpm_per_s *
                     (float)elapsed_ms / 1000.0f;
    decel_step_rpm = (float)motion->decel_rpm_per_s *
                     (float)elapsed_ms / 1000.0f;
    if (g_task5.state == BALL_TASK5_DECEL) {
        g_command_rpm -= decel_step_rpm;
        if (g_command_rpm <= 0.5f ||
            (motion->use_planned_feedforward == 0U &&
             remaining_mm <= 0.0f)) {
            g_command_rpm = 0.0f;
            g_task5.state = BALL_TASK5_SETTLE;
            BallControl_SetFineHold(1U);
            if (motion->use_planned_feedforward != 0U) {
                g_applied_feedforward_us = task_lap_slew_feedforward(
                    g_applied_feedforward_us, 0);
                BallControl_SetMotionFeedforwardUs(
                    g_applied_feedforward_us);
                g_task5.feedforward_us = g_applied_feedforward_us;
            } else {
                g_applied_feedforward_us = 0;
                BallControl_SetMotionFeedforwardUs(0);
                g_task5.feedforward_us = 0;
            }
            Track_Stop();
            WheelSpeed_SetTargets(0, 0);
            return;
        }
    } else if (g_command_rpm < (float)motion->max_rpm) {
        g_command_rpm += accel_step_rpm;
        if (g_command_rpm >= (float)motion->max_rpm) {
            g_command_rpm = (float)motion->max_rpm;
            g_task5.state = BALL_TASK5_TRACK;
        }
    }

    /* 第五问只在起步软加速，弯道内不改中心速度 */
    previous_drive_center_rpm = g_task5_drive_center_rpm;
    drive_center_rpm = g_command_rpm;
    if (g_active_lap_task == 5U) {
        drive_center_rpm = task5_first_curve_speed_cap(
            distance_mm, drive_center_rpm);
    }
    g_task5_drive_center_rpm = drive_center_rpm;
    Track_SetBaseSpeed((int16_t)(drive_center_rpm + 0.5f));
    targets_valid = task5_compute_track_targets(
        now_ms, drive_center_rpm, &left_target, &right_target);
    if (targets_valid == 0U) {
        WheelSpeed_SetTargets(0, 0);
        BallControl_SetMotionFeedforwardUs(0);
        g_applied_feedforward_us = 0;
        g_task5.feedforward_us = 0;
        return;
    }

    task5_limit_low_speed_track(drive_center_rpm, distance_mm, line_error,
                                &left_target, &right_target);

    /* 一圈任务保持两轮向前，低速度时也不允许单轮反转 */
    if (left_target < 0) left_target = 0;
    if (right_target < 0) right_target = 0;

    left_delta = (int64_t)left_count - (int64_t)g_start_left_count;
    right_delta = (int64_t)right_count - (int64_t)g_start_right_count;
    if (g_active_lap_task == 5U) {
        /* 平均里程只用于停车，第五问转向不用累计同步修正 */
        sync_rpm = 0;
    } else {
        sync_rpm = (int16_t)((left_delta - right_delta) /
                             TASK5_SYNC_COUNTS_PER_RPM);
        if (sync_rpm > TASK5_SYNC_MAX_RPM) sync_rpm = TASK5_SYNC_MAX_RPM;
        if (sync_rpm < -TASK5_SYNC_MAX_RPM) sync_rpm = -TASK5_SYNC_MAX_RPM;
    }
    left_target = (int16_t)(left_target - sync_rpm);
    right_target = (int16_t)(right_target + sync_rpm);
    if (motion->use_planned_feedforward != 0U) {
        /* 软停末段禁止同步修正把车轮拉成反转 */
        if (left_target < 0) left_target = 0;
        if (right_target < 0) right_target = 0;
    }
    task5_slew_wheel_targets(elapsed_ms, left_target, right_target,
                             &left_target, &right_target);
    WheelSpeed_SetTargets(left_target, right_target);
    g_task5.target_rpm = (int16_t)(drive_center_rpm + 0.5f);

    turn_feedforward_us = 0.0f;
    if (g_active_lap_task == 5U) {
        /* 左右轮速差提前反映转弯，用于摆杆反向预倾 */
        turn_correction_rpm =
            ((float)left_target - (float)right_target) * 0.5f;
        if (distance_mm < TASK5_TURN_FF_START_MM ||
            g_task5.state == BALL_TASK5_DECEL ||
            g_task5.state == BALL_TASK5_SETTLE) {
            turn_correction_rpm = 0.0f;
        }
        g_task5_filtered_turn_correction_rpm +=
            TASK5_TURN_FF_FILTER_ALPHA *
            (turn_correction_rpm -
             g_task5_filtered_turn_correction_rpm);
        turn_feedforward_us = -TASK5_TURN_FF_US_PER_RPM *
                              g_task5_filtered_turn_correction_rpm;
        if (turn_feedforward_us > TASK5_TURN_FF_LIMIT_US) {
            turn_feedforward_us = TASK5_TURN_FF_LIMIT_US;
        } else if (turn_feedforward_us < -TASK5_TURN_FF_LIMIT_US) {
            turn_feedforward_us = -TASK5_TURN_FF_LIMIT_US;
        }
    }

    if (motion->use_planned_feedforward != 0U) {
        /* 软起停使用规划加速度前馈，不用轮速微分噪声 */
        /* 使用中心速度斜率，巡航和过弯时为零 */
        feedforward_accel_mm_s2 =
            (drive_center_rpm - previous_drive_center_rpm) *
            circumference_mm * 1000.0f /
            (60.0f * (float)elapsed_ms);

        /* 实测轮速追赶命令时保留受限正加速度前馈 */
        if (g_active_lap_task == 5U &&
            distance_mm < TASK5_FIRST_CURVE_START_MM &&
            drive_center_rpm >= (float)motion->max_rpm - 0.5f &&
            (float)average_rpm <
                drive_center_rpm - TASK5_ACCEL_CATCHUP_GAP_RPM &&
            g_filtered_accel_mm_s2 > 0.0f) {
            catchup_accel_mm_s2 = g_filtered_accel_mm_s2;
            if (catchup_accel_mm_s2 >
                TASK5_ACCEL_CATCHUP_LIMIT_MM_S2) {
                catchup_accel_mm_s2 =
                    TASK5_ACCEL_CATCHUP_LIMIT_MM_S2;
            }
            if (catchup_accel_mm_s2 > feedforward_accel_mm_s2) {
                feedforward_accel_mm_s2 = catchup_accel_mm_s2;
            }
        }
        feedforward_gain_milli = (feedforward_accel_mm_s2 < 0.0f) ?
            motion->decel_ff_milli : motion->accel_ff_milli;
    } else {
        /* 未启用规划前馈的任务使用实测加速度前馈 */
        feedforward_accel_mm_s2 = g_filtered_accel_mm_s2;
        feedforward_gain_milli = motion->accel_ff_milli;
    }

    feedforward_command_us = feedforward_accel_mm_s2 *
        (float)feedforward_gain_milli / 1000.0f;
    if (motion->use_planned_feedforward != 0U &&
        feedforward_accel_mm_s2 > 0.0f) {
        /* 正向加速把钢球推向车尾，起步前馈取反向 */
        feedforward_command_us = -feedforward_command_us;
    }
    if (g_active_lap_task == 5U) {
        feedforward_command_us += turn_feedforward_us;
    }
    if (feedforward_command_us >
        (float)BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US) {
        feedforward_command_us =
            (float)BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US;
    } else if (feedforward_command_us <
               -(float)BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US) {
        feedforward_command_us =
            -(float)BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US;
    }
    feedforward_us = (int16_t)((feedforward_command_us >= 0.0f) ?
        (feedforward_command_us + 0.5f) :
        (feedforward_command_us - 0.5f));
    feedforward_us = task_lap_guard_feedforward(feedforward_us, &ball);
    if (motion->use_planned_feedforward != 0U) {
        g_applied_feedforward_us = task_lap_slew_feedforward(
            g_applied_feedforward_us, feedforward_us);
        feedforward_us = g_applied_feedforward_us;
    } else {
        g_applied_feedforward_us = feedforward_us;
    }
    BallControl_SetMotionFeedforwardUs(feedforward_us);
    g_task5.feedforward_us = feedforward_us;
}

void BallTask5_Process(uint32_t now_ms)
{
    if (g_active_lap_task == 5U) task_lap_process(now_ms);
}

void BallTask6_Process(uint32_t now_ms)
{
    if (g_active_lap_task == 6U) task_lap_process(now_ms);
}

static void task_lap_abort(uint32_t now_ms)
{
    if (g_task5.state == BALL_TASK5_IDLE ||
        g_task5.state == BALL_TASK5_ABORTED) return;
    g_task5.elapsed_ms = (uint32_t)(now_ms - g_start_tick);
    g_task5.state = BALL_TASK5_ABORTED;
    task5_stop_drive();
    BallControl_SetFineHold(0U);
    BallControl_SetEnabled(0U);
}

void BallTask5_Abort(uint32_t now_ms)
{
    if (g_active_lap_task == 5U) task_lap_abort(now_ms);
}

void BallTask6_Abort(uint32_t now_ms)
{
    if (g_active_lap_task == 6U) task_lap_abort(now_ms);
    BallTask6_ClearSetup();
}

uint8_t BallTask5_IsRunning(void)
{
    return (g_active_lap_task == 5U) ? task_lap_is_running() : 0U;
}

uint8_t BallTask6_IsRunning(void)
{
    return (g_active_lap_task == 6U) ? task_lap_is_running() : 0U;
}

void BallTask5_GetStatus(BallTask5Status *status)
{
    if (status == 0) return;
    if (g_active_lap_task == 5U) {
        *status = g_task5;
    } else {
        BallTask5Status idle = {0};
        *status = idle;
    }
}

void BallTask6_GetStatus(BallTask6Status *status)
{
    if (status == 0) return;
    if (g_active_lap_task != 6U) {
        BallTask6Status idle = {0};
        *status = idle;
        return;
    }

    status->state = (BallTask6State)g_task5.state;
    status->elapsed_ms = g_task5.elapsed_ms;
    status->distance_mm = g_task5.distance_mm;
    status->target_rpm = g_task5.target_rpm;
    status->measured_rpm = g_task5.measured_rpm;
    status->acceleration_mm_s2 = g_task5.acceleration_mm_s2;
    status->line_error = g_task5.line_error;
    status->target_position_mm = g_task5.target_position_mm;
    status->ball_position_mm = g_task5.ball_position_mm;
    status->ball_velocity_mm_s = g_task5.ball_velocity_mm_s;
    status->feedforward_us = g_task5.feedforward_us;
}
