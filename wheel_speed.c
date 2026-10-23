#include "wheel_speed.h"
#include "encoder.h"
#include "motor.h"

#define SPEED_I_MAX       14000.0f
#define SPEED_MAX_GAP_MS  100U
#define SPEED_BREAKAWAY   120
#define WHEEL_SPEED_KP_DEFAULT 12.0f
#define WHEEL_SPEED_KI_DEFAULT  0.50f

/* 编码器极性，按速度环坐标定义 */
#define WHEEL_LEFT_FEEDBACK_SIGN    1LL
#define WHEEL_RIGHT_FEEDBACK_SIGN   1LL

/* PI输入和反馈为RPM，输出为PWM */

typedef struct {
    float integral;
    int16_t last_target;
} WheelPid;

static WheelPid g_left_pid, g_right_pid;
static WheelSpeedStatus g_status;
static float g_kp = WHEEL_SPEED_KP_DEFAULT;
static float g_ki = WHEEL_SPEED_KI_DEFAULT;
static int32_t g_last_left_count;
static int32_t g_last_right_count;
static uint32_t g_last_tick;

static int16_t clamp_i16(int32_t value, int16_t limit)
{
    if (value > limit) return limit;
    if (value < -limit) return (int16_t)-limit;
    return (int16_t)value;
}

static int16_t wheel_step(WheelPid *pid, int16_t target, int16_t measured,
                          float period_scale)
{
    const int16_t output_limit = (int16_t)(PWM_MAX - PWM_DEAD);

    if (target == 0) {
        pid->integral = 0.0f;
        pid->last_target = 0;
        return 0;
    }

    if ((pid->last_target > 0 && target < 0) ||
        (pid->last_target < 0 && target > 0)) {
        pid->integral = 0.0f;
    }
    pid->last_target = target;

    float error = (float)target - (float)measured;
    float candidate_i = pid->integral + error * period_scale;
    if (candidate_i > SPEED_I_MAX) candidate_i = SPEED_I_MAX;
    if (candidate_i < -SPEED_I_MAX) candidate_i = -SPEED_I_MAX;

    float raw = g_kp * error + g_ki * candidate_i;

    /* 饱和时只累积能退出饱和的误差 */
    if ((raw <= (float)output_limit && raw >= -(float)output_limit) ||
        (raw > (float)output_limit && error < 0.0f) ||
        (raw < -(float)output_limit && error > 0.0f)) {
        pid->integral = candidate_i;
    }

    raw = g_kp * error + g_ki * pid->integral;
    int16_t output = clamp_i16((int32_t)raw, output_limit);

    /* 目标非零且车轮基本静止时补偿静摩擦 */
    if (measured >= -1 && measured <= 1) {
        if (output > 0 && output < SPEED_BREAKAWAY) output = SPEED_BREAKAWAY;
        if (output < 0 && output > -SPEED_BREAKAWAY) output = -SPEED_BREAKAWAY;
    }

    /* 只调节目标方向的驱动力，避免反向；不清积分，防止反复急刹 */
    if ((target > 0 && output < 0) || (target < 0 && output > 0)) {
        output = 0;
    }
    return output;
}

void WheelSpeed_Init(uint32_t now)
{
    g_status.enabled = 0U;
    g_status.stale = 0U;
    g_status.target_left = 0;
    g_status.target_right = 0;
    g_status.measured_left = 0;
    g_status.measured_right = 0;
    g_status.output_left = 0;
    g_status.output_right = 0;
    g_status.elapsed_ms = 0U;
    g_left_pid.integral = 0.0f;
    g_right_pid.integral = 0.0f;
    g_left_pid.last_target = 0;
    g_right_pid.last_target = 0;
    Encoder_GetCounts(&g_last_left_count, &g_last_right_count);
    g_last_tick = now;
}

void WheelSpeed_Enable(uint32_t now)
{
    WheelSpeed_Init(now);
    g_status.enabled = 1U;
}

void WheelSpeed_Disable(uint8_t brake)
{
    g_status.enabled = 0U;
    g_status.target_left = 0;
    g_status.target_right = 0;
    g_status.output_left = 0;
    g_status.output_right = 0;
    g_left_pid.integral = 0.0f;
    g_right_pid.integral = 0.0f;
    if (brake) Motor_Stop();
}

void WheelSpeed_SetTargets(int16_t left, int16_t right)
{
    if (left > WHEEL_SPEED_MAX_RPM) left = WHEEL_SPEED_MAX_RPM;
    if (left < -WHEEL_SPEED_MAX_RPM) left = -WHEEL_SPEED_MAX_RPM;
    if (right > WHEEL_SPEED_MAX_RPM) right = WHEEL_SPEED_MAX_RPM;
    if (right < -WHEEL_SPEED_MAX_RPM) right = -WHEEL_SPEED_MAX_RPM;
    g_status.target_left = left;
    g_status.target_right = right;
}

void WheelSpeed_SetGains(float kp, float ki)
{
    if (kp >= 0.0f && kp <= 50.0f) g_kp = kp;
    if (ki >= 0.0f && ki <= 10.0f) g_ki = ki;
}

/* 恢复第四问稳定轮速PI参数 */
void WheelSpeed_ResetGains(void)
{
    g_kp = WHEEL_SPEED_KP_DEFAULT;
    g_ki = WHEEL_SPEED_KI_DEFAULT;
}

uint8_t WheelSpeed_Update(uint32_t now)
{
    if (!g_status.enabled) return 0U;

    uint32_t elapsed = now - g_last_tick;
    if (elapsed < WHEEL_SPEED_PERIOD_MS) return 0U;

    int32_t left_count, right_count;
    Encoder_GetCounts(&left_count, &right_count);
    int64_t raw_left = (int64_t)left_count - (int64_t)g_last_left_count;
    int64_t raw_right = (int64_t)right_count - (int64_t)g_last_right_count;
    g_last_left_count = left_count;
    g_last_right_count = right_count;
    g_last_tick = now;
    g_status.elapsed_ms = elapsed > 0xFFFFU ? 0xFFFFU : (uint16_t)elapsed;

    if (elapsed > SPEED_MAX_GAP_MS) {
        g_status.stale = 1U;
        g_left_pid.integral = 0.0f;
        g_right_pid.integral = 0.0f;
        g_status.measured_left = 0;
        g_status.measured_right = 0;
        g_status.output_left = 0;
        g_status.output_right = 0;
        Motor_Stop();
        return 1U;
    }

    g_status.stale = 0U;
    /* 车轮向前转时计数增加，直接换算为正RPM */
    int64_t counts_left = raw_left * WHEEL_LEFT_FEEDBACK_SIGN *
                          (int64_t)WHEEL_SPEED_PERIOD_MS / (int64_t)elapsed;
    int64_t counts_right = raw_right * WHEEL_RIGHT_FEEDBACK_SIGN *
                           (int64_t)WHEEL_SPEED_PERIOD_MS / (int64_t)elapsed;
    g_status.measured_left = clamp_i16((int32_t)(counts_left / WHEEL_COUNTS_PER_RPM_20MS), 32767);
    g_status.measured_right = clamp_i16((int32_t)(counts_right / WHEEL_COUNTS_PER_RPM_20MS), 32767);

    float period_scale = (float)elapsed / (float)WHEEL_SPEED_PERIOD_MS;
    g_status.output_left = wheel_step(&g_left_pid, g_status.target_left,
                                      g_status.measured_left, period_scale);
    g_status.output_right = wheel_step(&g_right_pid, g_status.target_right,
                                       g_status.measured_right, period_scale);

    if (g_status.target_left == 0 && g_status.target_right == 0) {
        Motor_Stop();
    } else {
        if (g_status.target_left != 0 && g_status.output_left == 0) {
            Motor_LeftCoast();
        } else {
            Motor_Left(g_status.output_left);
        }
        if (g_status.target_right != 0 && g_status.output_right == 0) {
            Motor_RightCoast();
        } else {
            Motor_Right(g_status.output_right);
        }
    }
    return 1U;
}

void WheelSpeed_GetStatus(WheelSpeedStatus *status)
{
    if (status != 0) *status = g_status;
}
