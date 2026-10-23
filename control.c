/* 文件 control.c，用途 航向和位姿控制 */

#include "control.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"

/* 航向环参数，积分暂时关闭 */
static float Kp_a = 20.0f, Ki_a = 0.0f;
static float Kp_g = 0.80f, Ki_g = 0.0f;
static float angle_sum, gyro_sum;

float Control_YawError(float target, float current)
{
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

void Control_ResetYawPid(void)
{
    angle_sum = 0.0f;
    gyro_sum = 0.0f;
}

float Control_ComputeYawTurn(float base, float target)
{
    IMUSnapshot imu;
    IMU_GetSnapshot(&imu);

    /* 角度外环输出目标角速度 */
    float e1 = Control_YawError(target, imu.yaw);
    angle_sum += e1;
    if (angle_sum > 300.0f) angle_sum = 300.0f;
    if (angle_sum < -300.0f) angle_sum = -300.0f;
    float desired = Kp_a * e1 + Ki_a * angle_sum;
    if (desired > 180.0f) desired = 180.0f;
    if (desired < -180.0f) desired = -180.0f;

    /* 陀螺内环输出差速PWM */
    float e2 = desired - imu.gyro_z;
    gyro_sum += e2;
    if (gyro_sum > 80.0f) gyro_sum = 80.0f;
    if (gyro_sum < -80.0f) gyro_sum = -80.0f;
    float turn = Kp_g * e2 + Ki_g * gyro_sum;
    if (turn > 700.0f) turn = 700.0f;
    if (turn < -700.0f) turn = -700.0f;

    if (e1 > -0.8f && e1 < 0.8f &&
        imu.gyro_z > -3.0f && imu.gyro_z < 3.0f) {
        turn = 0.0f;
    } else if (base > -0.1f && base < 0.1f) {
        /* 原地转向时补偿静摩擦 */
        if (turn > 0.1f) turn += 300.0f;
        else if (turn < -0.1f) turn -= 300.0f;
    }
    return turn;
}

void YawControl(float base, float target)
{
    float turn = Control_ComputeYawTurn(base, target);
    int16_t mx = (int16_t)(PWM_MAX - PWM_DEAD);
    int32_t left = (int32_t)(base + turn);
    int32_t right = (int32_t)(base - turn);
    if (left > mx) left = mx;
    if (left < -mx) left = -mx;
    if (right > mx) right = mx;
    if (right < -mx) right = -mx;
    Motor_Left((int16_t)left);
    Motor_Right((int16_t)right);
}

static float g_yaw_target;

void Control_SetYawTarget(float yaw)
{
    /* 角度归一化到正负180度 */
    if (yaw > 180.0f) yaw -= 360.0f;
    else if (yaw < -180.0f) yaw += 360.0f;
    g_yaw_target = yaw;
    Control_ResetYawPid();
}

float Control_GetYawTarget(void)
{
    return g_yaw_target;
}

/* 位姿保持，前进为正 */
static int32_t g_pose_target;
static int32_t g_pose_last_position;
static int16_t g_pose_last_base;
static int16_t g_pose_last_turn;
static int32_t g_pose_error;

static int16_t slew(int16_t target, int16_t current, int16_t step)
{
    if (target > current + step) return (int16_t)(current + step);
    if (target < current - step) return (int16_t)(current - step);
    return target;
}

static int32_t pose_position(void)
{
    int32_t left, right;
    Encoder_GetCounts(&left, &right);
    int64_t average = ((int64_t)left + (int64_t)right) / 2;
    if (average > 2147483647LL) average = 2147483647LL;
    if (average < -2147483647LL) average = -2147483647LL;
    return (int32_t)-average;
}

void Control_ResetPoseHold(void)
{
    int32_t position = pose_position();
    g_pose_target = position;
    g_pose_last_position = position;
    g_pose_last_base = 0;
    g_pose_last_turn = 0;
    g_pose_error = 0;
    Control_ResetYawPid();
}

void Control_CapturePoseTarget(void)
{
    Control_ResetPoseHold();
}

int32_t Control_GetPoseError(void)
{
    return g_pose_error;
}

void Control_PoseHoldStep(void)
{
    int32_t position = pose_position();
    int32_t velocity = position - g_pose_last_position;
    g_pose_last_position = position;
    g_pose_error = g_pose_target - position;

    /* 位置P加速度阻尼，输出限幅并缓升 */
    int32_t base_raw = g_pose_error * 3 - velocity * 20;
    if (base_raw > 180) base_raw = 180;
    if (base_raw < -180) base_raw = -180;
    if (g_pose_error > -3 && g_pose_error < 3 &&
        velocity > -2 && velocity < 2) base_raw = 0;

    IMUSnapshot imu;
    IMU_GetSnapshot(&imu);
    float yaw_error = Control_YawError(g_yaw_target, imu.yaw);
    float rate_error = 3.0f * yaw_error - imu.gyro_z;
    int32_t turn_raw = (int32_t)(0.8f * rate_error);
    if (turn_raw > 160) turn_raw = 160;
    if (turn_raw < -160) turn_raw = -160;
    if (yaw_error > -1.2f && yaw_error < 1.2f &&
        imu.gyro_z > -4.0f && imu.gyro_z < 4.0f) turn_raw = 0;

    g_pose_last_base = slew((int16_t)base_raw, g_pose_last_base, 15);
    g_pose_last_turn = slew((int16_t)turn_raw, g_pose_last_turn, 15);

    int16_t left = (int16_t)(g_pose_last_base + g_pose_last_turn);
    int16_t right = (int16_t)(g_pose_last_base - g_pose_last_turn);
    if (left > 220) left = 220;
    if (left < -220) left = -220;
    if (right > 220) right = 220;
    if (right < -220) right = -220;

    if (left == 0 && right == 0) Motor_Stop();
    else {
        Motor_Left(left);
        Motor_Right(right);
    }
}
