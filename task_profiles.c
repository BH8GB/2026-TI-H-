/* 文件 task_profiles.c，用途 任务参数档 */

#include "task_profiles.h"
#include "ball_control.h"
#include "ball_task4.h"
#include "sensor.h"
#include "wheel_speed.h"

/* 第二问只做循迹 */
#define TASK2_WHEEL_KP       12.0f
#define TASK2_WHEEL_KI        0.50f
#define TASK2_TRACK_KP        1.00f

/* 第三问钢球往返 */
static const BallControlParams g_task3_ball_pid = {
    2.40f,
    0.95f,
    0.06f,
    0.75f,
    0.00f,
    140U
};

/* 第四问直线行驶并保持中心 */
#define TASK4_WHEEL_KP       12.0f
#define TASK4_WHEEL_KI        0.50f
static const BallControlParams g_task4_ball_pid = {
    /* 第四问单独提高外环和速度比例，不影响其他任务 */
    2.20f,
    1.10f,
    0.03f,
    0.80f,
    0.00f,
    90U
};
static const BallTask4Config g_task4_motion = {
    100U,
    40U,
    25U,
    6657U,
    350,
    1000
};

/* 第五和第六问使用各自参数，第五问保持中心，第六问保持任意位置 */
static const TaskLapPidProfile g_task5_pid_default = {
    10.5f,
    0.35f,
    0.45f,
    {1.90f, 1.05f, 0.01f, 0.80f, 0.12f, 140U}
};
static const TaskLapPidProfile g_task6_pid_default = {
    10.5f,
    0.35f,
    0.45f,
    {1.90f, 1.05f, 0.01f, 0.80f, 0.12f, 140U}
};

static TaskLapPidProfile g_task5_pid = {
    10.5f,
    0.35f,
    0.45f,
    {1.90f, 1.05f, 0.01f, 0.80f, 0.12f, 140U}
};
static TaskLapPidProfile g_task6_pid = {
    10.5f,
    0.35f,
    0.45f,
    {1.90f, 1.05f, 0.01f, 0.80f, 0.12f, 140U}
};

/* 写入前整组限幅，避免只改半套PID */
static uint8_t task_lap_pid_valid(const TaskLapPidProfile *profile)
{
    if (profile == 0 ||
        !(profile->wheel_kp >= 0.0f && profile->wheel_kp <= 50.0f) ||
        !(profile->wheel_ki >= 0.0f && profile->wheel_ki <= 10.0f) ||
        !(profile->track_kp >= 0.0f && profile->track_kp <= 20.0f) ||
        !(profile->ball.position_kp_per_s >= 0.20f &&
          profile->ball.position_kp_per_s <= 6.00f) ||
        !(profile->ball.velocity_kp_us_per_mm_s >= 0.05f &&
          profile->ball.velocity_kp_us_per_mm_s <= 3.00f) ||
        !(profile->ball.velocity_ki_us_per_mm >= 0.00f &&
          profile->ball.velocity_ki_us_per_mm <= 0.60f) ||
        !(profile->ball.velocity_damping_us_per_mm_s >= 0.00f &&
          profile->ball.velocity_damping_us_per_mm_s <= 0.80f) ||
        !(profile->ball.acceleration_damping_us_per_mm_s2 >= 0.00f &&
          profile->ball.acceleration_damping_us_per_mm_s2 <= 0.30f) ||
        profile->ball.static_comp_us > 220U) {
        return 0U;
    }
    return 1U;
}

/* 一次加载选中任务的完整PID */
static uint8_t task_lap_pid_apply(const TaskLapPidProfile *profile)
{
    if (task_lap_pid_valid(profile) == 0U) return 0U;
    WheelSpeed_SetGains(profile->wheel_kp, profile->wheel_ki);
    Track_SetKp(profile->track_kp);
    return BallControl_SetParams(&profile->ball);
}

uint8_t TaskProfiles_LoadTask2(void)
{
    WheelSpeed_SetGains(TASK2_WHEEL_KP, TASK2_WHEEL_KI);
    Track_SetKp(TASK2_TRACK_KP);
    return 1U;
}

uint8_t TaskProfiles_LoadTask3(void)
{
    return BallControl_SetParams(&g_task3_ball_pid);
}

uint8_t TaskProfiles_LoadTask4(void)
{
    uint8_t pid_ok;
    uint8_t motion_ok;

    pid_ok = TaskProfiles_LoadTask4Pid();
    motion_ok = BallTask4_SetConfig(&g_task4_motion);
    return (uint8_t)(pid_ok && motion_ok);
}

uint8_t TaskProfiles_LoadTask4Pid(void)
{
    WheelSpeed_SetGains(TASK4_WHEEL_KP, TASK4_WHEEL_KI);
    return BallControl_SetParams(&g_task4_ball_pid);
}

uint8_t TaskProfiles_LoadTask5(void)
{
    return task_lap_pid_apply(&g_task5_pid);
}

uint8_t TaskProfiles_LoadTask6(void)
{
    return task_lap_pid_apply(&g_task6_pid);
}

void TaskProfiles_GetTask5Pid(TaskLapPidProfile *profile)
{
    if (profile != 0) *profile = g_task5_pid;
}

void TaskProfiles_GetTask6Pid(TaskLapPidProfile *profile)
{
    if (profile != 0) *profile = g_task6_pid;
}

uint8_t TaskProfiles_SetTask5Pid(const TaskLapPidProfile *profile)
{
    if (task_lap_pid_valid(profile) == 0U) return 0U;
    g_task5_pid = *profile;
    return 1U;
}

uint8_t TaskProfiles_SetTask6Pid(const TaskLapPidProfile *profile)
{
    if (task_lap_pid_valid(profile) == 0U) return 0U;
    g_task6_pid = *profile;
    return 1U;
}

void TaskProfiles_ResetTask5Pid(void)
{
    g_task5_pid = g_task5_pid_default;
}

void TaskProfiles_ResetTask6Pid(void)
{
    g_task6_pid = g_task6_pid_default;
}
