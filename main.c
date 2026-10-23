/* 文件 main.c，用途 主循环和任务调度 */

#include "ti_msp_dl_config.h"
#include "oled.h"
#include "led.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include "imu.h"
#include "control.h"
#include "ball_control.h"
#include "ball_task3.h"
#include "ball_task4.h"
#include "ball_task5.h"
#include "ball_task6.h"
#include "ball_autotune.h"
#include "wheel_speed.h"
#include "task_profiles.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

volatile uint32_t g_sys_tick;  /* 1毫秒系统心跳 */

void SysTick_Handler(void) { g_sys_tick++; }

#define CMD_BUF_SIZE 96U
#define STARTUP_COMMAND_LOCKOUT_MS 800U
static char g_cmd[CMD_BUF_SIZE];
static volatile uint8_t g_cmd_idx;
static volatile uint8_t g_cmd_ready;
static volatile uint8_t g_cmd_overflow;
static volatile uint32_t g_cmd_last_tick;
static uint32_t g_cmd_accept_tick;

typedef enum {
    DRIVE_MANUAL = 0,
    DRIVE_TRACK,
    DRIVE_TASK2,
    DRIVE_YAW_HOLD,
    DRIVE_YAW_TURN,
    DRIVE_POSE_ARM,
    DRIVE_POSE_HOLD,
    DRIVE_TASK4,
    DRIVE_TASK5,
    DRIVE_TASK6
} DriveMode;

static volatile DriveMode g_drive_mode = DRIVE_MANUAL;
static uint32_t g_yaw_start_tick;
static uint8_t g_yaw_settle_count;
static uint8_t g_pose_arm_count;
static int32_t g_pose_arm_last_l;
static int32_t g_pose_arm_last_r;
static uint32_t g_mode_tick;
static uint32_t g_last_motion_cmd_tick;
/* 第二问独立循迹，PA7启动 */
static uint8_t g_task2_running;
/* 第二问结果，0无，1完成，2超时 */
static uint8_t g_task2_done;
static uint32_t g_task2_start_tick;
static uint32_t g_task2_elapsed_ms;
static uint32_t g_task2_last_control_tick;
static int32_t g_task2_start_left;
static int32_t g_task2_start_right;
static int16_t g_task2_distance_mm;
static int16_t g_task2_command_rpm;

#define TASK2_BUTTON_PORT              GPIOA
#define TASK2_BUTTON_PIN               DL_GPIO_PIN_7
#define TASK2_BUTTON_IOMUX             IOMUX_PINCM14
#define TASK2_LAP_DISTANCE_MM         5989L /* 一圈6089毫米，提前100毫米停车 */
#define TASK2_TIMEOUT_MS               20000U
#define TASK2_START_RPM                42
#define TASK2_MAX_RPM                 105
#define TASK2_ACCEL_RPM_PER_S          55
#define TASK2_DECEL_RPM_PER_S          70
#define TASK2_STOP_MARGIN_MM           240L
#define TASK2_WHEEL_DIAMETER_MM        66.57f
#define TASK2_COUNTS_PER_REV           30000.0f
#define TASK2_PI                       3.1415926f
static BallTask3State g_task3_reported_state = BALL_TASK3_IDLE;
static uint8_t g_task3_reported_vision_waiting;
static BallTask4State g_task4_reported_state = BALL_TASK4_IDLE;
static BallTask5State g_task5_reported_state = BALL_TASK5_IDLE;
static BallTask6State g_task6_reported_state = BALL_TASK6_IDLE;

#define START_BUTTON_DEBOUNCE_MS 30U
#define TASK6_BUTTON_REARM_MS   250U
static uint8_t g_start_button_raw;
static uint8_t g_start_button_stable;
static uint32_t g_start_button_change_tick;

#define TASK4_BUTTON_PORT              GPIOA
#define TASK4_BUTTON_PIN               DL_GPIO_PIN_26
#define TASK4_BUTTON_IOMUX             IOMUX_PINCM59
static uint8_t g_task4_button_raw;
static uint8_t g_task4_button_stable;
static uint32_t g_task4_button_change_tick;

/* PB9不是舵机PWM引脚，舵机使用PB8 */
#define TASK5_BUTTON_PORT              GPIOB
#define TASK5_BUTTON_PIN               DL_GPIO_PIN_9
#define TASK5_BUTTON_IOMUX             IOMUX_PINCM26
static uint8_t g_task5_button_raw;
static uint8_t g_task5_button_stable;
static uint32_t g_task5_button_change_tick;

/* PB14为第六问启动键，低有效上拉 */
#define TASK6_BUTTON_PORT              GPIOB
#define TASK6_BUTTON_PIN               DL_GPIO_PIN_14
#define TASK6_BUTTON_IOMUX             IOMUX_PINCM31
static uint8_t g_task6_button_raw;
static uint8_t g_task6_button_stable;
static uint32_t g_task6_button_change_tick;
static uint32_t g_task6_button_last_press_tick;
typedef enum {
    TASK6_SEQUENCE_CAPTURE_TARGET = 0,
    TASK6_SEQUENCE_START_POSITIONING,
    TASK6_SEQUENCE_START_RUN
} Task6SequenceStage;
static Task6SequenceStage g_task6_sequence_stage =
    TASK6_SEQUENCE_CAPTURE_TARGET;
static uint8_t g_task6_ready_reported;
/* 第三次按键可能早于READY，先记住请求 */
static uint8_t g_task6_start_pending;
static uint8_t g_task2_button_raw;
static uint8_t g_task2_button_stable;
static uint32_t g_task2_button_change_tick;

static void task2_stop(void);
static void task2_process(uint32_t now_ms);

void UART_DEBUG_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_DEBUG_INST)) {
    case DL_UART_IIDX_RX: {
        char c = (char)DL_UART_receiveData(UART_DEBUG_INST);
        if (g_sys_tick < g_cmd_accept_tick) {
            g_cmd_idx = 0U;
            g_cmd_ready = 0U;
            g_cmd_overflow = 0U;
            g_cmd[0] = '\0';
            break;
        }
        if (c == '\r' || c == '\n') {
            if (g_cmd_idx != 0U) g_cmd_ready = 1U;
        } else if (c >= ' ' && !g_cmd_ready) {
            if (g_cmd_idx < (uint8_t)(CMD_BUF_SIZE - 1U)) {
                g_cmd[g_cmd_idx++] = c;
                g_cmd[g_cmd_idx] = '\0';
            } else {
                g_cmd_overflow = 1U;
                g_cmd_ready = 1U;
            }
            g_cmd_last_tick = g_sys_tick;
        }
        break;
    }
    default: break;
    }
}

static void bt_send(const char *s) {
    /* 波形输出期间不插入文字状态，避免破坏解析 */
    if (s != 0 && s[0] == '>' &&
        BallAutotune_IsWheeltecCaptureActive() != 0U) {
        return;
    }
    while (*s) {
        while (DL_UART_isBusy(UART_DEBUG_INST) == true);
        DL_UART_Main_transmitData(UART_DEBUG_INST, (uint32_t)*s++);
    }
}
static void oled_line(uint8_t p, const char *s) {
    char l[22]; uint8_t i = 0;
    while (s[i] && i < 21) { l[i] = s[i]; i++; }
    while (i < 21) l[i++] = ' ';
    l[21] = '\0'; OLED_ShowString(p, 0, l);
}

/* OLED只保留任务计时，减少刷新量 */
static void oled_show_task_elapsed(const char *name, uint32_t elapsed_ms)
{
    char line[22];
    unsigned long seconds = (unsigned long)(elapsed_ms / 1000U);
    unsigned long centiseconds =
        (unsigned long)((elapsed_ms % 1000U) / 10U);

    snprintf(line, sizeof(line), "%s %lu.%02lus", name,
             seconds, centiseconds);
    oled_line(7U, line);
}

static int16_t g_ld, g_rd;

static uint8_t is_track_mode(DriveMode mode)
{
    return mode == DRIVE_TRACK;
}

static void disable_speed_loop(uint8_t brake)
{
    WheelSpeed_Disable(brake);
}

/* 第五和第六问共用底盘，切换模式前先中止 */
static void abort_lap_tasks(uint32_t now_ms)
{
    BallTask5_Abort(now_ms);
    BallTask6_Abort(now_ms);
}

static void set_manual_drive(int16_t left, int16_t right)
{
    g_task2_running = 0U;
    abort_lap_tasks(g_sys_tick);
    BallTask4_Abort(g_sys_tick);
    BallTask3_Abort(g_sys_tick);
    disable_speed_loop(1U);
    Track_Stop();
    Control_ResetPoseHold();
    g_drive_mode = DRIVE_MANUAL;
    g_ld = left;
    g_rd = right;
    g_last_motion_cmd_tick = g_sys_tick;
}

static void set_heading_drive(int16_t speed)
{
    g_task2_running = 0U;
    abort_lap_tasks(g_sys_tick);
    BallTask4_Abort(g_sys_tick);
    BallTask3_Abort(g_sys_tick);
    disable_speed_loop(1U);
    Track_Stop();
    Control_ResetPoseHold();
    g_ld = speed;
    g_rd = speed;
    Control_SetYawTarget(IMU_Yaw());
    WheelSpeed_Enable(g_sys_tick);
    g_mode_tick = g_sys_tick;
    g_last_motion_cmd_tick = g_sys_tick;
    g_drive_mode = DRIVE_YAW_HOLD;
}

static void start_yaw_turn(float target_yaw)
{
    g_task2_running = 0U;
    abort_lap_tasks(g_sys_tick);
    BallTask4_Abort(g_sys_tick);
    BallTask3_Abort(g_sys_tick);
    disable_speed_loop(1U);
    Track_Stop();
    Control_ResetPoseHold();
    g_ld = 0;
    g_rd = 0;
    Control_SetYawTarget(target_yaw);
    g_yaw_settle_count = 0;
    g_pose_arm_count = 0;
    g_yaw_start_tick = g_sys_tick;
    g_mode_tick = g_sys_tick;
    g_drive_mode = DRIVE_YAW_TURN;
}

/* PB21低有效按键，消抖后返回按下沿 */
static uint8_t start_button_pressed(uint32_t now_ms)
{
    uint8_t raw = (DL_GPIO_readPins(GPIO_CTRL_PORT,
                                    GPIO_CTRL_BTN_START_PIN) != 0U) ? 1U : 0U;
    if (raw != g_start_button_raw) {
        g_start_button_raw = raw;
        g_start_button_change_tick = now_ms;
    }
    if (raw != g_start_button_stable &&
        (uint32_t)(now_ms - g_start_button_change_tick) >=
            START_BUTTON_DEBOUNCE_MS) {
        g_start_button_stable = raw;
        return (raw == 0U) ? 1U : 0U;
    }
    return 0U;
}

/* PA26低有效按键，消抖后返回按下沿 */
static uint8_t task4_button_pressed(uint32_t now_ms)
{
    uint8_t raw = (DL_GPIO_readPins(TASK4_BUTTON_PORT,
                                    TASK4_BUTTON_PIN) != 0U) ? 1U : 0U;
    if (raw != g_task4_button_raw) {
        g_task4_button_raw = raw;
        g_task4_button_change_tick = now_ms;
    }
    if (raw != g_task4_button_stable &&
        (uint32_t)(now_ms - g_task4_button_change_tick) >=
            START_BUTTON_DEBOUNCE_MS) {
        g_task4_button_stable = raw;
        return (raw == 0U) ? 1U : 0U;
    }
    return 0U;
}

/* PB9低有效按键，消抖后返回按下沿 */
static uint8_t task5_button_pressed(uint32_t now_ms)
{
    uint8_t raw = (DL_GPIO_readPins(TASK5_BUTTON_PORT,
                                    TASK5_BUTTON_PIN) != 0U) ? 1U : 0U;
    if (raw != g_task5_button_raw) {
        g_task5_button_raw = raw;
        g_task5_button_change_tick = now_ms;
    }
    if (raw != g_task5_button_stable &&
        (uint32_t)(now_ms - g_task5_button_change_tick) >=
            START_BUTTON_DEBOUNCE_MS) {
        g_task5_button_stable = raw;
        return (raw == 0U) ? 1U : 0U;
    }
    return 0U;
}

/* PB14低有效按键，消抖后返回按下沿 */
static uint8_t task6_button_pressed(uint32_t now_ms)
{
    uint8_t raw = (DL_GPIO_readPins(TASK6_BUTTON_PORT,
                                    TASK6_BUTTON_PIN) != 0U) ? 1U : 0U;
    if (raw != g_task6_button_raw) {
        g_task6_button_raw = raw;
        g_task6_button_change_tick = now_ms;
    }
    if (raw != g_task6_button_stable &&
        (uint32_t)(now_ms - g_task6_button_change_tick) >=
            START_BUTTON_DEBOUNCE_MS) {
        g_task6_button_stable = raw;
        if (raw == 0U &&
            (uint32_t)(now_ms - g_task6_button_last_press_tick) >=
                TASK6_BUTTON_REARM_MS) {
            g_task6_button_last_press_tick = now_ms;
            return 1U;
        }
    }
    return 0U;
}

/* PA7低有效按键，消抖后返回按下沿 */
static uint8_t task2_button_pressed(uint32_t now_ms)
{
    uint8_t raw = (DL_GPIO_readPins(TASK2_BUTTON_PORT,
                                    TASK2_BUTTON_PIN) != 0U) ? 1U : 0U;
    if (raw != g_task2_button_raw) {
        g_task2_button_raw = raw;
        g_task2_button_change_tick = now_ms;
    }
    if (raw != g_task2_button_stable &&
        (uint32_t)(now_ms - g_task2_button_change_tick) >=
            START_BUTTON_DEBOUNCE_MS) {
        g_task2_button_stable = raw;
        return (raw == 0U) ? 1U : 0U;
    }
    return 0U;
}

/* 第三问启动和急停共用底盘制动流程 */
static void stop_ball_test_motion(void)
{
    g_task2_running = 0U;
    abort_lap_tasks(g_sys_tick);
    BallTask4_Abort(g_sys_tick);
    disable_speed_loop(1U);
    Track_Stop();
    Control_ResetPoseHold();
    Control_ResetYawPid();
    g_ld = 0;
    g_rd = 0;
    g_drive_mode = DRIVE_MANUAL;
    Motor_Stop();
}

/* 加载任务参数后清除滚球控制瞬态 */
static void reset_ball_runtime_for_task_start(void)
{
    BallControl_SetMotionFeedforwardUs(0);
    BallControl_SetFineHold(0U);
    BallControl_SetEnabled(0U);
    /* 急停后清除第六问定位标志，正式运行时重新加载 */
    BallControl_SetTask6PositioningAssist(0U);
}

/* 停稳小车后启动第三问 */
static BallTask3StartResult start_ball_task3_profile(uint8_t restore_profile)
{
    g_task2_running = 0U;
    stop_ball_test_motion();
    if (restore_profile != 0U) (void)TaskProfiles_LoadTask3();
    reset_ball_runtime_for_task_start();
    return BallTask3_Start(g_sys_tick);
}

static BallTask3StartResult start_ball_task3(void)
{
    return start_ball_task3_profile(1U);
}

static BallTask3StartResult start_ball_task3_tuning(void)
{
    return start_ball_task3_profile(0U);
}

static BallTask4StartResult start_ball_task4_profile(uint8_t restore_profile)
{
    g_task2_running = 0U;
    abort_lap_tasks(g_sys_tick);
    BallTask3_Abort(g_sys_tick);
    disable_speed_loop(1U);
    Track_Stop();
    Control_ResetPoseHold();
    Control_ResetYawPid();
    g_ld = 0;
    g_rd = 0;
    g_drive_mode = DRIVE_MANUAL;
    Motor_Stop();
    if (restore_profile != 0U) {
        (void)TaskProfiles_LoadTask4();
    } else {
        /* 第四问保留原命令，但加载独立PID */
        (void)TaskProfiles_LoadTask4Pid();
    }
    reset_ball_runtime_for_task_start();
    BallTask4StartResult result = BallTask4_Start(g_sys_tick);
    if (result == BALL_TASK4_START_OK) {
        g_drive_mode = DRIVE_TASK4;
    }
    return result;
}

static BallTask4StartResult start_ball_task4(void)
{
    return start_ball_task4_profile(1U);
}

static BallTask4StartResult start_ball_task4_tuning(void)
{
    return start_ball_task4_profile(0U);
}

static BallTask5StartResult start_ball_task5(void)
{
    g_task2_running = 0U;
    BallTask6_Abort(g_sys_tick);
    BallTask4_Abort(g_sys_tick);
    BallTask3_Abort(g_sys_tick);
    disable_speed_loop(1U);
    Track_Stop();
    Control_ResetPoseHold();
    Control_ResetYawPid();
    g_ld = 0;
    g_rd = 0;
    g_drive_mode = DRIVE_MANUAL;
    Motor_Stop();
    (void)TaskProfiles_LoadTask5();
    reset_ball_runtime_for_task_start();
    BallTask5StartResult result = BallTask5_Start(g_sys_tick);
    if (result == BALL_TASK5_START_OK) {
        g_drive_mode = DRIVE_TASK5;
    }
    return result;
}

/* 第六问前两次按键只动摆杆，底盘保持制动 */
static void task6_hold_chassis_stopped(void)
{
    g_task2_running = 0U;
    BallTask5_Abort(g_sys_tick);
    BallTask4_Abort(g_sys_tick);
    BallTask3_Abort(g_sys_tick);
    disable_speed_loop(1U);
    Track_Stop();
    Control_ResetPoseHold();
    Control_ResetYawPid();
    g_ld = 0;
    g_rd = 0;
    g_drive_mode = DRIVE_MANUAL;
    Motor_Stop();
}

static void task6_reset_sequence_state(void)
{
    g_task6_sequence_stage = TASK6_SEQUENCE_CAPTURE_TARGET;
    g_task6_ready_reported = 0U;
    g_task6_start_pending = 0U;
}

/* 赛前步骤失败时不推进阶段 */
static void report_task6_setup_error(BallTask6SetupResult result)
{
    switch (result) {
    case BALL_TASK6_SETUP_NO_VISION:
        bt_send(">ERR T6 NO VISION\r\n");
        break;
    case BALL_TASK6_SETUP_POSITION_UNSAFE:
        bt_send(">ERR T6 POSITION UNSAFE\r\n");
        break;
    case BALL_TASK6_SETUP_BALL_MOVING:
        bt_send(">ERR T6 HOLD BALL STILL\r\n");
        break;
    case BALL_TASK6_SETUP_NO_TARGET:
        bt_send(">ERR T6 CAPTURE TARGET FIRST\r\n");
        break;
    default:
        break;
    }
}

static void report_task6_setup_stage(const char *stage, int16_t target_mm)
{
    char reply[64];
    snprintf(reply, sizeof(reply), ">T6 %s TARGET:%+dMM\r\n",
             stage, target_mm);
    bt_send(reply);
}

static void report_task6_position_ready(uint32_t now_ms)
{
    BallControlStatus ball;
    char reply[64];

    BallControl_GetStatus(&ball, now_ms);
    snprintf(reply, sizeof(reply), ">T6 READY X:%+d T:%+dMM\r\n",
             ball.position_mm, BallTask6_GetCapturedTargetMm());
    bt_send(reply);
}

static BallTask6StartResult start_ball_task6(void)
{
    g_task2_running = 0U;
    BallTask5_Abort(g_sys_tick);
    BallTask4_Abort(g_sys_tick);
    BallTask3_Abort(g_sys_tick);
    disable_speed_loop(1U);
    Track_Stop();
    Control_ResetPoseHold();
    Control_ResetYawPid();
    g_ld = 0;
    g_rd = 0;
    g_drive_mode = DRIVE_MANUAL;
    Motor_Stop();
    (void)TaskProfiles_LoadTask6();
    reset_ball_runtime_for_task_start();
    BallTask6StartResult result = BallTask6_Start(g_sys_tick);
    if (result == BALL_TASK6_START_OK) {
        g_drive_mode = DRIVE_TASK6;
    }
    return result;
}

/* READY后执行已提交的第六问启动 */
static uint8_t task6_start_run_when_ready(void)
{
    BallTask6StartResult result;
    int16_t target_mm;

    if (BallTask6_IsPositionReady() == 0U) return 0U;
    target_mm = BallTask6_GetCapturedTargetMm();
    g_task6_start_pending = 0U;
    g_task6_reported_state = BALL_TASK6_IDLE;
    BallAutotune_StartTask6Capture(g_sys_tick);
    result = start_ball_task6();
    if (result == BALL_TASK6_START_OK) {
        task6_reset_sequence_state();
        report_task6_setup_stage("3/3 RUN", target_mm);
    } else {
        g_task6_sequence_stage = TASK6_SEQUENCE_START_POSITIONING;
        g_task6_ready_reported = 0U;
        if (result == BALL_TASK6_START_NO_VISION) {
            bt_send(">ERR T6 NO VISION\r\n");
        } else {
            bt_send(">ERR T6 POSITION UNSAFE\r\n");
        }
    }
    return 1U;
}

static void stop_ball_task4(void)
{
    g_task2_running = 0U;
    BallTask4_Abort(g_sys_tick);
    g_ld = 0;
    g_rd = 0;
    g_drive_mode = DRIVE_MANUAL;
    Motor_Stop();
}

static void stop_ball_task5(void)
{
    g_task2_running = 0U;
    BallTask5_Abort(g_sys_tick);
    g_ld = 0;
    g_rd = 0;
    g_drive_mode = DRIVE_MANUAL;
    Motor_Stop();
}

static void stop_ball_task6(void)
{
    g_task2_running = 0U;
    BallTask6_Abort(g_sys_tick);
    task6_reset_sequence_state();
    g_ld = 0;
    g_rd = 0;
    g_drive_mode = DRIVE_MANUAL;
    Motor_Stop();
}

/* 启动第二问，清零计数并开启循迹和速度环 */
static void task2_start(uint32_t now_ms)
{
    abort_lap_tasks(now_ms);
    BallTask4_Abort(now_ms);
    BallTask3_Abort(now_ms);
    BallControl_SetEnabled(0U);
    Track_Stop();
    WheelSpeed_Disable(1U);
    Encoder_LeftClear();
    Encoder_RightClear();
    Encoder_GetCounts(&g_task2_start_left, &g_task2_start_right);
    g_task2_start_tick = now_ms;
    g_task2_elapsed_ms = 0U;
    g_task2_last_control_tick = now_ms - 20U;
    g_task2_distance_mm = 0;
    g_task2_command_rpm = TASK2_START_RPM;
    g_task2_running = 1U;
    g_task2_done = 0U;
    (void)TaskProfiles_LoadTask2();
    Track_SetBaseSpeed(TASK2_START_RPM);
    Track_Start();
    WheelSpeed_Enable(now_ms);
    WheelSpeed_SetTargets(0, 0);
    g_drive_mode = DRIVE_TASK2;
    bt_send(">T2 START PA7\r\n");
}

/* 停止第二问，保留耗时和里程 */
static void task2_stop(void)
{
    Track_Stop();
    WheelSpeed_SetTargets(0, 0);
    WheelSpeed_Disable(1U);
    g_task2_running = 0U;
    g_drive_mode = DRIVE_MANUAL;
    g_ld = 0;
    g_rd = 0;
    Motor_Stop();
}

/* 第二问循迹，接近终点时线性减速 */
static void task2_process(uint32_t now_ms)
{
    int32_t left_count, right_count;
    int64_t left_delta, right_delta;
    int32_t avg_counts;
    int32_t distance_mm;
    int16_t target_l, target_r;
    int32_t remaining_mm;
    int32_t step;
    float circumference_mm = TASK2_PI * TASK2_WHEEL_DIAMETER_MM;

    if (g_task2_running == 0U) return;
    if ((uint32_t)(now_ms - g_task2_last_control_tick) < 20U) return;
    g_task2_last_control_tick = now_ms;
    g_task2_elapsed_ms = now_ms - g_task2_start_tick;
    Encoder_GetCounts(&left_count, &right_count);
    left_delta = (int64_t)left_count - g_task2_start_left;
    right_delta = (int64_t)right_count - g_task2_start_right;
    avg_counts = (int32_t)((left_delta + right_delta) / 2LL);
    distance_mm = (int32_t)((float)avg_counts * circumference_mm /
                            TASK2_COUNTS_PER_REV);
    if (distance_mm < 0) distance_mm = 0;
    if (distance_mm > 32767) distance_mm = 32767;
    g_task2_distance_mm = (int16_t)distance_mm;
    remaining_mm = TASK2_LAP_DISTANCE_MM - distance_mm;

    if (g_task2_elapsed_ms >= TASK2_TIMEOUT_MS || remaining_mm <= 0) {
        g_task2_elapsed_ms = now_ms - g_task2_start_tick;
        g_task2_done = (remaining_mm <= 0) ? 1U : 2U;
        task2_stop();
        bt_send(g_task2_done == 1U ? ">T2 DONE\r\n" :
                                      ">T2 TIMEOUT\r\n");
        return;
    }

    if (remaining_mm <= TASK2_STOP_MARGIN_MM) {
        step = (TASK2_DECEL_RPM_PER_S * 20 + 999) / 1000;
        if (step < 1) step = 1;
        g_task2_command_rpm -= (int16_t)step;
        if (g_task2_command_rpm < 18) g_task2_command_rpm = 18;
    } else if (g_task2_command_rpm < TASK2_MAX_RPM) {
        step = (TASK2_ACCEL_RPM_PER_S * 20 + 999) / 1000;
        if (step < 1) step = 1;
        g_task2_command_rpm += (int16_t)step;
        if (g_task2_command_rpm > TASK2_MAX_RPM) {
            g_task2_command_rpm = TASK2_MAX_RPM;
        }
    }

    Track_SetBaseSpeed(g_task2_command_rpm);
    if (Track_ComputeWheelTargets(now_ms, &target_l, &target_r) == 0U) {
        WheelSpeed_SetTargets(0, 0);
    } else {
        /* 第二问只允许向前，避免内侧轮反转 */
        if (target_l < 0) target_l = 0;
        if (target_r < 0) target_r = 0;
        WheelSpeed_SetTargets(target_l, target_r);
    }
}

static void report_ball_task4_state(void)
{
    BallTask4Status status;
    char reply[64];
    BallTask4_GetStatus(&status);
    if (status.state == g_task4_reported_state) return;
    g_task4_reported_state = status.state;
    switch (status.state) {
    case BALL_TASK4_ACCEL:
        bt_send(">T4 ACCEL TO B\r\n");
        break;
    case BALL_TASK4_CRUISE:
        bt_send(">T4 CRUISE\r\n");
        break;
    case BALL_TASK4_DECEL:
        bt_send(">T4 DECEL\r\n");
        break;
    case BALL_TASK4_SETTLE:
        bt_send(">T4 SETTLE\r\n");
        break;
    case BALL_TASK4_DONE:
        snprintf(reply, sizeof(reply), ">T4 DONE %luMS D:%dMM X:%dMM\r\n",
                 (unsigned long)status.elapsed_ms, status.distance_mm,
                 status.ball_position_mm);
        bt_send(reply);
        break;
    case BALL_TASK4_ABORTED:
        bt_send(">T4 ABORTED\r\n");
        break;
    case BALL_TASK4_FAULT_NO_VISION:
        bt_send(">T4 FAULT VISION\r\n");
        break;
    case BALL_TASK4_FAULT_EDGE:
        bt_send(">T4 FAULT BALL EDGE\r\n");
        break;
    case BALL_TASK4_FAULT_TIMEOUT:
        bt_send(">T4 FAULT TIMEOUT\r\n");
        break;
    default:
        break;
    }
}

static void report_ball_task5_state(void)
{
    BallTask5Status status;
    char reply[72];
    BallTask5_GetStatus(&status);
    if (status.state == g_task5_reported_state) return;
    g_task5_reported_state = status.state;
    switch (status.state) {
    case BALL_TASK5_ACCEL:
        bt_send(">T5 ACCEL\r\n");
        break;
    case BALL_TASK5_TRACK:
        bt_send(">T5 TRACK\r\n");
        break;
    case BALL_TASK5_DECEL:
        bt_send(">T5 DECEL\r\n");
        break;
    case BALL_TASK5_SETTLE:
        bt_send(">T5 SETTLE\r\n");
        break;
    case BALL_TASK5_DONE:
        snprintf(reply, sizeof(reply), ">T5 DONE %luMS D:%dMM X:%d/%dMM\r\n",
                 (unsigned long)status.elapsed_ms, status.distance_mm,
                 status.ball_position_mm, status.target_position_mm);
        bt_send(reply);
        break;
    case BALL_TASK5_ABORTED:
        bt_send(">T5 ABORTED\r\n");
        break;
    case BALL_TASK5_FAULT_NO_VISION:
        bt_send(">T5 FAULT VISION\r\n");
        break;
    case BALL_TASK5_FAULT_NOT_CENTERED:
        bt_send(">T5 FAULT PLACE BALL AT O\r\n");
        break;
    case BALL_TASK5_FAULT_NO_LINE:
        bt_send(">T5 FAULT LINE\r\n");
        break;
    case BALL_TASK5_FAULT_EDGE:
        bt_send(">T5 FAULT BALL EDGE\r\n");
        break;
    case BALL_TASK5_FAULT_TIMEOUT:
        bt_send(">T5 FAULT TIMEOUT\r\n");
        break;
    default:
        break;
    }
}

static void report_ball_task6_state(void)
{
    BallTask6Status status;
    char reply[72];
    BallTask6_GetStatus(&status);
    if (status.state == g_task6_reported_state) return;
    g_task6_reported_state = status.state;
    switch (status.state) {
    case BALL_TASK6_ACCEL:
        bt_send(">T6 ACCEL\r\n");
        break;
    case BALL_TASK6_TRACK:
        bt_send(">T6 TRACK\r\n");
        break;
    case BALL_TASK6_DECEL:
        bt_send(">T6 DECEL\r\n");
        break;
    case BALL_TASK6_SETTLE:
        bt_send(">T6 SETTLE\r\n");
        break;
    case BALL_TASK6_DONE:
        snprintf(reply, sizeof(reply), ">T6 DONE %luMS D:%dMM X:%d/%dMM\r\n",
                 (unsigned long)status.elapsed_ms, status.distance_mm,
                 status.ball_position_mm, status.target_position_mm);
        bt_send(reply);
        break;
    case BALL_TASK6_ABORTED:
        bt_send(">T6 ABORTED\r\n");
        break;
    case BALL_TASK6_FAULT_NO_VISION:
        bt_send(">T6 FAULT VISION\r\n");
        break;
    case BALL_TASK6_FAULT_START_POSITION:
        bt_send(">T6 FAULT POSITION UNSAFE\r\n");
        break;
    case BALL_TASK6_FAULT_NO_LINE:
        bt_send(">T6 FAULT LINE\r\n");
        break;
    case BALL_TASK6_FAULT_EDGE:
        bt_send(">T6 FAULT BALL EDGE\r\n");
        break;
    case BALL_TASK6_FAULT_TIMEOUT:
        bt_send(">T6 FAULT TIMEOUT\r\n");
        break;
    default:
        break;
    }
}

/* 状态变化时只发送一次结果 */
static void report_ball_task3_state(void)
{
    BallTask3Status status;
    char reply[48];
    BallTask3_GetStatus(&status);
    if (status.vision_waiting != g_task3_reported_vision_waiting) {
        g_task3_reported_vision_waiting = status.vision_waiting;
        if (status.vision_waiting != 0U) {
            bt_send(">T3 WAIT VISION\r\n");
        } else if (BallTask3_IsRunning() != 0U) {
            bt_send(">T3 VISION RECOVERED\r\n");
        }
    }
    if (status.state == g_task3_reported_state) return;
    g_task3_reported_state = status.state;

    switch (status.state) {
    case BALL_TASK3_MOVE_PLUS_5:
        bt_send(">T3 MOVE +50MM\r\n");
        break;
    case BALL_TASK3_MOVE_MINUS_5:
        sprintf(reply, ">T3 PLUS OK X:%dMM\r\n", status.plus_position_mm);
        bt_send(reply);
        break;
    case BALL_TASK3_DONE:
        sprintf(reply, ">T3 DONE %luMS X:%dMM\r\n",
                (unsigned long)status.elapsed_ms, status.final_position_mm);
        bt_send(reply);
        break;
    case BALL_TASK3_ABORTED:
        bt_send(">T3 ABORTED\r\n");
        break;
    case BALL_TASK3_FAULT_NO_VISION:
        bt_send(">T3 FAULT VISION\r\n");
        break;
    case BALL_TASK3_FAULT_NOT_CENTERED:
        bt_send(">T3 FAULT PLACE BALL AT O\r\n");
        break;
    case BALL_TASK3_FAULT_TIMEOUT:
        bt_send(">T3 FAULT TIMEOUT\r\n");
        break;
    case BALL_TASK3_FAULT_EDGE:
        bt_send(">T3 FAULT EDGE ESTOP\r\n");
        break;
    default:
        break;
    }
}

static uint8_t parse_float_arg(const char *text, float *value)
{
    char *end;
    if (text[0] == '\0') return 0U;
    float parsed = strtof(text, &end);
    if (end == text || *end != '\0') return 0U;
    /* 拒绝NaN和超大输入 */
    if (parsed != parsed || parsed < -10000.0f || parsed > 10000.0f) return 0U;
    *value = parsed;
    return 1U;
}

static uint8_t parse_uint_arg(const char *text, int16_t *value)
{
    uint32_t parsed = 0U;
    if (text[0] == '\0') {
        *value = 0;
        return 1U;
    }
    while (*text) {
        if (*text < '0' || *text > '9') return 0U;
        parsed = parsed * 10U + (uint32_t)(*text - '0');
        if (parsed > (uint32_t)(PWM_MAX - PWM_DEAD)) return 0U;
        text++;
    }
    *value = (int16_t)parsed;
    return 1U;
}

static uint8_t parse_rpm_arg(const char *text, int16_t *value)
{
    uint32_t parsed = 0U;
    if (text[0] == '\0') return 0U;
    while (*text) {
        if (*text < '0' || *text > '9') return 0U;
        parsed = parsed * 10U + (uint32_t)(*text - '0');
        if (parsed > WHEEL_SPEED_MAX_RPM) return 0U;
        text++;
    }
    *value = (int16_t)parsed;
    return 1U;
}

/* 钢球目标允许负数，单独解析并限幅 */
static uint8_t parse_int16_range(const char *text, int16_t minimum,
                                 int16_t maximum, int16_t *value)
{
    char *end;
    long parsed;
    if (text[0] == '\0') return 0U;
    parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < minimum || parsed > maximum) {
        return 0U;
    }
    *value = (int16_t)parsed;
    return 1U;
}

/* 浮点PID按千分值回传 */
static int32_t pid_milli(float value)
{
    return (int32_t)(value * 1000.0f + 0.5f);
}

/* 回传第五和第六问PID和状态 */
static void send_lap_pid_status(uint8_t task_id)
{
    TaskLapPidProfile pid;
    uint8_t state;
    int16_t target_mm;
    int16_t position_mm;
    char reply[160];

    if (task_id == 5U) {
        BallTask5Status status;
        TaskProfiles_GetTask5Pid(&pid);
        BallTask5_GetStatus(&status);
        state = (uint8_t)status.state;
        target_mm = status.target_position_mm;
        position_mm = status.ball_position_mm;
    } else {
        BallTask6Status status;
        TaskProfiles_GetTask6Pid(&pid);
        BallTask6_GetStatus(&status);
        state = (uint8_t)status.state;
        target_mm = status.target_position_mm;
        position_mm = status.ball_position_mm;
    }

    snprintf(reply, sizeof(reply),
             ">T%uPID WP:%ld WI:%ld TK:%ld BP:%ld BV:%ld BI:%ld BD:%ld BA:%ld BC:%u S:%u X:%d/%d\r\n",
             (unsigned)task_id,
             (long)pid_milli(pid.wheel_kp),
             (long)pid_milli(pid.wheel_ki),
             (long)pid_milli(pid.track_kp),
             (long)pid_milli(pid.ball.position_kp_per_s),
             (long)pid_milli(pid.ball.velocity_kp_us_per_mm_s),
             (long)pid_milli(pid.ball.velocity_ki_us_per_mm),
             (long)pid_milli(pid.ball.velocity_damping_us_per_mm_s),
             (long)pid_milli(pid.ball.acceleration_damping_us_per_mm_s2),
             (unsigned)pid.ball.static_comp_us,
             (unsigned)state, (int)position_mm, (int)target_mm);
    bt_send(reply);
}

/* 第五和第六问PID命令，p v i d a c为滚球参数，w j为轮速PI，k为循迹比例 */
static uint8_t handle_lap_pid_command(uint8_t task_id, const char *sub)
{
    TaskLapPidProfile pid;
    float value;
    int16_t integer;
    uint8_t recognized = 1U;
    uint8_t set_ok;

    if (sub[0] == 's' && sub[1] == '\0') {
        send_lap_pid_status(task_id);
        return 1U;
    }

    if (BallTask5_IsRunning() != 0U || BallTask6_IsRunning() != 0U ||
        BallTask6_IsPositioning() != 0U) {
        bt_send(task_id == 5U ? ">ERR T5 PID BUSY\r\n" :
                               ">ERR T6 PID BUSY\r\n");
        return 1U;
    }

    if (sub[0] == 'r' && sub[1] == '\0') {
        if (task_id == 5U) TaskProfiles_ResetTask5Pid();
        else TaskProfiles_ResetTask6Pid();
        send_lap_pid_status(task_id);
        return 1U;
    }

    if (task_id == 5U) TaskProfiles_GetTask5Pid(&pid);
    else TaskProfiles_GetTask6Pid(&pid);

    switch (sub[0]) {
    case 'p':
        if (parse_float_arg(sub + 1, &value) != 0U) {
            pid.ball.position_kp_per_s = value;
        } else recognized = 0U;
        break;
    case 'v':
        if (parse_float_arg(sub + 1, &value) != 0U) {
            pid.ball.velocity_kp_us_per_mm_s = value;
        } else recognized = 0U;
        break;
    case 'i':
        if (parse_float_arg(sub + 1, &value) != 0U) {
            pid.ball.velocity_ki_us_per_mm = value;
        } else recognized = 0U;
        break;
    case 'd':
        if (parse_float_arg(sub + 1, &value) != 0U) {
            pid.ball.velocity_damping_us_per_mm_s = value;
        } else recognized = 0U;
        break;
    case 'a':
        if (parse_float_arg(sub + 1, &value) != 0U) {
            pid.ball.acceleration_damping_us_per_mm_s2 = value;
        } else recognized = 0U;
        break;
    case 'c':
        if (parse_int16_range(sub + 1, 0, 220, &integer) != 0U) {
            pid.ball.static_comp_us = (uint16_t)integer;
        } else recognized = 0U;
        break;
    case 'w':
        if (parse_float_arg(sub + 1, &value) != 0U) {
            pid.wheel_kp = value;
        } else recognized = 0U;
        break;
    case 'j':
        if (parse_float_arg(sub + 1, &value) != 0U) {
            pid.wheel_ki = value;
        } else recognized = 0U;
        break;
    case 'k':
        if (parse_float_arg(sub + 1, &value) != 0U) {
            pid.track_kp = value;
        } else recognized = 0U;
        break;
    default:
        recognized = 0U;
        break;
    }

    if (recognized == 0U) return 0U;
    set_ok = (task_id == 5U) ? TaskProfiles_SetTask5Pid(&pid) :
                               TaskProfiles_SetTask6Pid(&pid);
    if (set_ok == 0U) {
        bt_send(task_id == 5U ? ">ERR T5 PID RANGE\r\n" :
                               ">ERR T6 PID RANGE\r\n");
    } else {
        send_lap_pid_status(task_id);
    }
    return 1U;
}

static uint8_t ball_cmd_exec(const char *c)
{
    BallAutotuneAction action = BallAutotune_HandleCommand(c, bt_send);
    if (action == BALL_AUTOTUNE_NOT_HANDLED) return 0U;

    if (action == BALL_AUTOTUNE_STOP_TRIAL) {
        stop_ball_task5();
        stop_ball_task6();
        stop_ball_task4();
        BallTask3_Abort(g_sys_tick);
        stop_ball_test_motion();
        BallControl_EStop(BALL_CONTROL_SAFETY_NONE);
        BallAutotune_SetTrialActive(0U);
        bt_send(">BSTOP OK\r\n");
    } else if (action == BALL_AUTOTUNE_START_TRIAL) {
        BallTask3StartResult result;
        g_task3_reported_state = BALL_TASK3_IDLE;
        result = start_ball_task3_tuning();
        if (result == BALL_TASK3_START_OK) {
            BallAutotune_SetTrialActive(1U);
            bt_send(">BTRIAL OK\r\n");
        } else if (result == BALL_TASK3_START_NO_VISION) {
            BallAutotune_SetTrialActive(0U);
            bt_send(">BERR NO VISION\r\n");
        } else {
            BallAutotune_SetTrialActive(0U);
            bt_send(">BERR PLACE BALL AT O\r\n");
        }
    }
    return 1U;
}

static void cmd_exec(const char *c)
{
    if (!c[0]) return;
    if (ball_cmd_exec(c) != 0U) return;
    if (BallAutotune_IsTrialActive() != 0U) {
        bt_send(">BERR AUTOTUNE ACTIVE USE BSTOP\r\n");
        return;
    }
    int16_t v = 0;
    int16_t rpm = 0;
    uint8_t int_arg_ok = parse_uint_arg(c + 1, &v);
    uint8_t rpm_arg_ok = parse_rpm_arg(c + 1, &rpm);

    switch (c[0]) {
    case 'f': if (rpm_arg_ok) { set_heading_drive(rpm); bt_send(">OK f RPM HOLD\r\n"); } else bt_send(">ERR f RPM 1-300\r\n"); break;
    case 'b': if (rpm_arg_ok) { set_heading_drive(-rpm); bt_send(">OK b RPM HOLD\r\n"); } else bt_send(">ERR b RPM 1-300\r\n"); break;
    case 'l': if (int_arg_ok) { set_manual_drive(-v, v); bt_send(">OK l\r\n"); } else bt_send(">ERR l arg\r\n"); break;
    case 'r': if (int_arg_ok) { set_manual_drive(v, -v); bt_send(">OK r\r\n"); } else bt_send(">ERR r arg\r\n"); break;
    case 's': set_manual_drive(0, 0); Motor_Stop(); bt_send(">OK s\r\n"); break;
    case 't':
        abort_lap_tasks(g_sys_tick);
        BallTask4_Abort(g_sys_tick);
        BallTask3_Abort(g_sys_tick);
        disable_speed_loop(1U);
        Track_Stop();
        Control_ResetYawPid();
        Track_Start();
        WheelSpeed_Enable(g_sys_tick);
        g_mode_tick = g_sys_tick;
        g_drive_mode = DRIVE_TRACK;
        g_ld = 0;
        g_rd = 0;
        bt_send(">TRK ON\r\n");
        break;
    case 'o': if (rpm_arg_ok) { Track_SetBaseSpeed(rpm); bt_send(">OK track RPM\r\n"); } else bt_send(">ERR o RPM 1-300\r\n"); break;
    case 'k': if (int_arg_ok) { Track_SetKp((float)v); bt_send(">OK kp\r\n"); } else bt_send(">ERR k arg\r\n"); break;
    case 'u': {
        float fv;
        if (parse_float_arg(c + 1, &fv) && fv >= 0.0f && fv <= 50.0f) { WheelSpeed_SetGains(fv, -1.0f); bt_send(">OK spdP\r\n"); }
        else bt_send(">ERR u arg\r\n");
        break;
    }
    case 'i': {
        float fv;
        if (parse_float_arg(c + 1, &fv) && fv >= 0.0f && fv <= 10.0f) { WheelSpeed_SetGains(-1.0f, fv); bt_send(">OK spdI\r\n"); }
        else bt_send(">ERR i arg\r\n");
        break;
    }
    case 'q': {
        char b[24];
        sprintf(b, ">R:%ld\r\n", (long)Encoder_RightGetCount()); bt_send(b);
        sprintf(b, ">L:%ld\r\n", (long)Encoder_LeftGetCount());  bt_send(b);
        break;
    }
    case 'a':
        abort_lap_tasks(g_sys_tick);
        BallTask4_Abort(g_sys_tick);
        BallTask3_Abort(g_sys_tick);
        if (c[1] == '1' && c[2] == '\0') {
            BallControl_SetEnabled(1U);
            bt_send(">BALL CTRL ON\r\n");
        } else if (c[1] == '0' && c[2] == '\0') {
            BallControl_SetEnabled(0U);
            bt_send(">BALL CTRL OFF\r\n");
        } else {
            bt_send(">ERR a0/a1\r\n");
        }
        break;
    case 'p': {
        int16_t target_mm;
        if (parse_int16_range(c + 1, -120, 120, &target_mm)) {
            abort_lap_tasks(g_sys_tick);
            BallTask4_Abort(g_sys_tick);
            BallTask3_Abort(g_sys_tick);
            BallControl_SetTargetMm(target_mm);
            BallControl_SetEnabled(1U);
            bt_send(">OK BALL TARGET MM\r\n");
        } else {
            bt_send(">ERR p -120..120\r\n");
        }
        break;
    }
    case 'n': {
        int16_t neutral_us;
        if (parse_int16_range(c + 1, 1100, 1900, &neutral_us)) {
            abort_lap_tasks(g_sys_tick);
            BallTask4_Abort(g_sys_tick);
            BallTask3_Abort(g_sys_tick);
            BallControl_SetNeutralUs((uint16_t)neutral_us);
            bt_send(">OK BALL NEUTRAL\r\n");
        } else {
            bt_send(">ERR n 1100..1900\r\n");
        }
        break;
    }
    case 'c': {
        int16_t compensation_us;
            if (parse_int16_range(c + 1, 0, 220, &compensation_us)) {
                BallControl_SetStaticCompUs((uint16_t)compensation_us);
                bt_send(">OK BALL STATIC COMP\r\n");
            } else {
                bt_send(">ERR c 0..220US\r\n");
            }
            break;
        }
    case '2': {
        if (c[1] == '\0') {
            task2_start(g_sys_tick);
        } else if (c[1] == 'x' && c[2] == '\0') {
            task2_stop();
            bt_send(">T2 STOP UART\r\n");
        } else {
            bt_send(">ERR 2/2x\r\n");
        }
        break;
    }
    case '3': {
        if (c[1] == '\0') {
            abort_lap_tasks(g_sys_tick);
            BallTask4_Abort(g_sys_tick);
            BallTask3StartResult result = start_ball_task3();
            if (result == BALL_TASK3_START_NO_VISION) {
                bt_send(">ERR T3 NO VISION\r\n");
            } else if (result == BALL_TASK3_START_NOT_CENTERED) {
                bt_send(">ERR T3 BALL NOT AT O\r\n");
            }
        } else if (c[1] == 'x' && c[2] == '\0') {
            BallTask3_Abort(g_sys_tick);
        } else {
            bt_send(">ERR 3/3x\r\n");
        }
        break;
    }
    case '4': {
        if (c[1] == '\0') {
            abort_lap_tasks(g_sys_tick);
            BallTask4StartResult result;
            g_task4_reported_state = BALL_TASK4_IDLE;
            /* 启动采集先于校验，失败原因也能显示 */
            BallAutotune_StartTask4Capture(g_sys_tick);
            result = start_ball_task4_tuning();
            if (result == BALL_TASK4_START_NO_VISION) {
                bt_send(">ERR T4 NO VISION\r\n");
            } else if (result == BALL_TASK4_START_NOT_CENTERED) {
                bt_send(">ERR T4 BALL NOT AT O\r\n");
            }
        } else if (c[1] == 'x' && c[2] == '\0') {
            stop_ball_task4();
        } else if (c[1] == 'v') {
            int16_t value;
            if (parse_int16_range(c + 2, 60, 160, &value) != 0U &&
                BallTask4_SetMaxRpm((uint16_t)value) != 0U) {
                bt_send(">OK T4 MAX RPM\r\n");
            } else bt_send(">ERR 4v 60..160 OR BUSY\r\n");
        } else if (c[1] == 'a') {
            int16_t value;
            if (parse_int16_range(c + 2, 20, 100, &value) != 0U &&
                BallTask4_SetRampRpmPerS((uint16_t)value) != 0U) {
                bt_send(">OK T4 ACCEL RPM/S\r\n");
            } else bt_send(">ERR 4a 20..100 OR BUSY\r\n");
        } else if (c[1] == 'd') {
            int16_t value;
            if (parse_int16_range(c + 2, 20, 80, &value) != 0U &&
                BallTask4_SetDecelRpmPerS((uint16_t)value) != 0U) {
                bt_send(">OK T4 DECEL RPM/S\r\n");
            } else bt_send(">ERR 4d 20..80 OR BUSY\r\n");
        } else if (c[1] == 'w') {
            float diameter_mm;
            uint16_t diameter_hundredth;
            if (parse_float_arg(c + 2, &diameter_mm) != 0U &&
                diameter_mm >= 50.0f && diameter_mm <= 100.0f) {
                diameter_hundredth =
                    (uint16_t)(diameter_mm * 100.0f + 0.5f);
                if (BallTask4_SetWheelDiameterHundredthMm(
                        diameter_hundredth) != 0U) {
                    bt_send(">OK T4 WHEEL MM\r\n");
                } else bt_send(">ERR T4 BUSY\r\n");
            } else bt_send(">ERR 4w 50.0..100.0\r\n");
        } else if (c[1] == 'f') {
            float gain;
            int16_t gain_milli;
            if (parse_float_arg(c + 2, &gain) != 0U &&
                gain >= -1.0f && gain <= 1.0f) {
                gain_milli = (int16_t)(gain * 1000.0f +
                    ((gain >= 0.0f) ? 0.5f : -0.5f));
                if (BallTask4_SetFeedforwardMilli(gain_milli) != 0U) {
                    bt_send(">OK T4 ACCEL FF\r\n");
                } else bt_send(">ERR T4 BUSY\r\n");
            } else bt_send(">ERR 4f -1.0..1.0\r\n");
        } else if (c[1] == 'g') {
            float gain;
            int16_t gain_milli;
            if (parse_float_arg(c + 2, &gain) != 0U &&
                gain >= 0.0f && gain <= 2.0f) {
                gain_milli = (int16_t)(gain * 1000.0f + 0.5f);
                if (BallTask4_SetDecelFeedforwardMilli(gain_milli) != 0U) {
                    bt_send(">OK T4 DECEL FF\r\n");
                } else bt_send(">ERR T4 BUSY\r\n");
            } else bt_send(">ERR 4g 0.0..2.0\r\n");
        } else if (c[1] == 's' && c[2] == '\0') {
            BallTask4Config config;
            BallTask4Status status;
            char reply[128];
            BallTask4_GetConfig(&config);
            BallTask4_GetStatus(&status);
            snprintf(reply, sizeof(reply),
                     ">T4 S:%u D:%d V:%u UP:%u DN:%u W:%u.%02u FA:%d FD:%d\r\n",
                     (unsigned)status.state, (int)status.distance_mm,
                     (unsigned)config.max_rpm,
                     (unsigned)config.ramp_rpm_per_s,
                     (unsigned)config.decel_rpm_per_s,
                     (unsigned)(config.wheel_diameter_hundredth_mm / 100U),
                     (unsigned)(config.wheel_diameter_hundredth_mm % 100U),
                     (int)config.feedforward_milli_us_per_mm_s2,
                     (int)config.decel_feedforward_milli_us_per_mm_s2);
            bt_send(reply);
        } else {
            bt_send(">ERR 4/4x/4v/4a/4d/4w/4f/4g/4s\r\n");
        }
        break;
    }
    case '5': {
        if (c[1] == '\0') {
            BallTask5StartResult result;
            g_task5_reported_state = BALL_TASK5_IDLE;
            BallAutotune_StartTask5Capture(g_sys_tick);
            result = start_ball_task5();
            if (result == BALL_TASK5_START_NO_VISION) {
                bt_send(">ERR T5 NO VISION\r\n");
            } else if (result == BALL_TASK5_START_NOT_CENTERED) {
                bt_send(">ERR T5 PLACE BALL AT O\r\n");
            }
        } else if (c[1] == 'x' && c[2] == '\0') {
            stop_ball_task5();
        } else if (handle_lap_pid_command(5U, c + 1) == 0U) {
            bt_send(">ERR 5/5x/5s/5r/5p/5v/5i/5d/5a/5c/5w/5j/5k\r\n");
        }
        break;
    }
    case '6': {
        if (c[1] == '\0') {
            BallTask6StartResult result;
            g_task6_reported_state = BALL_TASK6_IDLE;
            BallAutotune_StartTask6Capture(g_sys_tick);
            result = start_ball_task6();
            if (result == BALL_TASK6_START_OK) {
                task6_reset_sequence_state();
            } else if (result == BALL_TASK6_START_NO_VISION) {
                bt_send(">ERR T6 NO VISION\r\n");
            } else if (result == BALL_TASK6_START_POSITION_UNSAFE) {
                bt_send(">ERR T6 POSITION UNSAFE\r\n");
            }
        } else if (c[1] == 'x' && c[2] == '\0') {
            stop_ball_task6();
        } else if (handle_lap_pid_command(6U, c + 1) == 0U) {
            bt_send(">ERR 6/6x/6s/6r/6p/6v/6i/6d/6a/6c/6w/6j/6k\r\n");
        } else {
            /* PID命令已经发送结果 */
        }
        break;
    }
    case 'h': bt_send(">2/2x 3 4 5 6; T3=PB21 T4=PA26 T5=PB9 T6=PB14x3; 5s/6s PID; B...\r\n"); break;
    case 'z':
        set_manual_drive(0, 0);
        Motor_Stop();
        IMU_Calibrate();
        bt_send(">YAW ZERO\r\n");
        break;
    case 'y': {
        float target;
        if (!parse_float_arg(c + 1, &target) || target < -180.0f || target > 180.0f ||
            !IMU_IsFresh(g_sys_tick, 100U)) {
            bt_send(">ERR Y ARG/IMU\r\n");
            break;
        }
        abort_lap_tasks(g_sys_tick);
        BallTask4_Abort(g_sys_tick);
        disable_speed_loop(1U);
        Track_Stop();
        Control_SetYawTarget(target);
        Control_CapturePoseTarget();
        g_ld = 0;
        g_rd = 0;
        g_drive_mode = DRIVE_POSE_HOLD;
        bt_send(">YAW HOLD ON\r\n");
        break;
    }
    case 'g': {
        float target;
        char reply[32];
        if (!parse_float_arg(c + 1, &target) || target < -180.0f || target > 180.0f) {
            bt_send(">ERR G ARG\r\n");
            break;
        }
        if (!IMU_IsFresh(g_sys_tick, 100U)) {
            bt_send(">ERR IMU STALE\r\n");
            break;
        }
        start_yaw_turn(target);
        sprintf(reply, ">TURN ABS:%.1f\r\n", Control_GetYawTarget());
        bt_send(reply);
        break;
    }
    default: bt_send(">ERR:?\r\n"); break;
    }
}

int main(void)
{
    SYSCFG_DL_init(); LED_Init();
    SysTick_Config(32000);  /* 1毫秒心跳，32兆赫兹 */

    Sensor_Init(); Encoder_LeftInit(); Encoder_RightInit(); Encoder_HWInit();
    IMU_Init();

    DL_GPIO_clearPins(DIR_R_PORT, DIR_R_AIN1_PIN | DIR_R_AIN2_PIN);
    DL_GPIO_clearPins(DIR_L_PORT, DIR_L_BIN1_PIN | DIR_L_BIN2_PIN);
    DL_GPIO_enableOutput(DIR_R_PORT, DIR_R_AIN1_PIN | DIR_R_AIN2_PIN);
    DL_GPIO_enableOutput(DIR_L_PORT, DIR_L_BIN1_PIN | DIR_L_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, PWM_MAX, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, PWM_MAX, DL_TIMER_CC_1_INDEX);
    Motor_Init();
    WheelSpeed_Init(g_sys_tick);
    BallControl_Init(g_sys_tick);
    BallControl_SetEnabled(0U);
    BallTask3_Init();
    BallTask4_Init(g_sys_tick);
    BallTask5_Init(g_sys_tick);
    BallTask6_Init(g_sys_tick);
    BallAutotune_Init(g_sys_tick);

    /* PA7 PA26 PB9 PB14为独立低有效按键 */
    DL_GPIO_initDigitalInputFeatures(TASK2_BUTTON_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TASK4_BUTTON_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TASK5_BUTTON_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TASK6_BUTTON_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    g_start_button_raw =
        (DL_GPIO_readPins(GPIO_CTRL_PORT, GPIO_CTRL_BTN_START_PIN) != 0U) ? 1U : 0U;
    g_start_button_stable = g_start_button_raw;
    g_start_button_change_tick = g_sys_tick;
    g_task4_button_raw =
        (DL_GPIO_readPins(TASK4_BUTTON_PORT, TASK4_BUTTON_PIN) != 0U) ? 1U : 0U;
    g_task4_button_stable = g_task4_button_raw;
    g_task4_button_change_tick = g_sys_tick;
    g_task5_button_raw =
        (DL_GPIO_readPins(TASK5_BUTTON_PORT, TASK5_BUTTON_PIN) != 0U) ? 1U : 0U;
    g_task5_button_stable = g_task5_button_raw;
    g_task5_button_change_tick = g_sys_tick;
    g_task6_button_raw =
        (DL_GPIO_readPins(TASK6_BUTTON_PORT, TASK6_BUTTON_PIN) != 0U) ? 1U : 0U;
    g_task6_button_stable = g_task6_button_raw;
    g_task6_button_change_tick = g_sys_tick;
    g_task6_button_last_press_tick =
        g_sys_tick - TASK6_BUTTON_REARM_MS;
    task6_reset_sequence_state();
    g_task2_button_raw =
        (DL_GPIO_readPins(TASK2_BUTTON_PORT, TASK2_BUTTON_PIN) != 0U) ? 1U : 0U;
    g_task2_button_stable = g_task2_button_raw;
    g_task2_button_change_tick = g_sys_tick;
    g_cmd_accept_tick = g_sys_tick + STARTUP_COMMAND_LOCKOUT_MS;

    DL_UART_enableInterrupt(UART_DEBUG_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);

    if (OLED_Init() != 0) { while (1) { LED_Toggle(); delay_ms(200U); } }
    OLED_Clear();
    oled_line(0, "BT CAR LREV3");
    oled_line(1, "h=help");
    oled_line(6, "T2:PA7 T4:PA26");
    oled_line(7, "T5:PB9 T6:PB14");

    uint8_t hb = 0;
    uint8_t oled_page = 2U;
    uint32_t oled_tick = 0;
    char t[22];

    while (1) {
        if (g_task6_sequence_stage != TASK6_SEQUENCE_CAPTURE_TARGET &&
            BallTask6_HasCapturedTarget() == 0U &&
            BallTask6_IsRunning() == 0U) {
            task6_reset_sequence_state();
        }
        BallTask6_PrepareProcess(g_sys_tick);
        if (g_task6_sequence_stage == TASK6_SEQUENCE_START_RUN) {
            if (BallTask6_IsPositionReady() != 0U) {
                if (g_task6_ready_reported == 0U) {
                    g_task6_ready_reported = 1U;
                    report_task6_position_ready(g_sys_tick);
                }
            } else {
                g_task6_ready_reported = 0U;
            }
            if (g_task6_start_pending != 0U) {
                (void)task6_start_run_when_ready();
            }
        }

        if (start_button_pressed(g_sys_tick) != 0U) {
            if (BallAutotune_IsTrialActive() != 0U) {
                abort_lap_tasks(g_sys_tick);
                BallTask3_Abort(g_sys_tick);
                stop_ball_test_motion();
                BallControl_EStop(BALL_CONTROL_SAFETY_NONE);
                BallAutotune_SetTrialActive(0U);
                bt_send(">BSTOP PB21\r\n");
            } else if (BallTask4_IsRunning() != 0U) {
                stop_ball_task4();
            } else if (BallTask5_IsRunning() != 0U) {
                stop_ball_task5();
            } else if (BallTask6_IsRunning() != 0U) {
                stop_ball_task6();
            } else if (BallTask3_IsRunning() != 0U) {
                BallTask3_Abort(g_sys_tick);
                stop_ball_test_motion();
            } else {
                /* 同一启动故障也重新发送结果 */
                BallTask3StartResult result;
                g_task3_reported_state = BALL_TASK3_IDLE;
                g_task3_reported_vision_waiting = 0U;
                result = start_ball_task3();
                if (result == BALL_TASK3_START_OK) {
                    BallAutotune_StartWheeltecCapture(g_sys_tick);
                }
            }
        }

        if (task2_button_pressed(g_sys_tick) != 0U) {
            if (g_task2_running != 0U) {
                task2_stop();
                bt_send(">T2 STOP PA7\r\n");
            } else {
                task2_start(g_sys_tick);
            }
        }

        if (task5_button_pressed(g_sys_tick) != 0U) {
            if (BallTask5_IsRunning() != 0U) {
                stop_ball_task5();
            } else {
                BallTask3_Abort(g_sys_tick);
                BallTask4_Abort(g_sys_tick);
                BallAutotune_SetTrialActive(0U);
                g_task5_reported_state = BALL_TASK5_IDLE;
                /* 启动采集先于校验，拒绝原因也能显示 */
                BallAutotune_StartTask5Capture(g_sys_tick);
                BallTask5StartResult result = start_ball_task5();
                if (result == BALL_TASK5_START_NO_VISION) {
                    bt_send(">ERR T5 NO VISION\r\n");
                } else if (result == BALL_TASK5_START_NOT_CENTERED) {
                    bt_send(">ERR T5 PLACE BALL AT O\r\n");
                }
            }
        }

        if (task6_button_pressed(g_sys_tick) != 0U) {
            if (BallTask6_IsRunning() != 0U) {
                stop_ball_task6();
                bt_send(">T6 STOP PB14\r\n");
            } else if (g_task6_sequence_stage ==
                       TASK6_SEQUENCE_CAPTURE_TARGET) {
                BallTask6SetupResult setup_result;

                BallTask6_Abort(g_sys_tick);
                task6_hold_chassis_stopped();
                BallAutotune_SetTrialActive(0U);
                setup_result = BallTask6_CaptureTarget(g_sys_tick);
                if (setup_result == BALL_TASK6_SETUP_OK) {
                    g_task6_sequence_stage =
                        TASK6_SEQUENCE_START_POSITIONING;
                    report_task6_setup_stage(
                        "1/3 CAPTURED",
                        BallTask6_GetCapturedTargetMm());
                } else {
                    report_task6_setup_error(setup_result);
                }
            } else if (g_task6_sequence_stage ==
                       TASK6_SEQUENCE_START_POSITIONING) {
                BallTask6SetupResult setup_result;

                task6_hold_chassis_stopped();
                (void)TaskProfiles_LoadTask6();
                setup_result = BallTask6_BeginPositioning(g_sys_tick);
                if (setup_result == BALL_TASK6_SETUP_OK) {
                    g_task6_sequence_stage = TASK6_SEQUENCE_START_RUN;
                    g_task6_ready_reported = 0U;
                    report_task6_setup_stage(
                        "2/3 POSITION",
                        BallTask6_GetCapturedTargetMm());
                    BallAutotune_StartTask6PositionCapture(g_sys_tick);
                } else {
                    report_task6_setup_error(setup_result);
                }
            } else {
                if (BallTask6_IsPositionReady() == 0U) {
                    g_task6_start_pending = 1U;
                    bt_send(">T6 START ARMED WAIT READY\r\n");
                } else {
                    (void)task6_start_run_when_ready();
                }
            }
        }

        if (task4_button_pressed(g_sys_tick) != 0U) {
            if (BallTask4_IsRunning() != 0U) {
                stop_ball_task4();
            } else {
                BallTask4StartResult result;
                BallTask3_Abort(g_sys_tick);
                BallAutotune_SetTrialActive(0U);
                g_task4_reported_state = BALL_TASK4_IDLE;
                /* PA26总是打开第四问遥测，便于看拒绝原因 */
                BallAutotune_StartTask4Capture(g_sys_tick);
                result = start_ball_task4();
                if (result == BALL_TASK4_START_NO_VISION) {
                    bt_send(">ERR T4 NO VISION\r\n");
                } else if (result == BALL_TASK4_START_NOT_CENTERED) {
                    bt_send(">ERR T4 BALL NOT AT O\r\n");
                }
            }
        }

        /* 优先用CR LF结束命令，无结束符时40毫秒静默 */
        if (g_cmd_ready || (g_cmd_idx && (uint32_t)(g_sys_tick - g_cmd_last_tick) >= 40U)) {
            char tmp[CMD_BUF_SIZE];
            uint8_t overflow;
            DL_UART_disableInterrupt(UART_DEBUG_INST, DL_UART_INTERRUPT_RX);
            strcpy(tmp, g_cmd);
            overflow = g_cmd_overflow;
            g_cmd_idx = 0;
            g_cmd_ready = 0;
            g_cmd_overflow = 0;
            g_cmd[0] = '\0';
            DL_UART_enableInterrupt(UART_DEBUG_INST, DL_UART_INTERRUPT_RX);
            if (overflow) bt_send(">ERR CMD LONG\r\n");
            else cmd_exec(tmp);
        }

        Sensor_Read();
        task2_process(g_sys_tick);
        BallTask4_Process(g_sys_tick);
        BallTask5_Process(g_sys_tick);
        BallTask6_Process(g_sys_tick);
        BallControl_Process(g_sys_tick);
        BallTask3_Process(g_sys_tick);
        report_ball_task3_state();
        report_ball_task4_state();
        report_ball_task5_state();
        report_ball_task6_state();
        BallAutotune_Process(g_sys_tick, bt_send);

        /* 原地转向和位姿保持依赖IMU，短时失效时降级直行 */
        if (g_drive_mode != DRIVE_MANUAL && g_drive_mode != DRIVE_TRACK &&
            g_drive_mode != DRIVE_TASK2 &&
            g_drive_mode != DRIVE_YAW_HOLD && g_drive_mode != DRIVE_TASK4 &&
            g_drive_mode != DRIVE_TASK5 && g_drive_mode != DRIVE_TASK6 &&
            !IMU_IsFresh(g_sys_tick, 100U)) {
            disable_speed_loop(1U);
            Control_ResetPoseHold();
            g_ld = 0;
            g_rd = 0;
            g_drive_mode = DRIVE_MANUAL;
            bt_send(">FAULT IMU STALE\r\n");
        }

        if (g_drive_mode == DRIVE_TASK4 || g_drive_mode == DRIVE_TASK5 ||
            g_drive_mode == DRIVE_TASK6) {
            /* 这些模式下由任务模块负责轮速环 */
        } else if (g_drive_mode == DRIVE_TASK2) {
            /* 第二问每20毫秒更新循迹目标，共享轮速环负责执行 */
            WheelSpeed_Update(g_sys_tick);
        } else if (g_drive_mode == DRIVE_TRACK) {
            int16_t target_l, target_r;
            if (Track_ComputeWheelTargets(g_sys_tick, &target_l, &target_r)) {
                WheelSpeed_SetTargets(target_l, target_r);
            } else {
                WheelSpeed_SetTargets(0, 0);
            }
            WheelSpeed_Update(g_sys_tick);
        } else if (g_drive_mode == DRIVE_YAW_HOLD) {
            uint32_t now = g_sys_tick;
            if ((uint32_t)(now - g_mode_tick) >= 10U) {
                g_mode_tick = now;
                int32_t base_target = g_ld;
                int32_t turn_target = 0;

                /* IMU新鲜时保持航向，失效时左右同速 */
                if (IMU_IsFresh(now, 100U)) {
                    float turn_pwm =
                        Control_ComputeYawTurn((float)g_ld, Control_GetYawTarget());
                    /* 航向修正最多约20RPM差速 */
                    turn_target = (int32_t)(turn_pwm * 20.0f / 700.0f);
                }

                /* 航向修正不能跨过零点，避免单轮反转 */
                int32_t max_turn = base_target >= 0 ? base_target : -base_target;
                if (max_turn > 0) max_turn--;
                if (turn_target > max_turn) turn_target = max_turn;
                if (turn_target < -max_turn) turn_target = -max_turn;

                int32_t left_target = base_target + turn_target;
                int32_t right_target = base_target - turn_target;
                if (left_target > WHEEL_SPEED_MAX_RPM) left_target = WHEEL_SPEED_MAX_RPM;
                if (left_target < -WHEEL_SPEED_MAX_RPM) left_target = -WHEEL_SPEED_MAX_RPM;
                if (right_target > WHEEL_SPEED_MAX_RPM) right_target = WHEEL_SPEED_MAX_RPM;
                if (right_target < -WHEEL_SPEED_MAX_RPM) right_target = -WHEEL_SPEED_MAX_RPM;
                WheelSpeed_SetTargets((int16_t)left_target, (int16_t)right_target);
            }
            WheelSpeed_Update(now);
        } else if (g_drive_mode == DRIVE_YAW_TURN) {
            static uint32_t yaw_tick;
            uint32_t now = g_sys_tick;
            if ((uint32_t)(now - yaw_tick) >= 10U) {
                yaw_tick = now;
                float error = Control_YawError(Control_GetYawTarget(), IMU_Yaw());
                float gyro = IMU_GyroZ();

                if (g_drive_mode == DRIVE_YAW_TURN &&
                    (uint32_t)(now - g_yaw_start_tick) >= 8000U) {
                    Motor_Stop();
                    Control_ResetYawPid();
                    g_drive_mode = DRIVE_MANUAL;
                    bt_send(">TURN TIMEOUT\r\n");
                } else if (error > -2.5f && error < 2.5f &&
                           gyro > -8.0f && gyro < 8.0f) {
                    /* 进入容差区先刹车，避免冲过目标 */
                    Motor_Stop();
                    if (++g_yaw_settle_count >= 5U) {
                        Control_ResetYawPid();
                        g_pose_arm_count = 0;
                        g_pose_arm_last_l = Encoder_LeftGetCount();
                        g_pose_arm_last_r = Encoder_RightGetCount();
                        g_drive_mode = DRIVE_POSE_ARM;
                    }
                } else {
                    g_yaw_settle_count = 0;
                    YawControl(0.0f, Control_GetYawTarget());
                }
            }
        } else if (g_drive_mode == DRIVE_POSE_ARM) {
            static uint32_t arm_tick;
            uint32_t now = g_sys_tick;
            if ((uint32_t)(now - arm_tick) >= 10U) {
                arm_tick = now;
                int32_t left = Encoder_LeftGetCount();
                int32_t right = Encoder_RightGetCount();
                int32_t dl = left - g_pose_arm_last_l;
                int32_t dr = right - g_pose_arm_last_r;
                g_pose_arm_last_l = left;
                g_pose_arm_last_r = right;
                if (dl < 0) dl = -dl;
                if (dr < 0) dr = -dr;

                float error = Control_YawError(Control_GetYawTarget(), IMU_Yaw());
                float gyro = IMU_GyroZ();
                Motor_Stop();
                if (error > -2.5f && error < 2.5f &&
                    gyro > -8.0f && gyro < 8.0f && dl <= 2 && dr <= 2) {
                    if (++g_pose_arm_count >= 10U) {
                        Control_CapturePoseTarget();
                        g_drive_mode = DRIVE_POSE_HOLD;
                        bt_send(">TURN DONE HOLD\r\n");
                    }
                } else {
                    g_pose_arm_count = 0;
                    if (!(error > -2.5f && error < 2.5f)) {
                        g_drive_mode = DRIVE_YAW_TURN;
                        g_yaw_start_tick = now;
                        g_mode_tick = now;
                        Control_ResetYawPid();
                        g_yaw_settle_count = 0;
                    }
                }
            }
        } else if (g_drive_mode == DRIVE_POSE_HOLD) {
            static uint32_t pose_tick;
            uint32_t now = g_sys_tick;
            if ((uint32_t)(now - pose_tick) >= 10U) {
                pose_tick = now;
                Control_PoseHoldStep();
            }
        } else {
            Motor_Left(g_ld);
            Motor_Right(g_rd);
        }

        /* OLED阻塞刷新，每200毫秒更新一次 */
        if ((uint32_t)(g_sys_tick - oled_tick) < 200U) continue;
        oled_tick = g_sys_tick;

        WheelSpeedStatus speed_status;
        BallTask3Status task3_status;
        BallTask4Status task4_status;
        BallTask5Status task5_status;
        BallTask6Status task6_status;
        BallControlStatus ball_status;
        WheelSpeed_GetStatus(&speed_status);
        BallTask3_GetStatus(&task3_status);
        BallTask4_GetStatus(&task4_status);
        BallTask5_GetStatus(&task5_status);
        BallTask6_GetStatus(&task6_status);
        BallControl_GetStatus(&ball_status, g_sys_tick);

        if (g_task2_running != 0U) {
            hb ^= 1U;
            oled_show_task_elapsed("T2", g_task2_elapsed_ms);
            if (hb) LED_On(); else LED_Off();
            continue;
        }

        if (BallTask4_IsRunning() != 0U) {
            hb ^= 1U;
            oled_show_task_elapsed("T4", task4_status.elapsed_ms);
            if (hb) LED_On(); else LED_Off();
            continue;
        }

        if (BallTask5_IsRunning() != 0U) {
            hb ^= 1U;
            oled_show_task_elapsed("T5", task5_status.elapsed_ms);
            if (hb) LED_On(); else LED_Off();
            continue;
        }

        if (BallTask6_IsRunning() != 0U) {
            hb ^= 1U;
            oled_show_task_elapsed("T6", task6_status.elapsed_ms);
            if (hb) LED_On(); else LED_Off();
            continue;
        }

        if (BallTask3_IsRunning() != 0U) {
            static uint8_t task3_detail_div;
            hb ^= 1U;
            if (task3_status.vision_waiting != 0U) {
                snprintf(t, sizeof(t), "T3:WAIT VISION");
            } else if (task3_status.state == BALL_TASK3_MOVE_PLUS_5) {
                snprintf(t, sizeof(t), "T3:+5 X:%+d V:%+d",
                         task3_status.position_mm, task3_status.velocity_mm_s);
            } else {
                snprintf(t, sizeof(t), "T3:-5 X:%+d V:%+d",
                         task3_status.position_mm, task3_status.velocity_mm_s);
            }
            oled_line(7, t);
            if (task3_status.vision_waiting != 0U) {
                snprintf(t, sizeof(t), "LINK:%u VALID:%u",
                         (unsigned)ball_status.link_active,
                         (unsigned)ball_status.ball_valid);
                oled_line(6, t);
            } else if ((task3_detail_div++ & 1U) == 0U) {
                snprintf(t, sizeof(t), "K:%u U:%u R:%+d",
                         (unsigned)ball_status.breakaway_active,
                         (unsigned)ball_status.pulse_us,
                         ball_status.target_velocity_mm_s);
                oled_line(6, t);
            }
            if (hb) LED_On(); else LED_Off();
            continue;
        }

        if (g_task6_sequence_stage != TASK6_SEQUENCE_CAPTURE_TARGET &&
            BallTask6_HasCapturedTarget() != 0U) {
            int16_t target_mm = BallTask6_GetCapturedTargetMm();

            hb ^= 1U;
            if (g_task6_sequence_stage ==
                TASK6_SEQUENCE_START_POSITIONING) {
                snprintf(t, sizeof(t), "T6 1/3 T:%+dMM", target_mm);
            } else if (BallTask6_IsPositionReady() != 0U) {
                snprintf(t, sizeof(t), "T6 READY T:%+d", target_mm);
            } else {
                snprintf(t, sizeof(t), "T6 2/3 POSITION");
            }
            oled_line(7, t);
            if (hb) LED_On(); else LED_Off();
            continue;
        }

        if (oled_page == 2U) {
            sprintf(t, "R T:%d V:%d", speed_status.target_right,
                    speed_status.measured_right);
            oled_line(2, t);
        } else if (oled_page == 3U) {
            sprintf(t, "L T:%d V:%d", speed_status.target_left,
                    speed_status.measured_left);
            oled_line(3, t);
        } else if (oled_page == 4U) {
            sprintf(t, "Y:%6.1f T:%6.1f", IMU_Yaw(), Control_GetYawTarget());
            oled_line(4, t);
        } else if (oled_page == 5U) {
            sprintf(t, "M:%s G:%5.1f",
                    g_drive_mode == DRIVE_TRACK ? "TRK" :
                    g_drive_mode == DRIVE_TASK2 ? "T2" :
                    g_drive_mode == DRIVE_YAW_HOLD ? "HLD" :
                    g_drive_mode == DRIVE_YAW_TURN ? "YAW" :
                    g_drive_mode == DRIVE_POSE_ARM ? "ARM" :
                    g_drive_mode == DRIVE_POSE_HOLD ? "POS" :
                    g_drive_mode == DRIVE_TASK4 ? "T4" :
                    g_drive_mode == DRIVE_TASK5 ? "T5" :
                    g_drive_mode == DRIVE_TASK6 ? "T6" : "MAN",
                    IMU_GyroZ());
            oled_line(5, t);
        } else if (oled_page == 6U) {
            if (is_track_mode(g_drive_mode)) {
                sprintf(t, "S:%02X A:%lu", (unsigned)Track_GetSensorMask(),
                        (unsigned long)Track_GetFrameAge(g_sys_tick));
            } else {
                sprintf(t, "P:%ld I:%s", (long)Control_GetPoseError(),
                        IMU_IsFresh(g_sys_tick, 100U) ? "OK" : "OLD");
            }
            oled_line(6, t);
        } else {
            hb ^= 1U;
            if (g_task2_done != 0U) {
                unsigned long elapsed_s =
                    (unsigned long)(g_task2_elapsed_ms / 1000U);
                unsigned long elapsed_cs =
                    (unsigned long)((g_task2_elapsed_ms % 1000U) / 10U);
                snprintf(t, sizeof(t), "T2 %s %lu.%02lus D:%d",
                         g_task2_done == 1U ? "OK" : "TO",
                         elapsed_s, elapsed_cs, g_task2_distance_mm);
            } else if (task5_status.state == BALL_TASK5_DONE) {
                snprintf(t, sizeof(t), "T5 OK %lu.%02lus",
                         (unsigned long)(task5_status.elapsed_ms / 1000U),
                         (unsigned long)((task5_status.elapsed_ms % 1000U) / 10U));
            } else if (task5_status.state >= BALL_TASK5_FAULT_NO_VISION) {
                sprintf(t, "T5:FAULT %u D:%d", (unsigned)task5_status.state,
                        task5_status.distance_mm);
            } else if (task6_status.state == BALL_TASK6_DONE) {
                snprintf(t, sizeof(t), "T6 OK %lu.%02lus",
                         (unsigned long)(task6_status.elapsed_ms / 1000U),
                         (unsigned long)((task6_status.elapsed_ms % 1000U) / 10U));
            } else if (task6_status.state >= BALL_TASK6_FAULT_NO_VISION) {
                sprintf(t, "T6:FAULT %u D:%d", (unsigned)task6_status.state,
                        task6_status.distance_mm);
            } else if (task4_status.state == BALL_TASK4_DONE) {
                if (ball_status.link_active == 0U ||
                    ball_status.ball_valid == 0U ||
                    ball_status.confidence == 0U) {
                    sprintf(t, "T4:HOLD WAIT X:%d", ball_status.position_mm);
                } else {
                    snprintf(t, sizeof(t), "T4 OK %lu.%02lus",
                             (unsigned long)(task4_status.elapsed_ms / 1000U),
                             (unsigned long)((task4_status.elapsed_ms % 1000U) / 10U));
                }
            } else if (task4_status.state >= BALL_TASK4_FAULT_NO_VISION) {
                sprintf(t, "T4:FAULT %u D:%d", (unsigned)task4_status.state,
                        task4_status.distance_mm);
            } else if (task3_status.state == BALL_TASK3_MOVE_PLUS_5) {
                snprintf(t, sizeof(t), "T3 +5 %lu.%02lus",
                         (unsigned long)(task3_status.elapsed_ms / 1000U),
                         (unsigned long)((task3_status.elapsed_ms % 1000U) / 10U));
            } else if (task3_status.state == BALL_TASK3_MOVE_MINUS_5) {
                snprintf(t, sizeof(t), "T3 -5 %lu.%02lus",
                         (unsigned long)(task3_status.elapsed_ms / 1000U),
                         (unsigned long)((task3_status.elapsed_ms % 1000U) / 10U));
            } else if (task3_status.state == BALL_TASK3_DONE) {
                snprintf(t, sizeof(t), "T3 OK %lu.%02lus",
                         (unsigned long)(task3_status.elapsed_ms / 1000U),
                         (unsigned long)((task3_status.elapsed_ms % 1000U) / 10U));
            } else if (task3_status.state >= BALL_TASK3_FAULT_NO_VISION) {
                unsigned frame_age = (ball_status.frame_age_ms > 999U) ?
                    999U : (unsigned)ball_status.frame_age_ms;
                snprintf(t, sizeof(t), "F%u L%u V%u C%u A%u",
                         (unsigned)task3_status.state,
                         (unsigned)ball_status.link_active,
                         (unsigned)ball_status.ball_valid,
                         (unsigned)ball_status.confidence,
                         frame_age);
            } else if (ball_status.enabled == 0U) {
                if (ball_status.link_active != 0U && ball_status.ball_valid != 0U) {
                    sprintf(t, "T3:WAIT X:%d", (int)ball_status.position_mm);
                } else {
                    sprintf(t, "T3:WAIT PB21");
                }
            } else if (ball_status.link_active == 0U) {
                sprintf(t, "B:NO U:%u C:%u",
                        (unsigned)(ball_status.uart_errors % 1000U),
                        (unsigned)(ball_status.checksum_errors % 1000U));
            } else if (ball_status.ball_valid == 0U) {
                sprintf(t, "B:LOST P:%u", (unsigned)ball_status.pulse_us);
            } else {
                sprintf(t, "B:OK X:%d P:%u", (int)ball_status.position_mm,
                        (unsigned)ball_status.pulse_us);
            }
            oled_line(7, t);
            if (hb) LED_On(); else LED_Off();
        }
        oled_page++;
        if (oled_page > 7U) oled_page = 2U;
    }
}
