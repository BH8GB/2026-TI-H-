/* 文件 ball_autotune.c，用途 串口调参和遥测 */

#include "ball_autotune.h"
#include "ball_control.h"
#include "ball_task3.h"
#include "ball_task4.h"
#include "ball_task5.h"
#include "ball_task6.h"
#include "task_profiles.h"
#include "wheel_speed.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BALL_AUTOTUNE_STREAM_PERIOD_MS 50U
/* 第四问采集结束后保留短暂输出，方便上位机连接 */
#define BALL_WHEELTEC_TAIL_MS         10000U

typedef enum {
    BALL_WHEELTEC_OFF = 0,
    BALL_WHEELTEC_AUTO,
    BALL_WHEELTEC_CONTINUOUS
} BallWheeltecMode;

typedef enum {
    BALL_CAPTURE_TASK3 = 0,
    BALL_CAPTURE_TASK4,
    BALL_CAPTURE_TASK5,
    BALL_CAPTURE_TASK6_POSITION,
    BALL_CAPTURE_TASK6
} BallCaptureSource;

static uint8_t g_stream_enabled;
static uint8_t g_trial_active;
static BallWheeltecMode g_wheeltec_mode;
static BallCaptureSource g_capture_source;
static uint8_t g_wheeltec_terminal_seen;
static uint32_t g_last_stream_tick;
static uint32_t g_wheeltec_stop_tick;
static uint32_t g_capture_start_tick;
static char g_reply[128];

/* 浮点参数按千分单位回传，减少栈开销 */
static int32_t ball_tuning_milli(float value)
{
    return (int32_t)(value * 1000.0f + 0.5f);
}

/* 严格解析浮点参数，拒绝尾随字符和非法值 */
static uint8_t ball_parse_float(const char *text, float *value)
{
    char *end;
    float parsed;
    if (text == 0 || value == 0 || text[0] == '\0') return 0U;
    parsed = strtof(text, &end);
    if (end == text || *end != '\0' || parsed != parsed ||
        parsed < -10000.0f || parsed > 10000.0f) {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

/* 静摩擦补偿范围0到220微秒 */
static uint8_t ball_parse_compensation(const char *text, uint16_t *value)
{
    char *end;
    long parsed;
    if (text == 0 || value == 0 || text[0] == '\0') return 0U;
    parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < 0 || parsed > 220) return 0U;
    *value = (uint16_t)parsed;
    return 1U;
}

/* 应答回显整组参数，确认没有半组更新 */
static void ball_send_status(BallAutotuneSendFn send, const char *prefix)
{
    BallControlParams params;
    BallControlStatus status;
    BallTask3Status task;
    if (send == 0) return;
    BallControl_GetParams(&params);
    BallControl_GetStatus(&status, 0U);
    BallTask3_GetStatus(&task);
    snprintf(g_reply, sizeof(g_reply),
             ">%s P=%ld V=%ld I=%ld D=%ld A=%ld C=%u S=%u E=%u\r\n",
             prefix,
             (long)ball_tuning_milli(params.position_kp_per_s),
             (long)ball_tuning_milli(params.velocity_kp_us_per_mm_s),
             (long)ball_tuning_milli(params.velocity_ki_us_per_mm),
             (long)ball_tuning_milli(params.velocity_damping_us_per_mm_s),
             (long)ball_tuning_milli(
                 params.acceleration_damping_us_per_mm_s2),
             (unsigned)params.static_comp_us,
             (unsigned)task.state,
             (unsigned)status.safety_reason);
    send(g_reply);
}

/* 参数只能在试验停止时更新 */
static uint8_t ball_tuning_is_idle(BallAutotuneSendFn send)
{
    BallControlStatus status;
    BallControl_GetStatus(&status, 0U);
    if (g_trial_active != 0U || BallTask3_IsRunning() != 0U ||
        status.enabled != 0U) {
        if (send != 0) send(">BERR BUSY USE BSTOP\r\n");
        return 0U;
    }
    return 1U;
}

void BallAutotune_Init(uint32_t now_ms)
{
    g_stream_enabled = 0U;
    g_trial_active = 0U;
    g_wheeltec_mode = BALL_WHEELTEC_OFF;
    g_capture_source = BALL_CAPTURE_TASK3;
    g_wheeltec_terminal_seen = 0U;
    g_last_stream_tick = now_ms;
    g_wheeltec_stop_tick = now_ms;
    g_capture_start_tick = now_ms;
}

BallAutotuneAction BallAutotune_HandleCommand(const char *command,
                                               BallAutotuneSendFn send)
{
    BallControlParams params;
    float value;
    uint16_t compensation;

    if (command == 0 || command[0] != 'B') {
        return BALL_AUTOTUNE_NOT_HANDLED;
    }

    if (strcmp(command, "BSTATUS") == 0) {
        ball_send_status(send, "BSTAT");
        return BALL_AUTOTUNE_HANDLED;
    }
    if (strcmp(command, "BHELP") == 0) {
        if (send != 0) {
            send(">B BP/BV/BI/BD/BA/BC BSTATUS BDEFAULT BTRIAL BSTOP BSTREAM0/1 BWAVE0/1\r\n");
        }
        return BALL_AUTOTUNE_HANDLED;
    }
    if (strcmp(command, "BSTREAM1") == 0) {
        g_stream_enabled = 1U;
        if (send != 0) send(">BSTREAM 1\r\n");
        return BALL_AUTOTUNE_HANDLED;
    }
    if (strcmp(command, "BSTREAM0") == 0) {
        g_stream_enabled = 0U;
        if (send != 0) send(">BSTREAM 0\r\n");
        return BALL_AUTOTUNE_HANDLED;
    }
    if (strcmp(command, "BWAVE1") == 0) {
        g_capture_source = BALL_CAPTURE_TASK3;
        g_wheeltec_mode = BALL_WHEELTEC_CONTINUOUS;
        g_wheeltec_terminal_seen = 0U;
        if (send != 0) send(">BWAVE 1\r\n");
        return BALL_AUTOTUNE_HANDLED;
    }
    if (strcmp(command, "BWAVE0") == 0) {
        BallAutotune_StopWheeltecCapture();
        if (send != 0) send(">BWAVE 0\r\n");
        return BALL_AUTOTUNE_HANDLED;
    }
    if (strcmp(command, "BSTOP") == 0) {
        return BALL_AUTOTUNE_STOP_TRIAL;
    }
    if (strcmp(command, "BTRIAL") == 0) {
        if (g_trial_active != 0U || BallTask3_IsRunning() != 0U) {
            if (send != 0) send(">BERR TRIAL BUSY\r\n");
            return BALL_AUTOTUNE_HANDLED;
        }
        g_stream_enabled = 1U;
        return BALL_AUTOTUNE_START_TRIAL;
    }
    if (strcmp(command, "BDEFAULT") == 0) {
        if (ball_tuning_is_idle(send) == 0U) return BALL_AUTOTUNE_HANDLED;
        (void)TaskProfiles_LoadTask3();
        ball_send_status(send, "BOK DEFAULT");
        return BALL_AUTOTUNE_HANDLED;
    }

    if (ball_tuning_is_idle(send) == 0U) return BALL_AUTOTUNE_HANDLED;
    BallControl_GetParams(&params);

    if (command[1] == 'P' && ball_parse_float(command + 2, &value) != 0U) {
        params.position_kp_per_s = value;
    } else if (command[1] == 'V' &&
               ball_parse_float(command + 2, &value) != 0U) {
        params.velocity_kp_us_per_mm_s = value;
    } else if (command[1] == 'I' &&
               ball_parse_float(command + 2, &value) != 0U) {
        params.velocity_ki_us_per_mm = value;
    } else if (command[1] == 'D' &&
               ball_parse_float(command + 2, &value) != 0U) {
        params.velocity_damping_us_per_mm_s = value;
    } else if (command[1] == 'A' &&
               ball_parse_float(command + 2, &value) != 0U) {
        params.acceleration_damping_us_per_mm_s2 = value;
    } else if (command[1] == 'C' &&
               ball_parse_compensation(command + 2, &compensation) != 0U) {
        params.static_comp_us = compensation;
    } else {
        if (send != 0) send(">BERR COMMAND OR VALUE\r\n");
        return BALL_AUTOTUNE_HANDLED;
    }

    if (BallControl_SetParams(&params) == 0U) {
        if (send != 0) send(">BERR PARAM RANGE\r\n");
        return BALL_AUTOTUNE_HANDLED;
    }
    ball_send_status(send, "BOK");
    return BALL_AUTOTUNE_HANDLED;
}

void BallAutotune_SetTrialActive(uint8_t active)
{
    g_trial_active = (active != 0U) ? 1U : 0U;
    if (g_trial_active != 0U) g_capture_source = BALL_CAPTURE_TASK3;
}

uint8_t BallAutotune_IsTrialActive(void)
{
    return g_trial_active;
}

void BallAutotune_StartWheeltecCapture(uint32_t now_ms)
{
    g_capture_source = BALL_CAPTURE_TASK3;
    g_wheeltec_mode = BALL_WHEELTEC_AUTO;
    g_wheeltec_terminal_seen = 0U;
    g_wheeltec_stop_tick = now_ms;
    g_last_stream_tick = now_ms - BALL_AUTOTUNE_STREAM_PERIOD_MS;
}

void BallAutotune_StartTask4Capture(uint32_t now_ms)
{
    g_capture_source = BALL_CAPTURE_TASK4;
    g_wheeltec_mode = BALL_WHEELTEC_AUTO;
    g_wheeltec_terminal_seen = 0U;
    g_wheeltec_stop_tick = now_ms;
    g_last_stream_tick = now_ms - BALL_AUTOTUNE_STREAM_PERIOD_MS;
}

void BallAutotune_StartTask5Capture(uint32_t now_ms)
{
    g_capture_source = BALL_CAPTURE_TASK5;
    g_wheeltec_mode = BALL_WHEELTEC_AUTO;
    g_wheeltec_terminal_seen = 0U;
    g_wheeltec_stop_tick = now_ms;
    g_last_stream_tick = now_ms - BALL_AUTOTUNE_STREAM_PERIOD_MS;
}

void BallAutotune_StartTask6PositionCapture(uint32_t now_ms)
{
    g_capture_source = BALL_CAPTURE_TASK6_POSITION;
    g_capture_start_tick = now_ms;
    g_wheeltec_mode = BALL_WHEELTEC_AUTO;
    g_wheeltec_terminal_seen = 0U;
    g_wheeltec_stop_tick = now_ms;
    g_last_stream_tick = now_ms - BALL_AUTOTUNE_STREAM_PERIOD_MS;
}

void BallAutotune_StartTask6Capture(uint32_t now_ms)
{
    g_capture_source = BALL_CAPTURE_TASK6;
    g_wheeltec_mode = BALL_WHEELTEC_AUTO;
    g_wheeltec_terminal_seen = 0U;
    g_wheeltec_stop_tick = now_ms;
    g_last_stream_tick = now_ms - BALL_AUTOTUNE_STREAM_PERIOD_MS;
}

uint8_t BallAutotune_IsWheeltecCaptureActive(void)
{
    return (g_wheeltec_mode != BALL_WHEELTEC_OFF) ? 1U : 0U;
}

void BallAutotune_StopWheeltecCapture(void)
{
    g_wheeltec_mode = BALL_WHEELTEC_OFF;
    g_wheeltec_terminal_seen = 0U;
}

void BallAutotune_Process(uint32_t now_ms, BallAutotuneSendFn send)
{
    BallControlStatus ball;
    BallTask3Status task;
    BallTask4Status task4;
    BallTask5Status task5;
    BallTask6Status task6;
    WheelSpeedStatus wheels;
    uint32_t frame_age_ms;
    uint16_t diagnostic;
    uint8_t task_terminal;
    if ((g_stream_enabled == 0U && g_wheeltec_mode == BALL_WHEELTEC_OFF) ||
        send == 0 ||
        (uint32_t)(now_ms - g_last_stream_tick) <
            BALL_AUTOTUNE_STREAM_PERIOD_MS) {
        return;
    }
    g_last_stream_tick = now_ms;
    BallControl_GetStatus(&ball, now_ms);
    BallTask3_GetStatus(&task);
    BallTask4_GetStatus(&task4);
    BallTask5_GetStatus(&task5);
    BallTask6_GetStatus(&task6);
    WheelSpeed_GetStatus(&wheels);
    frame_age_ms = ball.frame_age_ms;
    if (frame_age_ms > 9999U) frame_age_ms = 9999U;
    diagnostic = (uint16_t)ball.confidence |
                 ((uint16_t)ball.ball_valid << 8) |
                 ((uint16_t)ball.breakaway_active << 9) |
                 ((uint16_t)ball.link_active << 10) |
                 ((uint16_t)ball.safety_reason << 11);
    /* B通道CSV字段顺序 */
    /* 波形采集期间不插入CSV，避免破坏字节流 */
    if (g_stream_enabled != 0U && g_wheeltec_mode == BALL_WHEELTEC_OFF) {
        snprintf(g_reply, sizeof(g_reply),
                 "#B,%lu,%u,%d,%d,%d,%d,%u,%d,%u,%u,%u,%u\r\n",
                 (unsigned long)task.elapsed_ms,
                 (unsigned)task.state,
                 (int)ball.target_mm,
                 (int)ball.position_mm,
                 (int)ball.velocity_mm_s,
                 (int)ball.target_velocity_mm_s,
                 (unsigned)ball.pulse_us,
                 (int)ball.control_offset_us,
                 (unsigned)ball.enabled,
                 (unsigned)ball.ball_valid,
                 (unsigned)ball.breakaway_active,
                 (unsigned)ball.safety_reason);
        send(g_reply);
    }

    if (g_wheeltec_mode != BALL_WHEELTEC_OFF) {
        if (g_capture_source == BALL_CAPTURE_TASK4) {
            /* 第四问通道对应评分和调参量 */
            /* 保留B开头波形帧，兼容上位机 */
            snprintf(g_reply, sizeof(g_reply),
                     "{B%lu:%u:%d:%d:%d:%d:%d:%d:%d:%d}$",
                     (unsigned long)task4.elapsed_ms,
                     (unsigned)task4.state,
                     (int)task4.distance_mm,
                     (int)task4.target_rpm,
                     (int)task4.measured_rpm,
                     (int)ball.motion_feedforward_us,
                     (int)task4.acceleration_mm_s2,
                     (int)ball.position_mm,
                     (int)ball.velocity_mm_s,
                     (int)ball.control_offset_us);
        } else if (g_capture_source == BALL_CAPTURE_TASK5) {
            /* 第五问十通道含义 */
            snprintf(g_reply, sizeof(g_reply),
                     "{B%lu:%u:%d:%d:%d:%d:%d:%d:%d:%d}$",
                     (unsigned long)task5.elapsed_ms,
                     (unsigned)task5.state,
                     (int)task5.distance_mm,
                     (int)task5.target_rpm,
                     (int)task5.measured_rpm,
                     (int)task5.line_error,
                     (int)((int32_t)wheels.measured_left -
                           (int32_t)wheels.measured_right),
                     (int)ball.position_mm,
                     (int)ball.velocity_mm_s,
                     (int)ball.control_offset_us);
        } else if (g_capture_source == BALL_CAPTURE_TASK6_POSITION) {
            uint8_t positioning_state;
            int16_t target_mm = BallTask6_GetCapturedTargetMm();
            int16_t error_mm = (int16_t)(target_mm - ball.position_mm);

            /* 第六问预定位通道含义 */
            if (ball.link_active == 0U || ball.ball_valid == 0U ||
                ball.confidence == 0U) {
                positioning_state = 3U;
            } else if (BallTask6_IsPositionReady() != 0U) {
                positioning_state = 2U;
            } else {
                positioning_state = 1U;
            }
            snprintf(g_reply, sizeof(g_reply),
                     "{B%lu:%u:%d:%d:%d:%d:%d:%u:%d:%d}$",
                     (unsigned long)(now_ms - g_capture_start_tick),
                     (unsigned)positioning_state,
                     (int)target_mm,
                     (int)ball.position_mm,
                     (int)error_mm,
                     (int)ball.velocity_mm_s,
                     (int)ball.target_velocity_mm_s,
                     (unsigned)ball.pulse_us,
                     (int)ball.control_offset_us,
                     (int)ball.task6_position_trim_us);
        } else if (g_capture_source == BALL_CAPTURE_TASK6) {
            /* 第六问通道顺序相同，目标为启动时锁存值 */
            snprintf(g_reply, sizeof(g_reply),
                     "{B%lu:%u:%d:%d:%d:%d:%d:%d:%d:%d}$",
                     (unsigned long)task6.elapsed_ms,
                     (unsigned)task6.state,
                     (int)task6.distance_mm,
                     (int)task6.target_rpm,
                     (int)task6.measured_rpm,
                     (int)task6.line_error,
                     (int)task6.target_position_mm,
                     (int)ball.position_mm,
                     (int)ball.velocity_mm_s,
                     (int)ball.control_offset_us);
        } else {
            /* 通道8和9显示视觉帧龄和诊断信息 */
            snprintf(g_reply, sizeof(g_reply),
                     "{B%lu:%u:%d:%d:%d:%d:%u:%d:%lu:%u}$",
                     (unsigned long)task.elapsed_ms,
                     (unsigned)task.state,
                     (int)ball.target_mm,
                     (int)ball.position_mm,
                     (int)ball.velocity_mm_s,
                     (int)ball.target_velocity_mm_s,
                     (unsigned)ball.pulse_us,
                     (int)ball.control_offset_us,
                     (unsigned long)frame_age_ms,
                     (unsigned)diagnostic);
        }
        send(g_reply);
    }

    if (g_capture_source == BALL_CAPTURE_TASK4) {
        task_terminal = (task4.state == BALL_TASK4_IDLE ||
                         task4.state == BALL_TASK4_DONE ||
                         task4.state == BALL_TASK4_ABORTED ||
                         task4.state >= BALL_TASK4_FAULT_NO_VISION) ? 1U : 0U;
    } else if (g_capture_source == BALL_CAPTURE_TASK5) {
        task_terminal = (task5.state == BALL_TASK5_IDLE ||
                         task5.state == BALL_TASK5_DONE ||
                         task5.state == BALL_TASK5_ABORTED ||
                         task5.state >= BALL_TASK5_FAULT_NO_VISION) ? 1U : 0U;
    } else if (g_capture_source == BALL_CAPTURE_TASK6_POSITION) {
        task_terminal = (BallTask6_IsPositioning() == 0U) ? 1U : 0U;
    } else if (g_capture_source == BALL_CAPTURE_TASK6) {
        task_terminal = (task6.state == BALL_TASK6_IDLE ||
                         task6.state == BALL_TASK6_DONE ||
                         task6.state == BALL_TASK6_ABORTED ||
                         task6.state >= BALL_TASK6_FAULT_NO_VISION) ? 1U : 0U;
    } else {
        task_terminal = (task.state == BALL_TASK3_DONE ||
                         task.state == BALL_TASK3_ABORTED ||
                         task.state >= BALL_TASK3_FAULT_NO_VISION) ? 1U : 0U;
    }
    if (task_terminal != 0U) {
        g_trial_active = 0U;
        if (g_wheeltec_mode == BALL_WHEELTEC_AUTO) {
            if (g_wheeltec_terminal_seen == 0U) {
                g_wheeltec_terminal_seen = 1U;
                g_wheeltec_stop_tick = now_ms + BALL_WHEELTEC_TAIL_MS;
            } else if ((int32_t)(now_ms - g_wheeltec_stop_tick) >= 0) {
                BallAutotune_StopWheeltecCapture();
            }
        }
    }
}
