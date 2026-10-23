/* 文件 ball_control.c，用途 钢球视觉接收和摆杆闭环 */

#include "ball_control.h"
#include "ti_msp_dl_config.h"
#include <math.h>

/* 串口帧固定10字节，帧头AA 55 */
#define BALL_FRAME_SIZE                 10U
#define BALL_FLAG_VALID                 0x01U
#define BALL_LINK_TIMEOUT_MS            200U
#define BALL_VALID_HOLD_MS              180U

/* 舵机周期20毫秒，对应50赫兹 */
#define BALL_SERVO_PERIOD_COUNTS        20000U
#define BALL_SERVO_MIN_US               1100U
#define BALL_SERVO_MAX_US               1900U
#define BALL_SERVO_DEFAULT_NEUTRAL_US   1500U
#define BALL_SERVO_MAX_OFFSET_US        360.0f
#define BALL_SERVO_DRIVE_SLEW_US        160U
#define BALL_SERVO_BRAKE_SLEW_US        720U
#define BALL_SERVO_RETURN_SLEW_US        80U

/* 位置外环给期望速度，速度内环给摆杆倾角 */
#define BALL_CONTROL_PERIOD_MS          20U
#define BALL_CONTROL_DT_S               0.020f
#define BALL_POSITION_KP_DEFAULT_PER_S  2.40f
#define BALL_TARGET_VELOCITY_LIMIT_MM_S  60.0f
#define BALL_VELOCITY_KP_DEFAULT        0.95f
#define BALL_VELOCITY_KI_DEFAULT        0.06f
#define BALL_VELOCITY_DAMP_DEFAULT      0.75f
#define BALL_ACCEL_DAMP_DEFAULT          0.00f
#define BALL_VELOCITY_INTEGRAL_LIMIT_MM 180.0f
#define BALL_CONTROL_VELOCITY_LIMIT_MM_S 500.0f
#define BALL_CONTROL_ACCEL_LIMIT_MM_S2  1000.0f
#define BALL_PLAN_ACCEL_MM_S2           160.0f
#define BALL_PLAN_BRAKE_MM_S2           300.0f
#define BALL_PLAN_DELAY_S                 0.10f
#define BALL_OVERSPEED_COMP_US_PER_MM_S   1.50f

/* 视觉跳变保护，按650毫米每秒设门限 */
#define BALL_POSITION_JUMP_BASE_MM       12.0f
#define BALL_POSITION_JUMP_SPEED_MM_S   650.0f
#define BALL_POSITION_REACQUIRE_MS       250U
#define BALL_LOCAL_VELOCITY_ALPHA         0.35f
#define BALL_LOCAL_ACCEL_ALPHA            0.20f

/* 位置和速度都进入死区时回水平 */
#define BALL_POSITION_DEADBAND_MM       4.0f
#define BALL_SETTLE_VELOCITY_MM_S       25.0f

/* 先克服静摩擦，检测到位移后撤力 */
#define BALL_STATIC_ENABLE_ERROR_MM     12.0f
/* 前进落位阶段仍保留静摩擦补偿 */
#define BALL_STATIC_MAX_VELOCITY_MM_S   45.0f
/* 精细保持回程只在接近停止时补偿 */
#define BALL_RETURN_STATIC_MAX_VELOCITY_MM_S 8.0f
#define BALL_STATIC_DEFAULT_COMP_US    140.0f
#define BALL_STATIC_MAX_COMP_US         220U
#define BALL_FINE_HOLD_ZONE_MM           40.0f
#define BALL_FINE_HOLD_STATIC_SCALE       0.50f
#define BALL_FINE_HOLD_SPEED_LIMIT_MM_S  45.0f
#define BALL_FINE_HOLD_SETTLE_VELOCITY_MM_S 2.0f
/* 第六问学习水平偏置，只在大偏差且停滞时积分 */
#define BALL_TASK6_POSITIVE_TRIM_INITIAL_US      (-140.0f)
#define BALL_TASK6_TRIM_START_ERROR_MM            5.0f
#define BALL_TASK6_TRIM_STALL_SPEED_MM_S          8.0f
#define BALL_TASK6_TRIM_KI_US_PER_MM_S            0.80f
#define BALL_TASK6_TRIM_LIMIT_US                 200.0f
/* 停住200毫秒后短促助推，检测到位移即撤回 */
#define BALL_TASK6_NUDGE_START_ERROR_MM            5.0f
#define BALL_TASK6_NUDGE_STALL_SPEED_MM_S          5.0f
#define BALL_TASK6_NUDGE_STOP_SPEED_MM_S          20.0f
#define BALL_TASK6_NUDGE_MIN_US                  240.0f
#define BALL_TASK6_NUDGE_MOVE_MM                    2
#define BALL_TASK6_NUDGE_ARM_TICKS                 10U
#define BALL_TASK6_NUDGE_MAX_TICKS                  4U
#define BALL_TASK6_NUDGE_COOLDOWN_TICKS            20U
#define BALL_BREAKAWAY_MIN_US           330.0f
#define BALL_BREAKAWAY_ERROR_MM         20.0f
#define BALL_BREAKAWAY_MOVE_MM           2
#define BALL_BREAKAWAY_STOP_MM_S         70.0f
#define BALL_BREAKAWAY_MAX_TICKS         8U

/* 正5厘米附近再次卡滞时使用更短助推 */
#define BALL_STALL_NUDGE_MIN_US          250.0f
#define BALL_STALL_NUDGE_MOVE_MM           2
#define BALL_STALL_NUDGE_STOP_MM_S        30.0f
#define BALL_STALL_NUDGE_TARGET_ZONE_MM   10.0f
#define BALL_STALL_NUDGE_MAX_TICKS         5U

/* 行程约正负120毫米，目标限制在安全区内 */
#define BALL_TARGET_LIMIT_MM             115
#define BALL_EDGE_STOP_MM                119
#define BALL_EDGE_REARM_MM               115

#define BALL_POSITION_KP_MIN             0.20f
#define BALL_POSITION_KP_MAX             6.00f
#define BALL_VELOCITY_KP_MIN             0.05f
#define BALL_VELOCITY_KP_MAX             3.00f
#define BALL_VELOCITY_KI_MIN             0.00f
#define BALL_VELOCITY_KI_MAX             0.60f
#define BALL_VELOCITY_DAMP_MIN           0.00f
#define BALL_VELOCITY_DAMP_MAX           0.80f
#define BALL_ACCEL_DAMP_MIN              0.00f
#define BALL_ACCEL_DAMP_MAX              0.30f

/* 接收错误中断集合，用于清空半帧 */
#define BALL_UART_ERROR_INTERRUPTS                                      \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                            \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                              \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                             \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                            \
     DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR)
#define BALL_UART_ALL_INTERRUPTS                                        \
    (DL_UART_MAIN_INTERRUPT_RX | BALL_UART_ERROR_INTERRUPTS)

/* 正坐标指向右端，脉宽增大使右端抬升 */
#define BALL_CONTROL_DIRECTION          (-1.0f)

/* 中断只收帧，浮点控制放在主循环 */
static volatile uint8_t g_rx_buffer[BALL_FRAME_SIZE];
static volatile uint8_t g_rx_index;
static volatile uint32_t g_rx_generation;
static volatile uint32_t g_rx_checksum_errors;
static volatile uint32_t g_rx_uart_errors;
static volatile uint8_t g_rx_flags;
static volatile uint8_t g_rx_sequence;
static volatile uint8_t g_rx_confidence;
static volatile int16_t g_rx_position_mm;
static volatile int16_t g_rx_velocity_mm_s;

/* 主循环状态，读取时短暂关闭视觉中断 */
static BallControlStatus g_status;
static uint32_t g_seen_generation;
static uint32_t g_last_frame_tick;
static uint32_t g_last_valid_tick;
static uint32_t g_last_control_tick;
static uint8_t g_have_valid_sample;
static uint8_t g_have_sequence;
static uint8_t g_last_sequence;
static uint16_t g_neutral_us = BALL_SERVO_DEFAULT_NEUTRAL_US;
static BallControlParams g_params = {
    BALL_POSITION_KP_DEFAULT_PER_S,
    BALL_VELOCITY_KP_DEFAULT,
    BALL_VELOCITY_KI_DEFAULT,
    BALL_VELOCITY_DAMP_DEFAULT,
    BALL_ACCEL_DAMP_DEFAULT,
    (uint16_t)BALL_STATIC_DEFAULT_COMP_US
};
static float g_velocity_integral_mm;
static float g_planned_velocity_mm_s;
static float g_filtered_velocity_mm_s;
static float g_filtered_acceleration_mm_s2;
static uint8_t g_breakaway_ticks;
static uint8_t g_breakaway_active;
static uint8_t g_stall_nudge_ticks;
static uint8_t g_stall_nudge_active;
static uint8_t g_fine_hold_enabled;
static uint8_t g_task6_positioning_assist_enabled;
static float g_task6_position_trim_us;
static uint8_t g_task6_nudge_stall_ticks;
static uint8_t g_task6_nudge_active_ticks;
static uint8_t g_task6_nudge_cooldown_ticks;
static int8_t g_task6_nudge_direction;
static int16_t g_task6_nudge_start_position_mm;
static int16_t g_breakaway_start_position_mm;
static int16_t g_stall_nudge_start_position_mm;
static int16_t g_motion_feedforward_us;

static void ball_arm_breakaway(void);

/* 限制舵机脉宽，防止撞限位 */
static uint16_t ball_clamp_pulse(uint16_t pulse_us)
{
    if (pulse_us < BALL_SERVO_MIN_US) return BALL_SERVO_MIN_US;
    if (pulse_us > BALL_SERVO_MAX_US) return BALL_SERVO_MAX_US;
    return pulse_us;
}

/* 浮点绝对值 */
static float ball_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 视觉速度和目标速度统一限幅 */
static float ball_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/* 清除第六问短助推状态 */
static void ball_task6_reset_nudge(void)
{
    g_task6_nudge_stall_ticks = 0U;
    g_task6_nudge_active_ticks = 0U;
    g_task6_nudge_cooldown_ticks = 0U;
    g_task6_nudge_direction = 0;
    g_task6_nudge_start_position_mm = g_status.position_mm;
}

/* 钢球连续停住才允许短助推 */
static void ball_task6_update_nudge(float error_mm, float velocity_mm_s)
{
    int8_t requested_direction;
    int16_t moved_mm;

    if (g_task6_positioning_assist_enabled == 0U ||
        ball_absf(error_mm) <= BALL_TASK6_NUDGE_START_ERROR_MM) {
        ball_task6_reset_nudge();
        return;
    }

    requested_direction = ((BALL_CONTROL_DIRECTION * error_mm) >= 0.0f) ?
        1 : -1;
    if (g_task6_nudge_active_ticks != 0U) {
        moved_mm = (int16_t)(g_status.position_mm -
                             g_task6_nudge_start_position_mm);
        if (moved_mm < 0) moved_mm = (int16_t)-moved_mm;
        if (requested_direction != g_task6_nudge_direction ||
            moved_mm >= BALL_TASK6_NUDGE_MOVE_MM ||
            ball_absf(velocity_mm_s) >= BALL_TASK6_NUDGE_STOP_SPEED_MM_S ||
            g_task6_nudge_active_ticks > BALL_TASK6_NUDGE_MAX_TICKS) {
            g_task6_nudge_active_ticks = 0U;
            g_task6_nudge_direction = 0;
            g_task6_nudge_cooldown_ticks =
                BALL_TASK6_NUDGE_COOLDOWN_TICKS;
        }
        return;
    }

    if (g_task6_nudge_cooldown_ticks != 0U) {
        g_task6_nudge_cooldown_ticks--;
        g_task6_nudge_stall_ticks = 0U;
        return;
    }

    if (ball_absf(velocity_mm_s) <= BALL_TASK6_NUDGE_STALL_SPEED_MM_S) {
        if (g_task6_nudge_stall_ticks < BALL_TASK6_NUDGE_ARM_TICKS) {
            g_task6_nudge_stall_ticks++;
        }
        if (g_task6_nudge_stall_ticks >= BALL_TASK6_NUDGE_ARM_TICKS) {
            g_task6_nudge_stall_ticks = 0U;
            g_task6_nudge_active_ticks = 1U;
            g_task6_nudge_direction = requested_direction;
            g_task6_nudge_start_position_mm = g_status.position_mm;
        }
    } else {
        g_task6_nudge_stall_ticks = 0U;
    }
}

/* 短助推只补最小力度，不削弱闭环制动 */
static float ball_task6_apply_nudge(float control_us)
{
    if (g_task6_nudge_active_ticks == 0U) return control_us;

    if (g_task6_nudge_direction > 0) {
        if (control_us < BALL_TASK6_NUDGE_MIN_US) {
            control_us = BALL_TASK6_NUDGE_MIN_US;
        }
    } else if (control_us > -BALL_TASK6_NUDGE_MIN_US) {
        control_us = -BALL_TASK6_NUDGE_MIN_US;
    }
    g_task6_nudge_active_ticks++;
    return control_us;
}

/* 按加速和制动斜率逼近速度包络 */
static float ball_move_toward(float current, float target,
                              float accel_step, float brake_step)
{
    float step = accel_step;
    if ((current > 0.0f && target <= 0.0f) ||
        (current < 0.0f && target >= 0.0f) ||
        ball_absf(target) < ball_absf(current)) {
        step = brake_step;
    }
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

/* 按剩余距离生成速度包络 */
static float ball_plan_velocity(float error_mm, float *safe_speed_mm_s)
{
    float distance_mm = ball_absf(error_mm) - BALL_POSITION_DEADBAND_MM;
    float speed_limit = BALL_TARGET_VELOCITY_LIMIT_MM_S;
    float delay_speed;
    float braking_speed;
    float position_speed;
    float safe_speed;
    float requested_velocity;

    if (distance_mm < 0.0f) distance_mm = 0.0f;
    delay_speed = BALL_PLAN_BRAKE_MM_S2 * BALL_PLAN_DELAY_S;
    braking_speed = sqrtf(delay_speed * delay_speed +
                           2.0f * BALL_PLAN_BRAKE_MM_S2 * distance_mm) -
                    delay_speed;
    position_speed = g_params.position_kp_per_s * distance_mm;
    if (g_fine_hold_enabled != 0U) {
        speed_limit = BALL_FINE_HOLD_SPEED_LIMIT_MM_S;
    }
    safe_speed = ball_clampf(braking_speed, 0.0f, speed_limit);
    if (position_speed < safe_speed) safe_speed = position_speed;
    if (safe_speed < 0.0f) safe_speed = 0.0f;

    requested_velocity = (error_mm >= 0.0f) ? safe_speed : -safe_speed;
    g_planned_velocity_mm_s = ball_move_toward(
        g_planned_velocity_mm_s, requested_velocity,
        BALL_PLAN_ACCEL_MM_S2 * BALL_CONTROL_DT_S,
        BALL_PLAN_BRAKE_MM_S2 * BALL_CONTROL_DT_S);
    *safe_speed_mm_s = safe_speed;
    return g_planned_velocity_mm_s;
}

/* 正常控制限制在中位附近，终点限位只作保护 */
static float ball_clamp_control_us(float control_us)
{
    if (control_us > BALL_SERVO_MAX_OFFSET_US) {
        return BALL_SERVO_MAX_OFFSET_US;
    }
    if (control_us < -BALL_SERVO_MAX_OFFSET_US) {
        return -BALL_SERVO_MAX_OFFSET_US;
    }
    return control_us;
}

/* 更新PWM比较值，脉宽等于周期减比较值 */
static void ball_servo_write_us(uint16_t pulse_us)
{
    pulse_us = ball_clamp_pulse(pulse_us);
    DL_TimerA_setCaptureCompareValue(
        SERVO_PWM_INST,
        BALL_SERVO_PERIOD_COUNTS - pulse_us,
        GPIO_SERVO_PWM_C0_IDX);
    g_status.pulse_us = pulse_us;
}

/* 舵机指令限速，减小齿隙冲击 */
static uint16_t ball_slew_pulse(uint16_t target_us, uint16_t current_us,
                                uint16_t max_step_us)
{
    if (target_us > (uint16_t)(current_us + max_step_us)) {
        return (uint16_t)(current_us + max_step_us);
    }
    if (current_us > (uint16_t)(target_us + max_step_us)) {
        return (uint16_t)(current_us - max_step_us);
    }
    return target_us;
}

/* 校验通过后一次发布整帧 */
static void ball_publish_frame(void)
{
    uint8_t checksum = 0U;
    uint8_t index;
    for (index = 0U; index < BALL_FRAME_SIZE - 1U; index++) {
        checksum = (uint8_t)(checksum + g_rx_buffer[index]);
    }
    if (checksum != g_rx_buffer[BALL_FRAME_SIZE - 1U]) {
        g_rx_checksum_errors++;
        return;
    }

    g_rx_flags = g_rx_buffer[2];
    g_rx_sequence = g_rx_buffer[3];
    g_rx_position_mm = (int16_t)((uint16_t)g_rx_buffer[4] |
                                 ((uint16_t)g_rx_buffer[5] << 8));
    g_rx_velocity_mm_s = (int16_t)((uint16_t)g_rx_buffer[6] |
                                   ((uint16_t)g_rx_buffer[7] << 8));
    g_rx_confidence = g_rx_buffer[8];
    g_rx_generation++;
}

/* 字节状态机可在任意位置重新同步帧头 */
static void ball_parse_byte(uint8_t data)
{
    if (g_rx_index == 0U) {
        if (data == 0xAAU) {
            g_rx_buffer[0] = data;
            g_rx_index = 1U;
        }
        return;
    }

    if (g_rx_index == 1U) {
        if (data == 0x55U) {
            g_rx_buffer[1] = data;
            g_rx_index = 2U;
        } else if (data == 0xAAU) {
            g_rx_buffer[0] = data;
        } else {
            g_rx_index = 0U;
        }
        return;
    }

    g_rx_buffer[g_rx_index++] = data;
    if (g_rx_index >= BALL_FRAME_SIZE) {
        ball_publish_frame();
        g_rx_index = 0U;
    }
}

/* 清空残帧并从下一个帧头重新同步 */
static void ball_uart_recover(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
        (void)DL_UART_Main_receiveData(UART_0_INST);
    }
    DL_UART_Main_clearInterruptStatus(UART_0_INST,
                                      BALL_UART_ALL_INTERRUPTS);
    g_rx_index = 0U;
}

/* UART1只接收K230数据，一次读空接收缓冲 */
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
            ball_parse_byte((uint8_t)DL_UART_Main_receiveData(UART_0_INST));
        }
        break;
    case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
    case DL_UART_MAIN_IIDX_BREAK_ERROR:
    case DL_UART_MAIN_IIDX_PARITY_ERROR:
    case DL_UART_MAIN_IIDX_FRAMING_ERROR:
    case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
        g_rx_uart_errors++;
        ball_uart_recover();
        break;
    default:
        break;
    }
}

/* 初始化先输出中位，再启动PWM和接收中断 */
void BallControl_Init(uint32_t now_ms)
{
    g_rx_index = 0U;
    g_rx_generation = 0U;
    g_rx_checksum_errors = 0U;
    g_rx_uart_errors = 0U;
    g_seen_generation = 0U;
    g_last_frame_tick = now_ms;
    g_last_valid_tick = now_ms;
    g_last_control_tick = now_ms;
    g_have_valid_sample = 0U;
    g_have_sequence = 0U;
    g_velocity_integral_mm = 0.0f;
    g_planned_velocity_mm_s = 0.0f;
    g_filtered_velocity_mm_s = 0.0f;
    g_filtered_acceleration_mm_s2 = 0.0f;
    g_breakaway_ticks = 0U;
    g_breakaway_active = 0U;
    g_stall_nudge_ticks = 0U;
    g_stall_nudge_active = 0U;
    g_fine_hold_enabled = 0U;
    g_task6_positioning_assist_enabled = 0U;
    g_task6_position_trim_us = 0.0f;
    ball_task6_reset_nudge();
    g_breakaway_start_position_mm = 0;
    g_stall_nudge_start_position_mm = 0;
    g_motion_feedforward_us = 0;

    g_status.enabled = 0U;
    g_status.link_active = 0U;
    g_status.ball_valid = 0U;
    g_status.confidence = 0U;
    g_status.sequence = 0U;
    g_status.breakaway_active = 0U;
    g_status.safety_reason = (uint8_t)BALL_CONTROL_SAFETY_NONE;
    g_status.target_mm = 0;
    g_status.position_mm = 0;
    g_status.velocity_mm_s = 0;
    g_status.acceleration_mm_s2 = 0;
    g_status.target_velocity_mm_s = 0;
    g_status.control_offset_us = 0;
    g_status.motion_feedforward_us = 0;
    g_status.task6_position_trim_us = 0;
    g_status.frame_count = 0U;
    g_status.checksum_errors = 0U;
    g_status.uart_errors = 0U;
    g_status.sequence_drops = 0U;
    g_status.frame_age_ms = 0xFFFFFFFFU;

    ball_servo_write_us(g_neutral_us);
    DL_TimerA_startCounter(SERVO_PWM_INST);

    /* 复位后丢弃K230残帧 */
    DL_UART_Main_disableInterrupt(UART_0_INST, BALL_UART_ALL_INTERRUPTS);
    ball_uart_recover();
    DL_UART_Main_enableInterrupt(UART_0_INST, BALL_UART_ALL_INTERRUPTS);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

/* 主循环更新快照并决定闭环或回中 */
void BallControl_Process(uint32_t now_ms)
{
    uint32_t generation;
    uint8_t flags;
    uint8_t sequence;
    uint8_t confidence;
    int16_t position_mm;
    uint32_t checksum_errors;
    uint32_t uart_errors;

    NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
    generation = g_rx_generation;
    flags = g_rx_flags;
    sequence = g_rx_sequence;
    confidence = g_rx_confidence;
    position_mm = g_rx_position_mm;
    checksum_errors = g_rx_checksum_errors;
    uart_errors = g_rx_uart_errors;
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    if (generation != g_seen_generation) {
        if (g_have_sequence != 0U) {
            uint8_t expected = (uint8_t)(g_last_sequence + 1U);
            if (sequence != expected) {
                g_status.sequence_drops += (uint8_t)(sequence - expected);
            }
        }
        g_have_sequence = 1U;
        g_last_sequence = sequence;
        g_seen_generation = generation;
        g_last_frame_tick = now_ms;
        g_status.frame_count = generation;
        g_status.link_active = 1U;
        g_status.sequence = sequence;
        if ((flags & BALL_FLAG_VALID) != 0U) {
            uint32_t sample_ms = (uint32_t)(now_ms - g_last_valid_tick);
            int16_t delta_mm = (int16_t)(position_mm - g_status.position_mm);
            float allowed_jump_mm = BALL_POSITION_JUMP_BASE_MM +
                BALL_POSITION_JUMP_SPEED_MM_S * (float)sample_ms / 1000.0f;
            uint8_t continuity_ok =
                (g_have_valid_sample == 0U ||
                 sample_ms > BALL_POSITION_REACQUIRE_MS ||
                 ball_absf((float)delta_mm) <= allowed_jump_mm) ? 1U : 0U;

            if (continuity_ok != 0U) {
                if (g_have_valid_sample != 0U && sample_ms > 0U &&
                    sample_ms <= BALL_POSITION_REACQUIRE_MS) {
                    float previous_velocity_mm_s = g_filtered_velocity_mm_s;
                    float measured_velocity_mm_s = ball_clampf(
                        (float)delta_mm * 1000.0f / (float)sample_ms,
                        -BALL_CONTROL_VELOCITY_LIMIT_MM_S,
                        BALL_CONTROL_VELOCITY_LIMIT_MM_S);
                    float measured_acceleration_mm_s2;
                    g_filtered_velocity_mm_s += BALL_LOCAL_VELOCITY_ALPHA *
                        (measured_velocity_mm_s - g_filtered_velocity_mm_s);
                    measured_acceleration_mm_s2 = ball_clampf(
                        (g_filtered_velocity_mm_s - previous_velocity_mm_s) *
                            1000.0f / (float)sample_ms,
                        -BALL_CONTROL_ACCEL_LIMIT_MM_S2,
                        BALL_CONTROL_ACCEL_LIMIT_MM_S2);
                    g_filtered_acceleration_mm_s2 += BALL_LOCAL_ACCEL_ALPHA *
                        (measured_acceleration_mm_s2 -
                         g_filtered_acceleration_mm_s2);
                } else {
                    g_filtered_velocity_mm_s = 0.0f;
                    g_filtered_acceleration_mm_s2 = 0.0f;
                }
                g_have_valid_sample = 1U;
                g_last_valid_tick = now_ms;
                g_status.confidence = confidence;
                g_status.position_mm = position_mm;
                g_status.velocity_mm_s = (int16_t)g_filtered_velocity_mm_s;
                g_status.acceleration_mm_s2 =
                    (int16_t)g_filtered_acceleration_mm_s2;
            } else {
                /* 跳变帧不参与控制，保留上一有效状态 */
                g_status.confidence = 0U;
            }
        } else {
            /* 无效帧不覆盖最后位置 */
            g_status.confidence = 0U;
        }
    }
    g_status.checksum_errors = checksum_errors;
    g_status.uart_errors = uart_errors;

    if (g_seen_generation == 0U ||
        (uint32_t)(now_ms - g_last_frame_tick) > BALL_LINK_TIMEOUT_MS) {
        g_status.link_active = 0U;
        g_status.ball_valid = 0U;
    } else if (g_have_valid_sample != 0U &&
               (uint32_t)(now_ms - g_last_valid_tick) <= BALL_VALID_HOLD_MS) {
        /* 视觉帧间隔约60到70毫秒，保持180毫秒容忍漏检 */
        g_status.ball_valid = 1U;
    } else {
        g_status.ball_valid = 0U;
    }

    /* 视觉超时回中并可恢复，越界急停需人工处理 */
    if (g_status.enabled != 0U) {
        if (g_status.link_active == 0U || g_status.ball_valid == 0U) {
            g_status.safety_reason = (uint8_t)BALL_CONTROL_SAFETY_VISION;
        } else if (ball_absf((float)g_status.position_mm) >=
                   (float)BALL_EDGE_STOP_MM) {
            BallControl_EStop(BALL_CONTROL_SAFETY_EDGE);
        } else if (g_status.safety_reason ==
                   (uint8_t)BALL_CONTROL_SAFETY_VISION) {
            g_status.safety_reason = (uint8_t)BALL_CONTROL_SAFETY_NONE;
            g_filtered_velocity_mm_s = 0.0f;
            g_filtered_acceleration_mm_s2 = 0.0f;
            g_status.velocity_mm_s = 0;
            g_status.acceleration_mm_s2 = 0;
            ball_arm_breakaway();
        }
    }

    if ((uint32_t)(now_ms - g_last_control_tick) >= BALL_CONTROL_PERIOD_MS) {
        uint16_t target_pulse = g_neutral_us;
        uint16_t slew_step_us = BALL_SERVO_RETURN_SLEW_US;
        float control_us = 0.0f;
        float desired_velocity_mm_s = 0.0f;
        g_last_control_tick = now_ms;

        if (g_status.enabled != 0U && g_status.link_active != 0U &&
            g_status.ball_valid != 0U) {
            float error_mm = (float)(g_status.target_mm - g_status.position_mm);
            float velocity_mm_s = ball_clampf(
                (float)g_status.velocity_mm_s,
                -BALL_CONTROL_VELOCITY_LIMIT_MM_S,
                BALL_CONTROL_VELOCITY_LIMIT_MM_S);
            float acceleration_mm_s2 = ball_clampf(
                (float)g_status.acceleration_mm_s2,
                -BALL_CONTROL_ACCEL_LIMIT_MM_S2,
                BALL_CONTROL_ACCEL_LIMIT_MM_S2);
            float velocity_error_mm_s;
            float safe_speed_mm_s = 0.0f;
            float overspeed_mm_s;
            float overspeed_compensation = 0.0f;
            float acceleration_feedback_us = 0.0f;
            float pulse_us;
            float settle_velocity_limit_mm_s =
                (g_fine_hold_enabled != 0U) ?
                BALL_FINE_HOLD_SETTLE_VELOCITY_MM_S :
                BALL_SETTLE_VELOCITY_MM_S;

            /* 第六问只在停滞时学习水平偏置 */
            if (g_task6_positioning_assist_enabled != 0U &&
                ball_absf(error_mm) > BALL_TASK6_TRIM_START_ERROR_MM &&
                (ball_absf(velocity_mm_s) <=
                     BALL_TASK6_TRIM_STALL_SPEED_MM_S ||
                 error_mm * velocity_mm_s <= 0.0f)) {
                g_task6_position_trim_us +=
                    BALL_CONTROL_DIRECTION * error_mm *
                    BALL_TASK6_TRIM_KI_US_PER_MM_S * BALL_CONTROL_DT_S;
                g_task6_position_trim_us = ball_clampf(
                    g_task6_position_trim_us,
                    -BALL_TASK6_TRIM_LIMIT_US,
                    BALL_TASK6_TRIM_LIMIT_US);
            }
            ball_task6_update_nudge(error_mm, velocity_mm_s);

            if (ball_absf(error_mm) <= BALL_POSITION_DEADBAND_MM &&
                ball_absf(velocity_mm_s) <= settle_velocity_limit_mm_s) {
                g_velocity_integral_mm = 0.0f;
                g_planned_velocity_mm_s = 0.0f;
                g_breakaway_active = 0U;
                g_stall_nudge_active = 0U;
                control_us = 0.0f;
            } else {
                desired_velocity_mm_s =
                    ball_plan_velocity(error_mm, &safe_speed_mm_s);
                velocity_error_mm_s = desired_velocity_mm_s - velocity_mm_s;
                overspeed_mm_s = ball_absf(velocity_mm_s) - safe_speed_mm_s;
                if (overspeed_mm_s > 0.0f) {
                    overspeed_compensation =
                        ((velocity_mm_s >= 0.0f) ? -1.0f : 1.0f) *
                        BALL_OVERSPEED_COMP_US_PER_MM_S * overspeed_mm_s;
                }

                /* 加速或远离目标时才启用加速度反馈 */
                if ((acceleration_mm_s2 * velocity_mm_s) > 0.0f ||
                    (acceleration_mm_s2 * error_mm) < 0.0f) {
                    acceleration_feedback_us =
                        -g_params.acceleration_damping_us_per_mm_s2 *
                         acceleration_mm_s2;
                }

                if (g_breakaway_active == 0U) {
                    g_velocity_integral_mm +=
                        velocity_error_mm_s * BALL_CONTROL_DT_S;
                    if (g_velocity_integral_mm > BALL_VELOCITY_INTEGRAL_LIMIT_MM) {
                        g_velocity_integral_mm = BALL_VELOCITY_INTEGRAL_LIMIT_MM;
                    } else if (g_velocity_integral_mm < -BALL_VELOCITY_INTEGRAL_LIMIT_MM) {
                        g_velocity_integral_mm = -BALL_VELOCITY_INTEGRAL_LIMIT_MM;
                    }
                } else {
                    g_velocity_integral_mm = 0.0f;
                }

                control_us = BALL_CONTROL_DIRECTION *
                    (g_params.velocity_kp_us_per_mm_s * velocity_error_mm_s +
                     g_params.velocity_ki_us_per_mm * g_velocity_integral_mm -
                     g_params.velocity_damping_us_per_mm_s * velocity_mm_s +
                     acceleration_feedback_us +
                     overspeed_compensation);

                /* 慢速大偏差时补偿静摩擦 */
                if (ball_absf(error_mm) >= BALL_STATIC_ENABLE_ERROR_MM &&
                    ball_absf(velocity_mm_s) <=
                        ((g_fine_hold_enabled != 0U) ?
                         BALL_RETURN_STATIC_MAX_VELOCITY_MM_S :
                         BALL_STATIC_MAX_VELOCITY_MM_S) &&
                    ball_absf(desired_velocity_mm_s) >= 15.0f) {
                    float static_comp_us = (float)g_params.static_comp_us;
                    if (g_fine_hold_enabled != 0U &&
                        ball_absf(error_mm) < BALL_FINE_HOLD_ZONE_MM) {
                        static_comp_us *= BALL_FINE_HOLD_STATIC_SCALE;
                    }
                    if ((BALL_CONTROL_DIRECTION * error_mm) > 0.0f) {
                        control_us += static_comp_us;
                    } else {
                        control_us -= static_comp_us;
                    }
                }

                /* 目标刚变化且钢球未滚动时给短时最低输出 */
                if (g_breakaway_active != 0U) {
                    int16_t moved_mm = (int16_t)(g_status.position_mm -
                                                  g_breakaway_start_position_mm);
                    if (moved_mm < 0) moved_mm = (int16_t)-moved_mm;
                    if ((float)moved_mm >= (float)BALL_BREAKAWAY_MOVE_MM ||
                        ball_absf(velocity_mm_s) >= BALL_BREAKAWAY_STOP_MM_S ||
                        ball_absf(error_mm) < BALL_BREAKAWAY_ERROR_MM ||
                        g_breakaway_ticks >= BALL_BREAKAWAY_MAX_TICKS) {
                        g_breakaway_active = 0U;
                    } else {
                        if ((BALL_CONTROL_DIRECTION * error_mm) > 0.0f) {
                            if (control_us < BALL_BREAKAWAY_MIN_US) {
                                control_us = BALL_BREAKAWAY_MIN_US;
                            }
                        } else {
                            if (control_us > -BALL_BREAKAWAY_MIN_US) {
                                control_us = -BALL_BREAKAWAY_MIN_US;
                            }
                        }
                        g_breakaway_ticks++;
                    }
                }

                /* 第三问停滞时使用更短更弱的助推 */
                if (g_stall_nudge_active != 0U) {
                    int16_t moved_mm = (int16_t)(g_status.position_mm -
                                                  g_stall_nudge_start_position_mm);
                    if (moved_mm < 0) moved_mm = (int16_t)-moved_mm;
                    if (moved_mm >= BALL_STALL_NUDGE_MOVE_MM ||
                        ball_absf(velocity_mm_s) >= BALL_STALL_NUDGE_STOP_MM_S ||
                        ball_absf(error_mm) <= BALL_STALL_NUDGE_TARGET_ZONE_MM ||
                        g_stall_nudge_ticks >= BALL_STALL_NUDGE_MAX_TICKS) {
                        g_stall_nudge_active = 0U;
                    } else {
                        if ((BALL_CONTROL_DIRECTION * error_mm) > 0.0f) {
                            if (control_us < BALL_STALL_NUDGE_MIN_US) {
                                control_us = BALL_STALL_NUDGE_MIN_US;
                            }
                        } else {
                            if (control_us > -BALL_STALL_NUDGE_MIN_US) {
                                control_us = -BALL_STALL_NUDGE_MIN_US;
                            }
                        }
                        g_stall_nudge_ticks++;
                    }
                }
            }

            if (g_task6_positioning_assist_enabled != 0U) {
                control_us += g_task6_position_trim_us;
            }
            control_us += (float)g_motion_feedforward_us;
            control_us = ball_task6_apply_nudge(control_us);
            control_us = ball_clamp_control_us(control_us);
            pulse_us = (float)g_neutral_us + control_us;
            if (pulse_us < (float)BALL_SERVO_MIN_US) {
                pulse_us = (float)BALL_SERVO_MIN_US;
            } else if (pulse_us > (float)BALL_SERVO_MAX_US) {
                pulse_us = (float)BALL_SERVO_MAX_US;
            }
            target_pulse = (uint16_t)pulse_us;

            if ((control_us * velocity_mm_s) > 0.0f) {
                slew_step_us = BALL_SERVO_BRAKE_SLEW_US;
            } else if (ball_absf(control_us) > 0.5f) {
                slew_step_us = BALL_SERVO_DRIVE_SLEW_US;
            }
        } else {
            g_velocity_integral_mm = 0.0f;
            g_planned_velocity_mm_s = 0.0f;
            g_breakaway_active = 0U;
            g_stall_nudge_active = 0U;
            g_task6_nudge_active_ticks = 0U;
            g_task6_nudge_stall_ticks = 0U;
        }

        g_status.target_velocity_mm_s = (int16_t)desired_velocity_mm_s;
        g_status.control_offset_us = (int16_t)control_us;
        g_status.task6_position_trim_us =
            (int16_t)g_task6_position_trim_us;
        g_status.breakaway_active =
            (g_breakaway_active != 0U || g_stall_nudge_active != 0U ||
             g_task6_nudge_active_ticks != 0U) ? 1U : 0U;
        ball_servo_write_us(ball_slew_pulse(target_pulse, g_status.pulse_us,
                                             slew_step_us));
    }
}

/* 记录当前位置，用位移判断是否已起动 */
static void ball_arm_breakaway(void)
{
    g_velocity_integral_mm = 0.0f;
    g_breakaway_start_position_mm = g_status.position_mm;
    g_breakaway_ticks = 0U;
    g_breakaway_active = 1U;
    g_stall_nudge_ticks = 0U;
    g_stall_nudge_active = 0U;
    g_status.breakaway_active = 1U;
}

void BallControl_EStop(BallControlSafetyReason reason)
{
    g_status.enabled = 0U;
    g_status.safety_reason = (uint8_t)reason;
    g_status.target_velocity_mm_s = 0;
    g_status.control_offset_us = 0;
    g_status.motion_feedforward_us = 0;
    g_status.task6_position_trim_us = 0;
    g_motion_feedforward_us = 0;
    g_task6_position_trim_us = 0.0f;
    ball_task6_reset_nudge();
    g_velocity_integral_mm = 0.0f;
    g_planned_velocity_mm_s = 0.0f;
    g_breakaway_ticks = 0U;
    g_breakaway_active = 0U;
    g_stall_nudge_ticks = 0U;
    g_stall_nudge_active = 0U;
    g_status.breakaway_active = 0U;

    /* 急停绕过斜率限制，立即回中 */
    ball_servo_write_us(g_neutral_us);
}

/* 关闭时立即回中，边缘故障需人工复位 */
void BallControl_SetEnabled(uint8_t enabled)
{
    if (enabled == 0U) {
        BallControl_EStop(BALL_CONTROL_SAFETY_NONE);
        return;
    }
    if (ball_absf((float)g_status.position_mm) >
        (float)BALL_EDGE_REARM_MM) {
        g_status.safety_reason = (uint8_t)BALL_CONTROL_SAFETY_EDGE;
        return;
    }

    g_status.safety_reason = (uint8_t)BALL_CONTROL_SAFETY_NONE;
    g_status.enabled = 1U;
    g_planned_velocity_mm_s = 0.0f;
    ball_arm_breakaway();
}

/* 目标限制在有效槽长内 */
void BallControl_SetTargetMm(int16_t target_mm)
{
    if (target_mm > BALL_TARGET_LIMIT_MM) target_mm = BALL_TARGET_LIMIT_MM;
    if (target_mm < -BALL_TARGET_LIMIT_MM) target_mm = -BALL_TARGET_LIMIT_MM;
    if (target_mm != g_status.target_mm) {
        float new_error_mm = (float)(target_mm - g_status.position_mm);

        /* 到达正5厘米后回程需重新克服静摩擦 */
        if (g_status.enabled != 0U &&
            ball_absf(new_error_mm) >= BALL_STATIC_ENABLE_ERROR_MM &&
            ball_absf((float)g_status.velocity_mm_s) <=
                ((g_fine_hold_enabled != 0U) ?
                 BALL_RETURN_STATIC_MAX_VELOCITY_MM_S :
                 BALL_STATIC_MAX_VELOCITY_MM_S)) {
            ball_arm_breakaway();
        } else {
            /* 高速改目标交给速度环制动 */
            g_breakaway_ticks = 0U;
            g_breakaway_active = 0U;
            g_stall_nudge_ticks = 0U;
            g_stall_nudge_active = 0U;
            g_status.breakaway_active = 0U;
        }
    }
    g_status.target_mm = target_mm;
    g_velocity_integral_mm = 0.0f;
    g_planned_velocity_mm_s = 0.0f;
}

/* 中位标定立即生效 */
void BallControl_SetNeutralUs(uint16_t pulse_us)
{
    g_neutral_us = ball_clamp_pulse(pulse_us);
    g_velocity_integral_mm = 0.0f;
    g_planned_velocity_mm_s = 0.0f;
    g_breakaway_active = 0U;
    g_stall_nudge_ticks = 0U;
    g_stall_nudge_active = 0U;
    g_status.breakaway_active = 0U;
    g_status.target_velocity_mm_s = 0;
    g_status.control_offset_us = 0;
    ball_servo_write_us(g_neutral_us);
}

/* 限制现场调参范围，防止摆杆倾角过大 */
void BallControl_SetStaticCompUs(uint16_t compensation_us)
{
    if (compensation_us > BALL_STATIC_MAX_COMP_US) {
        compensation_us = BALL_STATIC_MAX_COMP_US;
    }
    g_params.static_comp_us = compensation_us;
}

void BallControl_SetFineHold(uint8_t enabled)
{
    g_fine_hold_enabled = (enabled != 0U) ? 1U : 0U;
    if (g_fine_hold_enabled != 0U) {
        g_stall_nudge_ticks = 0U;
        g_stall_nudge_active = 0U;
    }
}

void BallControl_SetTask6PositioningAssist(uint8_t enabled)
{
    uint8_t next_enabled = (enabled != 0U) ? 1U : 0U;

    /* 正目标预下沉，重复使能保留已学偏置 */
    if (next_enabled != g_task6_positioning_assist_enabled) {
        g_task6_position_trim_us =
            (next_enabled != 0U && g_status.target_mm > 0) ?
            BALL_TASK6_POSITIVE_TRIM_INITIAL_US : 0.0f;
        g_status.task6_position_trim_us =
            (int16_t)g_task6_position_trim_us;
        ball_task6_reset_nudge();
    }
    g_task6_positioning_assist_enabled = next_enabled;
}

void BallControl_SetTask6PositionTrimUs(int16_t trim_us)
{
    if (trim_us > (int16_t)BALL_TASK6_TRIM_LIMIT_US) {
        trim_us = (int16_t)BALL_TASK6_TRIM_LIMIT_US;
    } else if (trim_us < -(int16_t)BALL_TASK6_TRIM_LIMIT_US) {
        trim_us = (int16_t)-BALL_TASK6_TRIM_LIMIT_US;
    }
    g_task6_position_trim_us = (float)trim_us;
    g_status.task6_position_trim_us = trim_us;
}

uint8_t BallControl_RequestStallNudge(void)
{
    float error_mm = (float)(g_status.target_mm - g_status.position_mm);

    if (g_status.enabled == 0U || g_status.link_active == 0U ||
        g_status.ball_valid == 0U || g_fine_hold_enabled != 0U ||
        g_breakaway_active != 0U ||
        ball_absf(error_mm) <= BALL_STALL_NUDGE_TARGET_ZONE_MM) {
        return 0U;
    }

    g_velocity_integral_mm = 0.0f;
    g_stall_nudge_start_position_mm = g_status.position_mm;
    g_stall_nudge_ticks = 0U;
    g_stall_nudge_active = 1U;
    g_status.breakaway_active = 1U;
    return 1U;
}

void BallControl_SetMotionFeedforwardUs(int16_t feedforward_us)
{
    if (feedforward_us > BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US) {
        feedforward_us = BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US;
    } else if (feedforward_us < -BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US) {
        feedforward_us = -BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US;
    }
    g_motion_feedforward_us = feedforward_us;
    g_status.motion_feedforward_us = feedforward_us;
}

uint8_t BallControl_SetParams(const BallControlParams *params)
{
    if (params == 0 ||
        !(params->position_kp_per_s >= BALL_POSITION_KP_MIN &&
          params->position_kp_per_s <= BALL_POSITION_KP_MAX) ||
        !(params->velocity_kp_us_per_mm_s >= BALL_VELOCITY_KP_MIN &&
          params->velocity_kp_us_per_mm_s <= BALL_VELOCITY_KP_MAX) ||
        !(params->velocity_ki_us_per_mm >= BALL_VELOCITY_KI_MIN &&
          params->velocity_ki_us_per_mm <= BALL_VELOCITY_KI_MAX) ||
        !(params->velocity_damping_us_per_mm_s >= BALL_VELOCITY_DAMP_MIN &&
          params->velocity_damping_us_per_mm_s <= BALL_VELOCITY_DAMP_MAX) ||
        !(params->acceleration_damping_us_per_mm_s2 >= BALL_ACCEL_DAMP_MIN &&
          params->acceleration_damping_us_per_mm_s2 <= BALL_ACCEL_DAMP_MAX) ||
        params->static_comp_us > BALL_STATIC_MAX_COMP_US) {
        return 0U;
    }

    /* 命令在主循环解析，避免与控制计算并发 */
    g_params = *params;
    g_velocity_integral_mm = 0.0f;
    g_planned_velocity_mm_s = 0.0f;
    return 1U;
}

void BallControl_GetParams(BallControlParams *params)
{
    if (params != 0) *params = g_params;
}

void BallControl_ResetParams(void)
{
    BallControlParams defaults = {
        BALL_POSITION_KP_DEFAULT_PER_S,
        BALL_VELOCITY_KP_DEFAULT,
        BALL_VELOCITY_KI_DEFAULT,
        BALL_VELOCITY_DAMP_DEFAULT,
        BALL_ACCEL_DAMP_DEFAULT,
        (uint16_t)BALL_STATIC_DEFAULT_COMP_US
    };
    (void)BallControl_SetParams(&defaults);
}

/* 读取状态时只关闭视觉中断 */
void BallControl_GetStatus(BallControlStatus *status, uint32_t now_ms)
{
    if (status == 0) return;
    NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
    *status = g_status;
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    if (status->link_active != 0U) {
        status->frame_age_ms = (uint32_t)(now_ms - g_last_frame_tick);
    } else {
        status->frame_age_ms = 0xFFFFFFFFU;
    }
}
