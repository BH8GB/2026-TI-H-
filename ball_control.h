/* 文件 ball_control.h，用途 钢球控制接口 */

#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H

#include <stdint.h>

/* 车身前馈最大脉宽，单位微秒 */
#define BALL_CONTROL_MOTION_FEEDFORWARD_MAX_US 120

/* 安全停机原因 */
typedef enum {
    BALL_CONTROL_SAFETY_NONE = 0,
    BALL_CONTROL_SAFETY_EDGE,
    BALL_CONTROL_SAFETY_VISION
} BallControlSafetyReason;

/* 运行时PID参数 */
typedef struct {
    float position_kp_per_s;
    float velocity_kp_us_per_mm_s;
    float velocity_ki_us_per_mm;
    float velocity_damping_us_per_mm_s;
    float acceleration_damping_us_per_mm_s2;
    uint16_t static_comp_us;
} BallControlParams;

/* 控制状态快照 */
typedef struct {
    uint8_t enabled;
    uint8_t link_active;
    uint8_t ball_valid;
    uint8_t confidence;
    uint8_t sequence;
    uint8_t breakaway_active;
    uint8_t safety_reason;
    int16_t target_mm;
    int16_t position_mm;
    int16_t velocity_mm_s;
    int16_t acceleration_mm_s2;
    int16_t target_velocity_mm_s;
    int16_t control_offset_us;
    int16_t motion_feedforward_us;
    int16_t task6_position_trim_us;
    uint16_t pulse_us;
    uint32_t frame_count;
    uint32_t checksum_errors;
    uint32_t uart_errors;
    uint32_t sequence_drops;
    uint32_t frame_age_ms;
} BallControlStatus;

/* 初始化UART1和舵机PWM */
void BallControl_Init(uint32_t now_ms);

/* 主循环调用，20毫秒周期 */
void BallControl_Process(uint32_t now_ms);

/* 开启或关闭闭环，关闭后回中 */
void BallControl_SetEnabled(uint8_t enabled);

/* 设置目标位置，单位毫米 */
void BallControl_SetTargetMm(int16_t target_mm);

/* 设置水平中位脉宽，单位微秒 */
void BallControl_SetNeutralUs(uint16_t pulse_us);

/* 设置静摩擦补偿，单位微秒 */
void BallControl_SetStaticCompUs(uint16_t compensation_us);

/* 开启末端精细保持 */
void BallControl_SetFineHold(uint8_t enabled);

/* 开启第六问预定位助推 */
void BallControl_SetTask6PositioningAssist(uint8_t enabled);

/* 设置第六问启动偏置，不改静态中位 */
void BallControl_SetTask6PositionTrimUs(int16_t trim_us);

/* 请求一次短促助推，用于中途卡滞 */
uint8_t BallControl_RequestStallNudge(void);

/* 添加车身加速度前馈 */
void BallControl_SetMotionFeedforwardUs(int16_t feedforward_us);

/* 整组提交参数，越界则拒绝 */
uint8_t BallControl_SetParams(const BallControlParams *params);

/* 读取当前参数 */
void BallControl_GetParams(BallControlParams *params);

/* 恢复默认参数 */
void BallControl_ResetParams(void);

/* 立即停机并回中 */
void BallControl_EStop(BallControlSafetyReason reason);

/* 读取状态快照 */
void BallControl_GetStatus(BallControlStatus *status, uint32_t now_ms);

#endif
