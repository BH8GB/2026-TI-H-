/* 文件 ball_autotune.h，用途 串口调参接口 */

#ifndef BALL_AUTOTUNE_H
#define BALL_AUTOTUNE_H

#include <stdint.h>

/* 同步发送回调 */
typedef void (*BallAutotuneSendFn)(const char *text);

/* 命令解析结果 */
typedef enum {
    BALL_AUTOTUNE_NOT_HANDLED = 0,
    BALL_AUTOTUNE_HANDLED,
    BALL_AUTOTUNE_START_TRIAL,
    BALL_AUTOTUNE_STOP_TRIAL
} BallAutotuneAction;

/* 初始化遥测和试验状态 */
void BallAutotune_Init(uint32_t now_ms);

/* 解析大写B开头命令 */
BallAutotuneAction BallAutotune_HandleCommand(const char *command,
                                               BallAutotuneSendFn send);

/* 同步第三问试验占用状态 */
void BallAutotune_SetTrialActive(uint8_t active);

/* 是否正在自动调参 */
uint8_t BallAutotune_IsTrialActive(void);

/* PB21启动第三问采集，结束后短暂续传 */
void BallAutotune_StartWheeltecCapture(uint32_t now_ms);

/* PA26启动第四问采集 */
void BallAutotune_StartTask4Capture(uint32_t now_ms);

/* PB9启动第五问采集 */
void BallAutotune_StartTask5Capture(uint32_t now_ms);

/* PB14再次按下启动第六问预定位诊断 */
void BallAutotune_StartTask6PositionCapture(uint32_t now_ms);

/* PB14或命令6启动第六问采集 */
void BallAutotune_StartTask6Capture(uint32_t now_ms);

/* 串口是否被波形帧占用 */
uint8_t BallAutotune_IsWheeltecCaptureActive(void);

/* 仅停止波形输出 */
void BallAutotune_StopWheeltecCapture(void);

/* 以20赫兹输出波形帧，也可输出CSV */
void BallAutotune_Process(uint32_t now_ms, BallAutotuneSendFn send);

#endif
