#ifndef TASK_PROFILES_H
#define TASK_PROFILES_H

#include "ball_control.h"
#include <stdint.h>

/* 第五和第六问各保存一套PID参数 */
typedef struct {
    float wheel_kp;
    float wheel_ki;
    float track_kp;
    BallControlParams ball;
} TaskLapPidProfile;

/* 每个任务独立参数，加载一套不影响其他任务 */
uint8_t TaskProfiles_LoadTask2(void);
uint8_t TaskProfiles_LoadTask3(void);
uint8_t TaskProfiles_LoadTask4(void);
/* 只加载第四问PID，保留运动参数 */
uint8_t TaskProfiles_LoadTask4Pid(void);
uint8_t TaskProfiles_LoadTask5(void);
uint8_t TaskProfiles_LoadTask6(void);

/* 读取第五和第六问PID */
void TaskProfiles_GetTask5Pid(TaskLapPidProfile *profile);
void TaskProfiles_GetTask6Pid(TaskLapPidProfile *profile);

/* 修改第五和第六问PID，整组合法才提交 */
uint8_t TaskProfiles_SetTask5Pid(const TaskLapPidProfile *profile);
uint8_t TaskProfiles_SetTask6Pid(const TaskLapPidProfile *profile);

/* 恢复第五和第六问默认PID */
void TaskProfiles_ResetTask5Pid(void);
void TaskProfiles_ResetTask6Pid(void);

#endif
