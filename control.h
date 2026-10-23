/* 文件 control.h，用途 航向控制接口 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

void YawControl(float base_speed, float target_yaw);  /* 直接输出PWM航向控制 */
float Control_ComputeYawTurn(float base_speed, float target_yaw); /* 仅计算差速修正 */
void Control_SetYawTarget(float yaw);                 /* 设置目标角度 */
float Control_GetYawTarget(void);
float Control_YawError(float target, float current);
void Control_ResetYawPid(void);

void Control_CapturePoseTarget(void);
void Control_ResetPoseHold(void);
void Control_PoseHoldStep(void);
int32_t Control_GetPoseError(void);

#endif
