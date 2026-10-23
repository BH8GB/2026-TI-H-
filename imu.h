/* 文件 imu.h，用途 IMU接口 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    float yaw;
    float gyro_z;
    uint32_t yaw_tick;
    uint32_t gyro_tick;
    uint8_t yaw_valid;
    uint8_t gyro_valid;
} IMUSnapshot;

void IMU_Init(void);               /* 初始化UART2并发送校准 */
float IMU_Yaw(void);               /* 返回当前航向角，单位度 */
float IMU_GyroZ(void);             /* 返回Z轴角速度，单位度每秒 */
void IMU_Calibrate(void);          /* 发送航向归零指令 */

uint8_t IMU_IsFresh(uint32_t now, uint32_t max_age_ms);
uint32_t IMU_GetYawAge(uint32_t now);
uint32_t IMU_GetGyroAge(uint32_t now);
uint16_t IMU_GetErrorCount(void);
void IMU_GetSnapshot(IMUSnapshot *snapshot);

void IMU_Service(void);            /* 主循环数据处理 */

#endif
