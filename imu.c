/* 文件 imu.c，用途 MPU6050串口解析 */

#include "imu.h"
#include "ti_msp_dl_config.h"
#include "led.h"

static volatile float g_yaw  = 0.0f;
static volatile float g_gyro = 0.0f;
static volatile uint32_t g_yaw_tick;
static volatile uint32_t g_gyro_tick;
static volatile uint8_t g_yaw_valid;
static volatile uint8_t g_gyro_valid;
static volatile uint16_t g_imu_errors;

extern volatile uint32_t g_sys_tick;

static void imu_send(const uint8_t command[5])
{
    for (uint8_t i = 0; i < 5U; i++) {
        DL_UART_transmitDataBlocking(UART_IMU_INST, command[i]);
    }
}

void IMU_Init(void)
{
    NVIC_ClearPendingIRQ(UART_IMU_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
    IMU_Calibrate();
}

void IMU_Calibrate(void)
{
    static const uint8_t unlock[] = {0x55, 0xAA, 0x13, 0x8E, 0x5F};
    static const uint8_t yaw_zero[] = {0x55, 0xAA, 0x15, 0x00, 0x00};
    static const uint8_t save[] = {0x55, 0xAA, 0x00, 0x00, 0x00};

    g_yaw_valid = 0;
    g_gyro_valid = 0;
    imu_send(unlock);
    delay_ms(100U);
    imu_send(yaw_zero);
    delay_ms(100U);
    imu_send(save);
}

float IMU_Yaw(void)   { return g_yaw; }
float IMU_GyroZ(void) { return g_gyro; }

uint32_t IMU_GetYawAge(uint32_t now)
{
    return g_yaw_valid ? now - g_yaw_tick : 0xFFFFFFFFU;
}

uint32_t IMU_GetGyroAge(uint32_t now)
{
    return g_gyro_valid ? now - g_gyro_tick : 0xFFFFFFFFU;
}

uint8_t IMU_IsFresh(uint32_t now, uint32_t max_age_ms)
{
    return g_yaw_valid && g_gyro_valid &&
           (uint32_t)(now - g_yaw_tick) <= max_age_ms &&
           (uint32_t)(now - g_gyro_tick) <= max_age_ms;
}

uint16_t IMU_GetErrorCount(void) { return g_imu_errors; }

void IMU_GetSnapshot(IMUSnapshot *snapshot)
{
    if (snapshot == 0) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    snapshot->yaw = g_yaw;
    snapshot->gyro_z = g_gyro;
    snapshot->yaw_tick = g_yaw_tick;
    snapshot->gyro_tick = g_gyro_tick;
    snapshot->yaw_valid = g_yaw_valid;
    snapshot->gyro_valid = g_gyro_valid;
    if (primask == 0U) __enable_irq();
}

void IMU_Service(void) {}

/* 解析5字节陀螺仪帧 */
static void imu_parse(uint8_t c)
{
    static uint8_t buf[5], idx;

    buf[idx++] = c;
    if (buf[0] != 0x5A) { idx = 0; return; }
    if (idx < 5) return;

    uint8_t sum = buf[0] + buf[1] + buf[2] + buf[3];
    if (sum != buf[4]) {
        g_imu_errors++;
        idx = (c == 0x5A) ? 1U : 0U;
        if (idx != 0U) buf[0] = c;
        return;
    }

    if (buf[1] == 0xAA) {
        short wz = (short)((buf[3] << 8) | buf[2]);
        g_gyro = (float)wz / 32768.0f * 2000.0f;
        g_gyro_tick = g_sys_tick;
        g_gyro_valid = 1;
    } else if (buf[1] == 0xBB) {
        short yaw = (short)((buf[3] << 8) | buf[2]);
        g_yaw  = (float)yaw / 32768.0f * 180.0f;
        g_yaw_tick = g_sys_tick;
        g_yaw_valid = 1;
    } else {
        g_imu_errors++;
    }
    idx = 0;
}

/* UART2中断 */
void UART_IMU_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_IMU_INST) == DL_UART_IIDX_RX) {
        imu_parse((uint8_t)UART_IMU_INST->RXDATA);
    }
}
