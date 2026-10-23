/* 文件 sensor.c，用途 八路灰度循迹 */

#include "sensor.h"
#include "wheel_speed.h"
#include "ti_msp_dl_config.h"
#include <string.h>

uint8_t  g_sensor_raw[8];
uint8_t  g_last_rx_byte;
uint16_t g_rx_total;
static uint16_t g_valid_frames;
static uint16_t g_bad_frames;
static uint16_t g_rx_overflow;

#define RX_SZ  256
static volatile char  g_rx[RX_SZ];
static volatile uint16_t g_wr;
static volatile uint16_t g_rd;

static int16_t g_base_speed = 100; /* 循迹基础轮速，单位RPM */
static float g_kp = 1.0f;          /* 灰度误差到转速差的比例 */
#define TRACK_CORR_LIMIT_RPM 40
#define TRACK_SEARCH_RPM     20
static uint8_t g_track_run;

static uint8_t g_line_ok;
static uint32_t g_frame_tick;
static uint32_t g_lost_tick;
static int8_t g_last_error;
static uint8_t g_have_last_error;

extern volatile uint32_t g_sys_tick;

static void track_send(const char *s) {
    while (*s) DL_UART_transmitDataBlocking(UART_TRACK_INST, (uint8_t)*s++);
}

/* UART3中断，保存接收字节 */
static volatile char g_last_4[4];  /* 最近4字节 */
static volatile uint8_t g_last_idx;

void UART3_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_TRACK_INST) == DL_UART_IIDX_RX) {
        uint8_t c = (uint8_t)UART_TRACK_INST->RXDATA;
        g_last_rx_byte = c;
        g_rx_total++;
        /* 保存最近4字节供OLED显示 */
        g_last_4[g_last_idx++ & 3] = (char)c;
        /* 同时写入环形缓冲区 */
        uint16_t next = (uint16_t)(g_wr + 1U);
        if (next >= RX_SZ) next = 0U;
        if (next != g_rd) {
            g_rx[g_wr] = (char)c;
            g_wr = next;
        } else {
            g_rx_overflow++;
        }
    }
}

/* OLED显示最后4字节 */
const char *Sensor_GetLast4(void) { return (const char *)g_last_4; }

/* 解析灰度传感器D帧 */
static void parse_dframe(const char *buf)
{
    uint8_t parsed[8];
    const char *p = buf + 2;

    for (int i = 0; i < 8; i++) {
        p = strchr(p, ':');
        if (!p || (p[1] != '0' && p[1] != '1')) {
            g_bad_frames++;
            return;
        }
        parsed[i] = (p[1] == '0') ? 1U : 0U;
        p += 2;
        if (i < 7) {
            p = strchr(p, ',');
            if (!p) {
                g_bad_frames++;
                return;
            }
            p++;
        }
    }

    if (!strchr(p, '#')) {
        g_bad_frames++;
        return;
    }

    memcpy(g_sensor_raw, parsed, sizeof(parsed));
    g_valid_frames++;
    g_line_ok = 1;
    g_frame_tick = g_sys_tick;
}

/* 从环形缓冲区提取完整帧 */
static uint8_t rx_pop(char *c)
{
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    if (g_rd == g_wr) {
        if (p == 0U) __enable_irq();
        return 0U;
    }
    *c = g_rx[g_rd++];
    if (g_rd >= RX_SZ) g_rd = 0;
    if (p == 0U) __enable_irq();
    return 1U;
}

static void process_rx(void)
{
    static char frame[128];
    static uint8_t len;
    char c;
    while (rx_pop(&c)) {
        if (c == '$') len = 0;
        if (len < 127U) frame[len++] = c;
        if (c == '#' && len >= 3U && frame[0] == '$' && frame[1] == 'D') {
            frame[len] = '\0';
            parse_dframe(frame);
            len = 0;
        }
    }
}

/* 对外接口 */
void Sensor_Init(void) {
    NVIC_ClearPendingIRQ(UART_TRACK_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_TRACK_INST_INT_IRQN);
    /* 发送指令后立即返回，数据由主循环处理 */
    track_send("$0,0,1#");
}

void Sensor_Read(void) { process_rx(); }

int8_t Sensor_CalcError(void) {
    if (!g_line_ok) return -128;
    static const int8_t w[8] = {-72,-54,-40,-30,30,40,54,72};
    int16_t sw=0, sn=0;
    for (int i=0; i<8; i++) { if (g_sensor_raw[i]) { sw+=w[i]; sn++; } }
    if (sn==0) return -128;
    return (int8_t)(sw/sn);
}

static uint8_t Sensor_CalcError_ok(void) {
    for (int i = 0; i < 8; i++) { if (g_sensor_raw[i]) return 1; }
    return 0;
}

void Track_Start(void)
{
    uint32_t now = g_sys_tick;
    g_track_run = 1;
    g_line_ok = 0;
    g_lost_tick = now;
    g_frame_tick = now;
    g_last_error = 0;
    g_have_last_error = 0;
    track_send("$0,0,1#");
}

void Track_Stop(void)
{
    g_track_run = 0;
    g_have_last_error = 0;
}

uint8_t Track_IsRunning(void)   { return g_track_run; }
void Track_SetBaseSpeed(int16_t s) { g_base_speed = s; }
void Track_SetKp(float kp)         { g_kp = kp; }
int16_t Track_GetBaseSpeed(void)   { return g_base_speed; }
uint8_t Track_GetLineOk(void)      { return g_line_ok; }
uint32_t Track_GetFrameAge(uint32_t now) { return g_valid_frames ? now - g_frame_tick : 0xFFFFFFFFU; }
uint16_t Track_GetValidFrames(void) { return g_valid_frames; }
uint16_t Track_GetBadFrames(void) { return g_bad_frames; }
uint16_t Track_GetOverflowCount(void) { return g_rx_overflow; }
uint8_t Track_GetSensorMask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 8U; i++) {
        if (g_sensor_raw[i]) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

uint8_t Track_ComputeWheelTargets(uint32_t now, int16_t *left, int16_t *right)
{
    if (!g_track_run || left == 0 || right == 0) return 0U;

    Sensor_Read();

    int32_t target_l, target_r;
    uint8_t frame_fresh = g_line_ok &&
        ((uint32_t)(now - g_frame_tick) <= 100U);
    uint8_t sensor_has_line = Sensor_CalcError_ok();

    if (frame_fresh && sensor_has_line) {
        int16_t err = Sensor_CalcError();
        g_last_error = (int8_t)err;
        g_have_last_error = 1U;
        g_lost_tick = now;

        float corr_f = (float)err * g_kp;
        if (corr_f > (float)TRACK_CORR_LIMIT_RPM) {
            corr_f = (float)TRACK_CORR_LIMIT_RPM;
        } else if (corr_f < -(float)TRACK_CORR_LIMIT_RPM) {
            corr_f = -(float)TRACK_CORR_LIMIT_RPM;
        }
        int32_t corr = (int32_t)corr_f;
        target_l = (int32_t)g_base_speed + corr;
        target_r = (int32_t)g_base_speed - corr;
    } else if (frame_fresh && !sensor_has_line) {
        /* 全白表示暂时脱线，保持直行；数据过期则停车 */
        target_l = g_base_speed;
        target_r = g_base_speed;
        g_lost_tick = now;
    } else if (g_have_last_error && (uint32_t)(now - g_lost_tick) <= 250U) {
        if (g_last_error < 0) {
            target_l = -TRACK_SEARCH_RPM;
            target_r = TRACK_SEARCH_RPM;
        } else {
            target_l = TRACK_SEARCH_RPM;
            target_r = -TRACK_SEARCH_RPM;
        }
    } else {
        *left = 0;
        *right = 0;
        return 0U;
    }

    if (target_l > WHEEL_SPEED_MAX_RPM) target_l = WHEEL_SPEED_MAX_RPM;
    if (target_l < -WHEEL_SPEED_MAX_RPM) target_l = -WHEEL_SPEED_MAX_RPM;
    if (target_r > WHEEL_SPEED_MAX_RPM) target_r = WHEEL_SPEED_MAX_RPM;
    if (target_r < -WHEEL_SPEED_MAX_RPM) target_r = -WHEEL_SPEED_MAX_RPM;

    *left = (int16_t)target_l;
    *right = (int16_t)target_r;
    return 1U;
}
