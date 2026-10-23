/* 文件 encoder.c，用途 双轮编码器x2解码 */

#include "encoder.h"
#include "ti_msp_dl_config.h"

/* 编码器引脚 PA15 PA16 PA17 PA24 */
#define L_A_PIN   DL_GPIO_PIN_17
#define L_A_IOMUX IOMUX_PINCM39
#define L_B_PIN   DL_GPIO_PIN_24
#define L_B_IOMUX IOMUX_PINCM54
#define R_A_PIN   DL_GPIO_PIN_15
#define R_A_IOMUX IOMUX_PINCM37
#define R_B_PIN   DL_GPIO_PIN_16
#define R_B_IOMUX IOMUX_PINCM38

static volatile int32_t g_l_cnt, g_l_dlt, g_r_cnt, g_r_dlt;

#define CRIT_ENTER  uint32_t _p_ = __get_PRIMASK(); __disable_irq()
#define CRIT_EXIT   if (_p_ == 0U) __enable_irq()

void Encoder_LeftInit(void)  { CRIT_ENTER; g_l_cnt = 0; g_l_dlt = 0; CRIT_EXIT; }
void Encoder_RightInit(void) { CRIT_ENTER; g_r_cnt = 0; g_r_dlt = 0; CRIT_EXIT; }

void Encoder_HWInit(void)
{
    /* 四路输入，上拉并开启迟滞 */
    DL_GPIO_initDigitalInputFeatures(L_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(L_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(R_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(R_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 输入滤波，抑制电机干扰 */
    DL_GPIO_setLowerPinsInputFilter(GPIOA,
        DL_GPIO_PIN_15_INPUT_FILTER_8_CYCLES);
    DL_GPIO_setUpperPinsInputFilter(GPIOA,
        DL_GPIO_PIN_16_INPUT_FILTER_8_CYCLES |
        DL_GPIO_PIN_17_INPUT_FILTER_8_CYCLES |
        DL_GPIO_PIN_24_INPUT_FILTER_8_CYCLES);

    /* PA15为低组，PA16 PA17 PA24为高组 */
    DL_GPIO_setLowerPinsPolarity(GPIOA,
        DL_GPIO_PIN_15_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOA,
        DL_GPIO_PIN_16_EDGE_RISE_FALL |
        DL_GPIO_PIN_17_EDGE_RISE_FALL |
        DL_GPIO_PIN_24_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(GPIOA,
        L_A_PIN | L_B_PIN | R_A_PIN | R_B_PIN);
    /* 仅A相开中断，B相只读电平 */
    DL_GPIO_enableInterrupt(GPIOA,
        L_A_PIN | R_A_PIN);

    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

/* 编码器中断，A相触发 */
void GROUP1_IRQHandler(void)
{
    uint32_t st = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        L_A_PIN | R_A_PIN);
    if (st == 0U) return;

    uint32_t d = GPIOA->DIN31_0;
    uint8_t la = (d & L_A_PIN) ? 1 : 0;
    uint8_t lb = (d & L_B_PIN) ? 1 : 0;
    uint8_t ra = (d & R_A_PIN) ? 1 : 0;
    uint8_t rb = (d & R_B_PIN) ? 1 : 0;

    /* 根据B相电平和方向计数 */
    if (st & L_A_PIN) { g_l_cnt += (la == lb) ? 1 : -1; g_l_dlt += (la == lb) ? 1 : -1; }
    /* 右轮向前为正，向后为负 */
    if (st & R_A_PIN) { g_r_cnt += (ra == rb) ? -1 : 1; g_r_dlt += (ra == rb) ? -1 : 1; }

    DL_GPIO_clearInterruptStatus(GPIOA, st);
}

int32_t Encoder_LeftGetCount(void)  { CRIT_ENTER; int32_t v = g_l_cnt; CRIT_EXIT; return v; }
int32_t Encoder_LeftReadDelta(void) { CRIT_ENTER; int32_t v = g_l_dlt; g_l_dlt = 0; CRIT_EXIT; return v; }
void    Encoder_LeftClear(void)     { CRIT_ENTER; g_l_cnt = 0; g_l_dlt = 0; CRIT_EXIT; }
int32_t Encoder_RightGetCount(void) { CRIT_ENTER; int32_t v = g_r_cnt; CRIT_EXIT; return v; }
int32_t Encoder_RightReadDelta(void){ CRIT_ENTER; int32_t v = g_r_dlt; g_r_dlt = 0; CRIT_EXIT; return v; }
void    Encoder_RightClear(void)    { CRIT_ENTER; g_r_cnt = 0; g_r_dlt = 0; CRIT_EXIT; }

void Encoder_GetCounts(int32_t *left, int32_t *right)
{
    CRIT_ENTER;
    if (left != 0) *left = g_l_cnt;
    if (right != 0) *right = g_r_cnt;
    CRIT_EXIT;
}
