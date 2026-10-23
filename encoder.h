/* 文件 encoder.h，用途 编码器接口 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_PPR                 500L
#define ENCODER_GEAR_RATIO          30L
#define ENCODER_QUADRATURE_FACTOR   2L  /* x2解码，仅A相双边沿 */
/* 实测30000计数每圈 */
#define ENCODER_COUNTS_PER_REV      30000L

void    Encoder_HWInit(void);
void    Encoder_LeftInit(void);
int32_t Encoder_LeftGetCount(void);
int32_t Encoder_LeftReadDelta(void);
void    Encoder_LeftClear(void);
void    Encoder_RightInit(void);
int32_t Encoder_RightGetCount(void);
int32_t Encoder_RightReadDelta(void);
void    Encoder_RightClear(void);
void    Encoder_GetCounts(int32_t *left, int32_t *right);

#endif
