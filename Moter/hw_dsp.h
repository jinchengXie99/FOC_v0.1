#ifndef HW_DSP_H
#define HW_DSP_H
#include <stdint.h>
#include <math.h>

void cordic_sincos(float angle, float *p_sin, float *p_cos);

/* 初始化 STM32G4 CORDIC 硬件；应在首次三角函数计算前调用。 */
void HW_DSP_Init(void);

/* CORDIC 正余弦接口；angle 单位 rad，输出结果范围约为 [-1, 1]。 */
static inline void HW_DSP_SinCos(float angle, float *p_sin, float *p_cos) {
    /* 通过底层函数完成 CORDIC 输入编码、运算等待及结果解码。 */
    extern void cordic_sincos(float angle, float *s, float *c);
    cordic_sincos(angle, p_sin, p_cos);
}

static inline float HW_DSP_Sqrt(float x) {
    /* 当前封装使用标准库平方根；保留统一数学接口便于调用方复用。 */
    return sqrtf(x);
}

#endif
