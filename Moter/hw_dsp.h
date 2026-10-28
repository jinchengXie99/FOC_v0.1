#ifndef HW_DSP_H
#define HW_DSP_H
#include <stdint.h>
#include <math.h>

void cordic_sincos(float angle, float *p_sin, float *p_cos);

/* ============================================================
 * G4 CORDIC 硬件加速接口
 * ============================================================ */
void HW_DSP_Init(void);

/* 快速三角函数 (极速，替代 math.h 的 sinf/cosf) */
static inline void HW_DSP_SinCos(float angle, float *p_sin, float *p_cos) {
    /* 需填入 CORDIC 寄存器的读写逻辑，此处仅保留接口定义 */
    extern void cordic_sincos(float angle, float *s, float *c);
    cordic_sincos(angle, p_sin, p_cos);
}

static inline float HW_DSP_Sqrt(float x) {
    return sqrtf(x);
}

#endif
