#include "hw_dsp.h"
#include "hw_cordic.h"

/* 初始化 DSP 数学加速接口所依赖的 CORDIC 外设，启动时调用一次。 */
void HW_DSP_Init(void) {
    HW_CORDIC_Init();
}

/* 提供通用 DSP 层入口，实际角度规约与寄存器操作由 hw_cordic.c 执行。 */
void cordic_sincos(float angle, float *p_sin, float *p_cos) {
    HW_CORDIC_SinCos(angle, p_sin, p_cos);
}
