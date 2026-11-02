#include "math_filter.h"

/** @brief 设置一阶低通滤波器系数并指定初始输出。 */
void Math_Filter_Init(Math_Filter_t *filt, float alpha, float init_val) {
    filt->alpha = alpha;
    filt->out = init_val;
}

/**
 * @brief 对输入执行一阶 IIR 低通滤波。
 * @param in 当前采样值，量纲由调用方决定
 * @return 更新后的滤波输出，量纲与输入相同
 *
 * alpha 越小，输出越平滑但响应越慢；调用方应保证 alpha 在 [0,1]。
 */
float Math_Filter_Update(Math_Filter_t *filt, float in) {
    filt->out = filt->out * (1.0f - filt->alpha) + in * filt->alpha;
    return filt->out;
}
