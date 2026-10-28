#include "math_filter.h"

void Math_Filter_Init(Math_Filter_t *filt, float alpha, float init_val) {
    filt->alpha = alpha;
    filt->out = init_val;
}

/* 极速一阶低通：y(n) = (1-a)*y(n-1) + a*x(n) */
float Math_Filter_Update(Math_Filter_t *filt, float in) {
    filt->out = filt->out * (1.0f - filt->alpha) + in * filt->alpha;
    return filt->out;
}