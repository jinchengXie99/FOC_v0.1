#ifndef MATH_FILTER_H
#define MATH_FILTER_H

typedef struct {
    float alpha;    /* 滤波系数 (0.0 ~ 1.0)，越小越平滑但滞后越大 */
    float out;      /* 当前滤波输出值 */
} Math_Filter_t;

void Math_Filter_Init(Math_Filter_t *filt, float alpha, float init_val);
float Math_Filter_Update(Math_Filter_t *filt, float in);

#endif
