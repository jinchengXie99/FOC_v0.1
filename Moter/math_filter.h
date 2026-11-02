#ifndef MATH_FILTER_H
#define MATH_FILTER_H

/* 一阶低通滤波器状态；输出量纲与输入相同。 */
typedef struct {
    float alpha;    /* 新输入权重，建议范围 [0,1]。 */
    float out;      /* 上一次更新后的输出值。 */
} Math_Filter_t;

/* 配置滤波权重 alpha，并将 out 初始化为 init_val。 */
void Math_Filter_Init(Math_Filter_t *filt, float alpha, float init_val);
/* 加入一个新采样并返回滤波后的值。 */
float Math_Filter_Update(Math_Filter_t *filt, float in);

#endif
