#ifndef MATH_PID_H
#define MATH_PID_H
#include <stdint.h>

/* 通用位置式 PID 状态；各系数和信号量纲由调用模块定义。 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;     /* 积分状态。 */
    float prev_error;   /* 上次误差，供微分项计算。 */
    float out_max;      /* 正向输出上限。 */
    float out_min;      /* 负向输出下限。 */
    float integral_max; /* 积分状态上限。 */
    float integral_min;
} Math_PID_t;

/* 初始化 PID 系数；i_max 和 out_max 分别设置积分及输出对称限幅。 */
void Math_PID_Init(Math_PID_t *pid, float kp, float ki, float kd, float i_max, float out_max);
/* 清除积分器和误差历史，不更改配置参数。 */
void Math_PID_Reset(Math_PID_t *pid);

/* 计算一次 PID；dt 以秒计，返回值量纲由调用者定义。 */
float Math_PID_Calc(Math_PID_t *pid, float target, float actual, float dt);

#endif
