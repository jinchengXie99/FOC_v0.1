#ifndef MATH_PID_H
#define MATH_PID_H
#include <stdint.h>

/* ============================================================
 * 标准 PID 控制器句柄
 * ============================================================ */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;     // 积分累加器
    float prev_error;   // 上一次误差(用于微分)
    float out_max;      // 输出限幅(归一化上限)
    float out_min;      // 输出限幅(归一化下限)
    float integral_max; // 积分抗饱和限幅
    float integral_min;
} Math_PID_t;

void Math_PID_Init(Math_PID_t *pid, float kp, float ki, float kd, float i_max, float out_max);
void Math_PID_Reset(Math_PID_t *pid);

/* 输出单位与调用者的参考量/控制量一致，并由 PID 配置限幅。 */
float Math_PID_Calc(Math_PID_t *pid, float target, float actual, float dt);

#endif
