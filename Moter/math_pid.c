#include "math_pid.h"
#include <stddef.h>

/** @brief 配置 PID 系数、积分/输出对称限幅并清零动态状态。 */
void Math_PID_Init(Math_PID_t *pid, float kp, float ki, float kd, float i_max, float out_max) {
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = i_max; // 积分抗饱和限幅
    pid->integral_min = -i_max;
    pid->out_max = out_max;
    pid->out_min = -out_max;
}

/** @brief 清除积分和上次误差，保留控制器参数及限幅设置。 */
void Math_PID_Reset(Math_PID_t *pid) {
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/**
 * @brief 计算一次位置式 PID 输出并进行积分、总输出限幅。
 * @param target 目标值
 * @param actual 实测值
 * @param dt 采样周期 (s)，启用 D 项时必须为正值
 * @return 输出值；物理单位取决于调用者和 PID 系数配置
 */
float Math_PID_Calc(Math_PID_t *pid, float target, float actual, float dt) {
    float error = target - actual;
    
    /* 比例项由当前误差直接产生。 */
    float p_term = pid->kp * error;

    /* 积分误差并将内部积分状态限制在配置范围内。 */
    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    
    /* 微分项按误差变化率计算；kd 较小时视为关闭，避免放大高频噪声。 */
    float d_term = 0.0f;
    if (pid->kd > 0.001f) {
        d_term = pid->kd * (error - pid->prev_error) / dt;
        pid->prev_error = error;
    }

    /* P、I、D 三项相加形成未限幅控制输出。 */
    float output = p_term + pid->integral + d_term;

    /* 将最终输出限制在配置范围内。 */
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}
