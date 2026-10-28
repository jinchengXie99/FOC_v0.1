#include "math_pid.h"
#include <stddef.h>

/* ============================================================
 * 初始化 PID 参数
 * ============================================================ */
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

/* ============================================================
 * 清零 PID 状态 (用于状态切换、故障复位)
 * ============================================================ */
void Math_PID_Reset(Math_PID_t *pid) {
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/* ============================================================
 * PID 极速计算
 * ============================================================ */
float Math_PID_Calc(Math_PID_t *pid, float target, float actual, float dt) {
    float error = target - actual;
    
    // 1. 比例项
    float p_term = pid->kp * error;

    // 2. 积分项 (带抗饱和)
    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    
    // 3. 微分项 (可选，电机控制中高频噪声大，通常弱化或不用 D 项)
    float d_term = 0.0f;
    if (pid->kd > 0.001f) {
        d_term = pid->kd * (error - pid->prev_error) / dt;
        pid->prev_error = error;
    }

    // 4. 综合输出
    float output = p_term + pid->integral + d_term;

    // 5. 输出总限幅
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}