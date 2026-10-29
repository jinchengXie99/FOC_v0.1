#include "ctrl_pi.h"
#include <stddef.h>

/* ============================================================
 * 速度环 PI 实现
 * ============================================================ */
void Speed_PI_Init(Speed_PI_t *pi, float kp, float ki, float out_max, float out_min) {
    if (pi == NULL) return;
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_max = out_max;
    pi->out_min = out_min;
    pi->i_max = out_max;  // 积分限幅通常与输出限幅一致
    pi->i_min = out_min;
}

void Speed_PI_Reset(Speed_PI_t *pi) {
    if (pi == NULL) return;
    pi->integral = 0.0f;
}

float Speed_PI_Calc(Speed_PI_t *pi, float target, float actual, float dt) {
    float error = target - actual;
    
    // 计算比例项
    float p_out = pi->kp * error;
    
    // 积分累加并限幅
    pi->integral += pi->ki * error * dt;
    if (pi->integral > pi->i_max) pi->integral = pi->i_max;
    if (pi->integral < pi->i_min) pi->integral = pi->i_min;
    
    // 综合输出并限幅
    float out = p_out + pi->integral;
    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;
    
    // 【可选优化】动态抗积分饱和：如果输出饱和且误差还在向同一方向推，停止积分
    if (out >= pi->out_max && error > 0) pi->integral -= pi->ki * error * dt;
    if (out <= pi->out_min && error < 0) pi->integral -= pi->ki * error * dt;

    return out;
}

/* ============================================================
 * 电流环 PI 实现
 * ============================================================ */
void Current_PI_Init(Current_PI_t *pi, float kp, float ki, float out_max) {
    if (pi == NULL) return;
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_max = out_max;
    pi->out_min = -out_max;
}

void Current_PI_Reset(Current_PI_t *pi) {
    if (pi == NULL) return;
    pi->integral = 0.0f;
}

float Current_PI_Calc(Current_PI_t *pi, float target, float actual, float dt) {
    float error = target - actual;
    
    float p_out = pi->kp * error;
    
    pi->integral += pi->ki * error * dt;
    if (pi->integral > pi->out_max) pi->integral = pi->out_max;
    if (pi->integral < pi->out_min) pi->integral = pi->out_min;
    
    float out = p_out + pi->integral;
    
    // 电流环输出限幅
    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;

    return out;
}