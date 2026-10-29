#ifndef CTRL_PI_H
#define CTRL_PI_H

#include <stdint.h>

/* ============================================================
 * 速度环 PI 控制器 (输出物理电流 A)
 * ============================================================ */
typedef struct {
    float kp;
    float ki;
    float integral;       // 积分累加器
    float out_max;        // 正向输出限幅 (例如 18.0A)
    float out_min;        // 反向输出限幅 (例如 -10.0A)
    float i_max;          // 积分抗饱和限幅 (通常等于 out_max)
    float i_min;          // 积分抗饱和限幅 (通常等于 out_min)
} Speed_PI_t;

void Speed_PI_Init(Speed_PI_t *pi, float kp, float ki, float out_max, float out_min);
void Speed_PI_Reset(Speed_PI_t *pi);
float Speed_PI_Calc(Speed_PI_t *pi, float target, float actual, float dt);

/* ============================================================
 * 电流环 PI 控制器 (输出物理电压 V)
 * ============================================================ */
typedef struct {
    float kp;
    float ki;
    float integral;       // 积分累加器
    float out_max;        // 输出电压限幅 (通常与母线电压相关)
    float out_min;
} Current_PI_t;

void Current_PI_Init(Current_PI_t *pi, float kp, float ki, float out_max);
void Current_PI_Reset(Current_PI_t *pi);
float Current_PI_Calc(Current_PI_t *pi, float target, float actual, float dt);

#endif
