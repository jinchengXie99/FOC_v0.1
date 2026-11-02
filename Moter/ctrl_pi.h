#ifndef CTRL_PI_H
#define CTRL_PI_H

#include <stdint.h>

/* 速度环 PI：RPM 误差输入，q 轴电流参考 (A) 输出。 */
typedef struct {
    float kp;
    float ki;
    float integral;       /* 积分器当前输出状态 (A)。 */
    float out_max;        /* 输出电流上限 (A)。 */
    float out_min;        /* 输出电流下限 (A)。 */
    float i_max;          /* 积分状态上限 (A)。 */
    float i_min;          /* 积分状态下限 (A)。 */
} Speed_PI_t;

/* 初始化速度 PI；kp 单位 A/RPM，ki 单位 A/(RPM*s)。 */
void Speed_PI_Init(Speed_PI_t *pi, float kp, float ki, float out_max, float out_min);
/* 清除积分状态，不修改比例/限幅参数。 */
void Speed_PI_Reset(Speed_PI_t *pi);
/* target/actual 为 RPM，dt 为秒，返回 q 轴电流参考 (A)。 */
float Speed_PI_Calc(Speed_PI_t *pi, float target, float actual, float dt);

/* 电流环 PI：电流误差输入，dq 电压指令 (V) 输出。 */
typedef struct {
    float kp;
    float ki;
    float integral;       /* 积分器当前输出状态 (V)。 */
    float out_max;        /* 正向输出电压限幅 (V)。 */
    float out_min;
} Current_PI_t;

/* 初始化电流 PI，并将输出范围设置为 [-out_max, +out_max]。 */
void Current_PI_Init(Current_PI_t *pi, float kp, float ki, float out_max);
/* 清除积分状态，不修改比例/限幅参数。 */
void Current_PI_Reset(Current_PI_t *pi);
/* target/actual 为 A，dt 为秒，返回电压指令 (V)。 */
float Current_PI_Calc(Current_PI_t *pi, float target, float actual, float dt);

#endif
