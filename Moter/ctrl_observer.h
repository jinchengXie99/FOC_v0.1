#ifndef CTRL_OBSERVER_H
#define CTRL_OBSERVER_H
#include <stdbool.h>

/* 非线性磁链观测器与 PLL 的运行状态。磁链单位 Wb，角度单位 rad。 */
typedef struct {
    volatile float obs_x1;       /* 观测器内部 alpha 磁链状态 (Wb)。 */
    volatile float obs_x2;       /* 观测器内部 beta 磁链状态 (Wb)。 */
    volatile float pll_integral; /* PLL 积分状态，作为电角速度的一部分 (rad/s)。 */
    volatile float theta_est;    /* 转子电角度估计 (rad)。 */
    volatile float omega_est;    /* 转子电角速度估计 (rad/s)。 */
    volatile float speed_rpm;    /* 机械转速估计 (RPM)，符号表示转向。 */
    volatile float speed_rpm_filt; /* 供低频速度环/遥测使用的滤波转速 (RPM)。 */
} Ctrl_Observer_t;

/* 清零观测器磁链、PLL、角度及速度状态。 */
void Ctrl_Observer_Init(Ctrl_Observer_t *obs);

/* 在电流采样中断中更新观测器；电压为上一周期实际输出的 alpha-beta 值。 */
void Ctrl_Observer_Update(Ctrl_Observer_t *obs, float v_alpha, float v_beta, float i_alpha, float i_beta, float dt);

/* IF 强拖结束、闭环接管时将观测器状态与强拖角度/速度对齐。 */
void Ctrl_Observer_ForceAlign(Ctrl_Observer_t *obs, float theta_if, float freq_e, float i_alpha, float i_beta);

#endif
