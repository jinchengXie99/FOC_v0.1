#ifndef CTRL_OBSERVER_H
#define CTRL_OBSERVER_H
#include <stdbool.h>

/* ============================================================
 * VESC 非线性磁链观测器句柄
 * ============================================================ */
typedef struct {
    volatile float obs_x1;       // 内部磁链 alpha
    volatile float obs_x2;       // 内部磁链 beta
    volatile float pll_integral; // PLL 积分器
    volatile float theta_est;    // 估计电角度
    volatile float omega_est;    // 估计电角速度 (rad/s)
    volatile float speed_rpm;    // 估计机械转速 (RPM)
} Ctrl_Observer_t;

void Ctrl_Observer_Init(Ctrl_Observer_t *obs);

/* 在 20kHz 中断中调用 */
void Ctrl_Observer_Update(Ctrl_Observer_t *obs, float v_alpha, float v_beta, float i_alpha, float i_beta, float dt);

/* 强拖切入闭环瞬间，强行对齐状态 */
void Ctrl_Observer_ForceAlign(Ctrl_Observer_t *obs, float theta_if, float freq_e, float i_alpha, float i_beta);

#endif
