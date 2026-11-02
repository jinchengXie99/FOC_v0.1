#include "ctrl_observer.h"
#include "mtr_config.h"
#include "hw_dsp.h"
#include <math.h>

/** @brief 将磁链观测器、PLL、角度和转速状态清零。 */
void Ctrl_Observer_Init(Ctrl_Observer_t *obs) {
    obs->obs_x1 = 0.0f;
    obs->obs_x2 = 0.0f;
    obs->pll_integral = 0.0f;
    obs->theta_est = 0.0f;
    obs->omega_est = 0.0f;
    obs->speed_rpm = 0.0f;
	  obs->speed_rpm_filt  = 0.0f;
}

/**
 * @brief 更新非线性磁链观测器及其 PLL 速度/角度估计。
 *
 * 根据定子电压模型积分磁链状态，利用非线性误差项将估计磁链
 * 拉向永磁体磁链幅值，再由 PLL 跟踪磁链角度。
 *
 * @param obs 观测器状态
 * @param v_alpha alpha 轴电压输入 (V)
 * @param v_beta beta 轴电压输入 (V)
 * @param i_alpha alpha 轴电流输入 (A)
 * @param i_beta beta 轴电流输入 (A)
 * @param dt 采样周期 (s)，通常为 50 us
 */
void Ctrl_Observer_Update(Ctrl_Observer_t *obs, float v_alpha, float v_beta, float i_alpha, float i_beta, float dt) {
   
    /*
     * 使用平均电感模型计算磁链中的电感压降。若将来启用凸极电机
     * 的电流相关电感模型，应在此处替换 L_dynamic 的计算。
     */
		float L_dynamic = MOTOR_LS;
//		if (fabsf(i_d) > 0.1f || fabsf(i_q) > 0.1f) {
//				float ld_lq_diff = MOTOR_LQ - MOTOR_LD;
//				L_dynamic = MOTOR_LS - ld_lq_diff/2.0f 
//									+ ld_lq_diff * (i_q * i_q) / (i_d * i_d + i_q * i_q);
//		}
    /* 1. 计算定子电流对应的电感磁链分量。 */
    float L_ia = L_dynamic * i_alpha;
    float L_ib = L_dynamic * i_beta;
    
    /* 2. 计算观测磁链幅值平方与永磁体磁链平方之间的误差。 */
    float lambda_sq = MOTOR_FLUX_LINKAGE * MOTOR_FLUX_LINKAGE;
    float x1_ia = obs->obs_x1 - L_ia;
    float x2_ib = obs->obs_x2 - L_ib;
    float err = lambda_sq - (x1_ia * x1_ia + x2_ib * x2_ib);
    /* 仅保留向内的误差修正，避免估计幅值偏小时产生反向积分。 */
    if (err > 0.0f) {
        err = 0.0f;
    }
    /* 3. 按电机电压模型积分磁链，并按估计转速调整观测器增益。 */
		float gamma_adaptive = OBSERVER_GAIN;
		float speed_abs = fabsf(obs->omega_est);
		if (speed_abs < 200.0f) { 					// 低速区
				gamma_adaptive *= 0.5f; 				// 降低增益，压制噪声
		} else if (speed_abs > 1000.0f) { 	// 高速区
				gamma_adaptive *= 1.2f; 				// 提高增益，保证带宽
		}
    obs->obs_x1 += (v_alpha - MOTOR_RS * i_alpha + gamma_adaptive * err * x1_ia) * dt;
    obs->obs_x2 += (v_beta  - MOTOR_RS * i_beta  + gamma_adaptive * err * x2_ib) * dt;
    
    /* 4. 从总磁链中去除电感磁链，得到转子永磁磁链估计。 */
    float flux_alpha = obs->obs_x1 - L_ia;
    float flux_beta  = obs->obs_x2 - L_ib;

    /* 5. 将磁链矢量投影到估计角度的正交轴，形成 PLL 相位误差。 */
    float sin_e, cos_e;
    cordic_sincos(obs->theta_est, &sin_e, &cos_e);
    float phase_err = -flux_alpha * sin_e + flux_beta * cos_e;
    
    /* 按磁链幅值归一化，使相位误差尺度近似为无量纲。 */
    phase_err = phase_err / MOTOR_FLUX_LINKAGE;

    /* 6. PI 型 PLL 输出电角速度 omega_est (rad/s)。 */
    obs->pll_integral += phase_err * PLL_KI * dt;
    
    /* 限制 PLL 积分项，避免异常相位误差造成积分器持续累积。 */
    if (obs->pll_integral > 3000.0f)  obs->pll_integral = 3000.0f;
    if (obs->pll_integral < -3000.0f) obs->pll_integral = -3000.0f;
    
    obs->omega_est = obs->pll_integral + phase_err * PLL_KP;

    /* 对速度估计做有限性检查和最大电角速度限制。 */
    if (isnan(obs->omega_est) || isinf(obs->omega_est)) obs->omega_est = 0.0f;
    if (obs->omega_est > 6000.0f)  obs->omega_est = 6000.0f;
    if (obs->omega_est < -6000.0f) obs->omega_est = -6000.0f;
    
    /* 电角速度换算为机械转速，除以极对数并转换 rad/s 到 RPM。 */
    obs->speed_rpm = (obs->omega_est / MOTOR_POLE_PAIRS) * (60.0f / 6.28318531f);

    /* 7. 积分电角速度得到新角度，并回绕到 [0, 2*pi) 区间。 */
    obs->theta_est += obs->omega_est * dt;
    if (obs->theta_est >= 6.28318531f) obs->theta_est -= 6.28318531f;
    if (obs->theta_est < 0.0f)         obs->theta_est += 6.28318531f;
}

/**
 * @brief 强拖切换到闭环时，将观测器状态对齐至开环估计。
 * @param obs 观测器状态
 * @param theta_if 当前强拖电角度 (rad)
 * @param freq_e 当前强拖电频率 (Hz，符号表示转向)
 * @param i_alpha alpha 轴电流 (A)
 * @param i_beta beta 轴电流 (A)
 */
void Ctrl_Observer_ForceAlign(Ctrl_Observer_t *obs, float theta_if, float freq_e, float i_alpha, float i_beta) {
    float sin_e, cos_e;
    cordic_sincos(theta_if, &sin_e, &cos_e);

    /* 初始化磁链状态，使扣除电感磁链后的磁链角与强拖角度一致。 */
    obs->obs_x1 = MOTOR_FLUX_LINKAGE * cos_e + MOTOR_LS * i_alpha;
    obs->obs_x2 = MOTOR_FLUX_LINKAGE * sin_e + MOTOR_LS * i_beta;

    /* 用开环电频率预置 PLL 速度状态和有符号机械转速。 */
    obs->pll_integral = 6.28318531f * freq_e;
    obs->omega_est = obs->pll_integral;
    obs->speed_rpm = freq_e * 60.0f / MOTOR_POLE_PAIRS;
    obs->speed_rpm_filt = obs->speed_rpm;

    /* 切换瞬间的观测角和滤波转速均与开环状态对齐。 */
    obs->theta_est = theta_if;
}
