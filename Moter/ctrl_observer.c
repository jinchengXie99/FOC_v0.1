#include "ctrl_observer.h"
#include "mtr_config.h"
#include "hw_dsp.h"
#include <math.h>

/* ============================================================
 * 初始化观测器内部状态
 * ============================================================ */
void Ctrl_Observer_Init(Ctrl_Observer_t *obs) {
    obs->obs_x1 = 0.0f;
    obs->obs_x2 = 0.0f;
    obs->pll_integral = 0.0f;
    obs->theta_est = 0.0f;
    obs->omega_est = 0.0f;
    obs->speed_rpm = 0.0f;
	  obs->speed_rpm_filt  = 0.0f;
}

/* ============================================================
 * 20kHz 高频观测器更新 ( Ortega-Praly 算法)
 * ============================================================ */
void Ctrl_Observer_Update(Ctrl_Observer_t *obs, float v_alpha, float v_beta, float i_alpha, float i_beta,float i_d,float i_q, float dt) {
   
	// 电机若是内置式（Ld≠Lq），必须根据电流实时调整电感
		float L_dynamic = MOTOR_LS;
//		if (fabsf(i_d) > 0.1f || fabsf(i_q) > 0.1f) {
//				float ld_lq_diff = MOTOR_LQ - MOTOR_LD;
//				L_dynamic = MOTOR_LS - ld_lq_diff/2.0f 
//									+ ld_lq_diff * (i_q * i_q) / (i_d * i_d + i_q * i_q);
//		}
		// 1. 计算电感压降项
    float L_ia = L_dynamic * i_alpha;
    float L_ib = L_dynamic * i_beta;
    
    // 2. 计算磁链误差
    float lambda_sq = MOTOR_FLUX_LINKAGE * MOTOR_FLUX_LINKAGE;
    float x1_ia = obs->obs_x1 - L_ia;
    float x2_ib = obs->obs_x2 - L_ib;
    float err = lambda_sq - (x1_ia * x1_ia + x2_ib * x2_ib);
    //  单向离合器 保证收敛
    if (err > 0.0f) {
        err = 0.0f;
    }
    // 3. 状态变量积分 (加入修正项)
		float gamma_adaptive = OBSERVER_GAIN;
		float speed_abs = fabsf(obs->omega_est);
		if (speed_abs < 200.0f) { 					// 低速区
				gamma_adaptive *= 0.5f; 				// 降低增益，压制噪声
		} else if (speed_abs > 1000.0f) { 	// 高速区
				gamma_adaptive *= 1.2f; 				// 提高增益，保证带宽
		}
    obs->obs_x1 += (v_alpha - MOTOR_RS * i_alpha + gamma_adaptive * err * x1_ia) * dt;
    obs->obs_x2 += (v_beta  - MOTOR_RS * i_beta  + gamma_adaptive * err * x2_ib) * dt;
    
    // 4. 提取转子磁链
    float flux_alpha = obs->obs_x1 - L_ia;
    float flux_beta  = obs->obs_x2 - L_ib;

    // 5. PLL 鉴相器 (提取角度误差)
    float sin_e, cos_e;
    cordic_sincos(obs->theta_est, &sin_e, &cos_e);
    float phase_err = -flux_alpha * sin_e + flux_beta * cos_e;
    
    // 归一化角度误差
    phase_err = phase_err / MOTOR_FLUX_LINKAGE;

    // 6. PLL 积分与速度更新
    obs->pll_integral += phase_err * PLL_KI * dt;
    
    // 积分抗饱和 (防止失控)
    if (obs->pll_integral > 3000.0f)  obs->pll_integral = 3000.0f;
    if (obs->pll_integral < -3000.0f) obs->pll_integral = -3000.0f;
    
    obs->omega_est = obs->pll_integral + phase_err * PLL_KP;

    // 物理限幅与 NaN 保护
    if (isnan(obs->omega_est) || isinf(obs->omega_est)) obs->omega_est = 0.0f;
    if (obs->omega_est > 6000.0f)  obs->omega_est = 6000.0f;
    if (obs->omega_est < -6000.0f) obs->omega_est = -6000.0f;
    
    // 计算机械转速 (RPM)
    obs->speed_rpm = (obs->omega_est / MOTOR_POLE_PAIRS) * (60.0f / 6.28318531f);

    // 7. 角度积分更新
    obs->theta_est += obs->omega_est * dt;
    if (obs->theta_est >= 6.28318531f) obs->theta_est -= 6.28318531f;
    if (obs->theta_est < 0.0f)         obs->theta_est += 6.28318531f;
}

/* ============================================================
 * 强拖切入闭环瞬间的强行对齐 (无缝启动核心)
 * ============================================================ */
void Ctrl_Observer_ForceAlign(Ctrl_Observer_t *obs, float theta_if, float freq_e, float i_alpha, float i_beta) {
    float sin_e, cos_e;
    cordic_sincos(theta_if, &sin_e, &cos_e);

    // 将观测器的内部磁链状态直接拉到当前的强拖角度位置
    obs->obs_x1 = MOTOR_FLUX_LINKAGE * cos_e + MOTOR_LS * i_alpha;
    obs->obs_x2 = MOTOR_FLUX_LINKAGE * sin_e + MOTOR_LS * i_beta;

    // 强行将 PLL 积分器设为当前速度
    obs->pll_integral = 6.28318531f * freq_e;

    // 强行覆盖角度
    obs->theta_est = theta_if;
}
