#include "ctrl_current_loop.h"
#include "ctrl_pi.h"
#include "math_svpwm.h"
#include "hw_pwm.h"
#include "hw_dsp.h"
#include "drv_config.h"
#include "mtr_config.h"
#include "safe_fault.h"
#include <math.h>

/* ============================================================
 * 20kHz 电流环闭环控制 (在 ADC 中断中调用)
 * ============================================================ */
void Ctrl_CurrentLoop_Execute(Motor_Ctrl_t *motor, float vbus, float dt) {
    if (vbus < DRV_UVP_THRESHOLD || isnan(vbus) || isinf(vbus)) {
        motor->fault_code = FAULT_UVP;
        motor->state = STATE_FAULT;
        HW_PWM_Disable();
        return;
    }

    // 1. 计算 Park 变换需要的三角函数
    float sin_e, cos_e;
    cordic_sincos(motor->obs.theta_est, &sin_e, &cos_e);

    // 2. Park 变换 (由 alpha-beta 到 d-q)
    float i_d_meas =  motor->i_alpha * cos_e + motor->i_beta * sin_e;
    float i_q_meas = -motor->i_alpha * sin_e + motor->i_beta * cos_e;

    motor->i_d = i_d_meas;
    motor->i_q = i_q_meas;

    // 3. 目标电流融合 (MTPA 与 弱磁叠加)
    float id_target = motor->id_mtpa_ref; 
    if (fabsf(motor->id_fw_ref) > fabsf(id_target)) {
        id_target = motor->id_fw_ref; 
    }

    // 4. 电流环误差计算
    float err_d = id_target - i_d_meas;
    float err_q = motor->iq_ref - i_q_meas;

    // 5. 计算 PI 输出 (此时已包含积分和限幅)
    float v_d_cmd = Current_PI_Calc(&motor->pi_id, id_target, i_d_meas, dt);
    float v_q_cmd = Current_PI_Calc(&motor->pi_iq, motor->iq_ref, i_q_meas, dt);

    #if USE_DECOUPLING
        // 解耦补偿：Vd -= w * L * Iq, Vq += w * L * Id + w * Flux
        v_d_cmd -= motor->obs.omega_est * MOTOR_LS * i_q_meas;
        v_q_cmd += motor->obs.omega_est * MOTOR_LS * i_d_meas;
        v_q_cmd += motor->obs.omega_est * MOTOR_FLUX_LINKAGE; 
    #endif

    // 6. 电压圆限幅
    float v_limit = vbus * 0.57735f; // 1/sqrt(3)
    float v_mag = HW_DSP_Sqrt(v_d_cmd * v_d_cmd + v_q_cmd * v_q_cmd);
    
    if (v_mag > v_limit) {
        float scale = v_limit / v_mag;
        v_d_cmd *= scale;
        v_q_cmd *= scale;
        
        // 如果电压达到极限，直接冻结积分器，防止 Windup
        // 注意：这里的 integral 是微调，不要让函数内部积分导致输出饱和
        // 由于我们在函数内部做了限幅，为了防止饱和，我们在这里将积分往回拉
        motor->pi_id.integral = v_d_cmd - (motor->pi_id.kp * err_d);
        motor->pi_iq.integral = v_q_cmd - (motor->pi_iq.kp * err_q);
        
        // 防止积分器越界
        if (motor->pi_id.integral > v_limit) motor->pi_id.integral = v_limit;
        if (motor->pi_id.integral < -v_limit) motor->pi_id.integral = -v_limit;
        if (motor->pi_iq.integral > v_limit) motor->pi_iq.integral = v_limit;
        if (motor->pi_iq.integral < -v_limit) motor->pi_iq.integral = -v_limit;
    }

    motor->v_d = v_d_cmd;
    motor->v_q = v_q_cmd;
    motor->v_alpha = v_d_cmd * cos_e - v_q_cmd * sin_e;
    motor->v_beta = v_d_cmd * sin_e + v_q_cmd * cos_e;

    // 7. 反 Park & SVPWM 计算
    float duty_a, duty_b, duty_c;
    Math_SVPWM_Gen(v_d_cmd, v_q_cmd, motor->obs.theta_est, vbus, &duty_a, &duty_b, &duty_c);

    // 8. 寄存器级更新占空比
    HW_PWM_SetDuty(duty_a, duty_b, duty_c);
}
