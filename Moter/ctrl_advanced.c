#include "ctrl_advanced.h"
#include "mtr_config.h"
#include "hw_dsp.h"
#include <math.h>

/**
 * @brief 计算最大转矩电流比 (MTPA) 的 d 轴电流参考。
 * @param iq_target q 轴目标电流 (A)
 * @return d 轴电流参考 (A)，通常为非正值
 *
 * 表贴式电机的 d/q 电感近似相等时，磁阻转矩很小，直接返回零；
 * 凸极电机则使用电感差和永磁体磁链计算 MTPA 参考。
 */
float CTRL_Calc_MTPA(float iq_target) {
    float l_diff = MOTOR_LQ - MOTOR_LD;
    
    /* 表贴式电机或极小电流下不需要 MTPA 补偿。 */
    if (fabsf(l_diff) < 1e-7f || fabsf(iq_target) < 0.1f) {
        return 0.0f;
    }
    
    /* 凸极电机的解析 MTPA 解；lambda 单位 Wb，电流单位 A。 */
    float lambda = MOTOR_FLUX_LINKAGE;
    float numerator = lambda - HW_DSP_Sqrt(lambda * lambda + 8.0f * l_diff * l_diff * iq_target * iq_target);
    float denominator = 4.0f * l_diff;
    
    return numerator / denominator;
}

/**
 * @brief 按电压利用率产生平滑的弱磁 d 轴电流参考。
 * @param vd d 轴电压分量 (V)
 * @param vq q 轴电压分量 (V)
 * @param vbus 直流母线电压 (V)
 * @param iq_meas q 轴实测电流 (A)
 * @param iq_ref q 轴目标电流 (A)
 * @param current_fw_ref 上一周期弱磁电流参考 (A)
 * @param dt 调用周期 (s)，当前实现使用固定每次调用步长
 * @return 限制在 [-FW_MAX_CURRENT, 0] A 的弱磁参考
 */
float CTRL_Calc_FieldWeakening(float vd, float vq, float vbus, float iq_meas, float iq_ref, float current_fw_ref, float dt) {
    /* 计算 dq 电压模长及 SVPWM 线性区电压上限的利用率。 */
    float v_mag = HW_DSP_Sqrt(vd * vd + vq * vq);
    float v_limit = vbus * 0.57735f; 
    float duty_ratio = (v_limit > 10.0f) ? (v_mag / v_limit) : 0.0f;
    
    float fw_target = 0.0f;
    
    /* 电压利用率超过阈值后，逐步增加负 d 轴电流以扩展高速范围。 */
    if (duty_ratio > FW_VOLTAGE_THRESHOLD) {
        float ratio = (duty_ratio - FW_VOLTAGE_THRESHOLD) / (1.0f - FW_VOLTAGE_THRESHOLD);
        if (ratio > 1.0f) ratio = 1.0f;
        
        fw_target = -ratio * FW_MAX_CURRENT;
        
        /* Iq 实测超出目标时回退弱磁，避免异常电流下继续加深弱磁。 */
        float i_err = (iq_meas - iq_ref) / FW_MAX_CURRENT;
        if (i_err > 0.0f) {
            fw_target += i_err * FW_MAX_CURRENT;
            if (fw_target > 0.0f) fw_target = 0.0f; 
        }
    }
    
    /* 限制每次调用的参考变化量，减少弱磁指令突变。 */
    float result = current_fw_ref;
    if (fw_target < result) result -= FW_RAMP_STEP;
    else if (fw_target > result) result += FW_RAMP_STEP;
    
    /* 弱磁参考只能为零或负值，且不超过配置的最大幅值。 */
    if (result > 0.0f) result = 0.0f;
    if (result < -FW_MAX_CURRENT) result = -FW_MAX_CURRENT;
    
    return result;
}
