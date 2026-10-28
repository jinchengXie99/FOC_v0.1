#include "ctrl_advanced.h"
#include "mtr_config.h"
#include "hw_dsp.h"
#include <math.h>

/* ============================================================
 * MTPA (最大转矩电流比) 计算
 * ============================================================ */
float CTRL_Calc_MTPA(float iq_target) {
    float l_diff = MOTOR_LQ - MOTOR_LD;
    
    // 表贴式电机 (Ld ≈ Lq) 直接返回 0
    if (fabsf(l_diff) < 1e-7f || fabsf(iq_target) < 0.1f) {
        return 0.0f;
    }
    
    // 内置式电机 MTPA 公式
    float lambda = MOTOR_FLUX_LINKAGE;
    float numerator = lambda - HW_DSP_Sqrt(lambda * lambda + 8.0f * l_diff * l_diff * iq_target * iq_target);
    float denominator = 4.0f * l_diff;
    
    return numerator / denominator; // 算出来是负数
}

/* ============================================================
 * 弱磁 (Field Weakening) 计算
 * ============================================================ */
float CTRL_Calc_FieldWeakening(float vd, float vq, float vbus, float iq_meas, float iq_ref, float current_fw_ref, float dt) {
    // 1. 计算电压利用率
    float v_mag = HW_DSP_Sqrt(vd * vd + vq * vq);
    float v_limit = vbus * 0.57735f; 
    float duty_ratio = (v_limit > 10.0f) ? (v_mag / v_limit) : 0.0f;
    
    float fw_target = 0.0f;
    
    // 2. 超过阈值触发弱磁
    if (duty_ratio > FW_VOLTAGE_THRESHOLD) {
        float ratio = (duty_ratio - FW_VOLTAGE_THRESHOLD) / (1.0f - FW_VOLTAGE_THRESHOLD);
        if (ratio > 1.0f) ratio = 1.0f;
        
        fw_target = -ratio * FW_MAX_CURRENT;
        
        // 3. 防失控回退 (Backoff)：如果 Iq 远超目标，说明失步了，弱磁必须退出来
        float i_err = (iq_meas - iq_ref) / FW_MAX_CURRENT;
        if (i_err > 0.0f) {
            fw_target += i_err * FW_MAX_CURRENT;
            if (fw_target > 0.0f) fw_target = 0.0f; 
        }
    }
    
    // 4. 平滑斜坡 (防止弱磁电流突变引起电压震荡)
    float result = current_fw_ref;
    if (fw_target < result) result -= FW_RAMP_STEP;
    else if (fw_target > result) result += FW_RAMP_STEP;
    
    // 再次限幅
    if (result > 0.0f) result = 0.0f;
    if (result < -FW_MAX_CURRENT) result = -FW_MAX_CURRENT;
    
    return result;
}
