#include "safe_fault.h"
#include "hw_pwm.h"
#include "app_main.h" // 引用全局电机状态
#include "drv_config.h"
#include "hw_adc.h"
#include <math.h>

static volatile uint8_t soft_overcurrent_count;

uint8_t SAFE_CheckCurrentFast(float ia, float ib, float ic) {
    float i_max = fabsf(ia);
    uint8_t fault = FAULT_NONE;

    if (fabsf(ib) > i_max) i_max = fabsf(ib);
    if (fabsf(ic) > i_max) i_max = fabsf(ic);

    if (i_max > DRV_OC_HARD_LIMIT) {
        fault = FAULT_OC_HARD;
    } else if (i_max > DRV_OC_SOFT_LIMIT) {
        if (soft_overcurrent_count < DRV_OC_DEBOUNCE_CNT) {
            soft_overcurrent_count++;
        }
        if (soft_overcurrent_count >= DRV_OC_DEBOUNCE_CNT) {
            fault = FAULT_OC_SOFT;
        }
    } else {
        soft_overcurrent_count = 0U;
    }

    if (fault != FAULT_NONE) {
        g_motor.fault_code = fault;
        g_motor.state = STATE_FAULT;
        HW_PWM_Disable();
    }
    return fault;
}

/* 母线电压保护由 1kHz 调度调用；过流另在 ADC 高频路径检测。 */
uint8_t SAFE_Check(float vbus, float ia, float ib, float ic) {
    // 1. 电流硬限值检查 (1kHz 备用路径)
    float i_max = fabsf(ia);
    if (fabsf(ib) > i_max) i_max = fabsf(ib);
    if (fabsf(ic) > i_max) i_max = fabsf(ic);

    if (i_max > DRV_OC_HARD_LIMIT) {
        return FAULT_OC_HARD; // 硬阈值过流，25A
    }

    // 2. 软件级过压/欠压保护
    if (vbus > DRV_OVP_THRESHOLD) {
        return FAULT_OVP; // 过压
    }
    if (vbus < DRV_UVP_THRESHOLD && g_motor.state != STATE_IDLE) {
        return FAULT_UVP; // 欠压
    }

    return FAULT_NONE; // 正常
}

/* ============================================================
 * 故障锁存处理 (在主循环中轮询恢复)
 * ============================================================ */
void SAFE_HandleFault(void) {
    if (g_motor.state == STATE_FAULT) {
        // 强制关断 SD 引脚
        HW_PWM_Disable();
        
        // 亮红灯
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // 红亮
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);   // 绿灭

        // 只有当电流归零，且按下复位键时，才解除锁存
        uint8_t key_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET);
        float vbus = HW_ADC_GetVbus();
        if (key_pressed &&
            g_motor.target_rpm <= 0.0f &&
            vbus > DRV_UVP_RECOVER &&
            vbus < DRV_OVP_RECOVER &&
            fabsf(g_motor.i_a) < 0.5f &&
            fabsf(g_motor.i_b) < 0.5f &&
            fabsf(g_motor.i_c) < 0.5f) {
            // 复位状态
            g_motor.fault_code = 0;
            g_motor.state = STATE_IDLE;
            Ctrl_Observer_Init(&g_motor.obs);
							
						Speed_PI_Reset(&g_motor.pi_speed);
						Current_PI_Reset(&g_motor.pi_id);
						Current_PI_Reset(&g_motor.pi_iq);
 
							
            soft_overcurrent_count = 0U;
            g_motor.iq_ref = 0.0f;
            g_motor.id_mtpa_ref = 0.0f;
            g_motor.id_fw_ref = 0.0f;
            
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);   // 红灭
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // 绿亮
        }
    }
}
