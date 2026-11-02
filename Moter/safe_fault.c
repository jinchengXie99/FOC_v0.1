#include "safe_fault.h"
#include "hw_pwm.h"
#include "app_main.h" // 引用全局电机状态
#include "drv_config.h"
#include "hw_adc.h"
#include <math.h>

static volatile uint8_t soft_overcurrent_count;

/**
 * @brief 在高速采样路径检查三相电流，并立即处理过流故障。
 * @return 无故障返回 FAULT_NONE，否则返回硬件级或软件级过流码
 */
uint8_t SAFE_CheckCurrentFast(float ia, float ib, float ic) {
    /* 过流阈值作用于三相中绝对值最大的相电流。 */
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

/**
 * @brief 执行低频安全检查。
 * @param vbus 母线电压 (V)
 * @param ia A 相电流反馈 (A)
 * @param ib B 相电流反馈 (A)
 * @param ic C 相电流反馈 (A)
 * @return 检出的故障码；返回 FAULT_NONE 表示未触发阈值
 *
 * 此路径检查硬过流、带消抖的母线过压/欠压；软过流消抖在 ADC
 * 高频路径处理，以缩短保护响应时间。
 */
uint8_t SAFE_Check(float vbus, float ia, float ib, float ic) {
    /* 低频路径再次检查硬过流，作为高速检测之外的保护冗余。 */
    float i_max = fabsf(ia);
    if (fabsf(ib) > i_max) i_max = fabsf(ib);
    if (fabsf(ic) > i_max) i_max = fabsf(ic);

    if (i_max > DRV_OC_HARD_LIMIT) {
        return FAULT_OC_HARD; // 硬阈值过流，25A
    }

    /* 母线过压需连续三个低频样本超过阈值才触发，降低瞬态毛刺误报。 */
    static uint8_t ovp_debounce = 0;

    if (vbus > DRV_OVP_THRESHOLD) {
        if (++ovp_debounce >= 3) return FAULT_OVP;
    } else {
        ovp_debounce = 0;
    }

    /* 仅非 IDLE 状态检测欠压；连续五个低频样本低于门限后触发。 */
    static uint8_t uvp_debounce = 0;
    if (g_motor.state != STATE_IDLE && vbus < DRV_UVP_THRESHOLD) {
     
        if (++uvp_debounce >= 5) return FAULT_UVP;
    } else {
        uvp_debounce = 0;
    }

    return FAULT_NONE; // 正常
}

/**
 * @brief 主循环中的故障锁存与人工复位处理。
 *
 * 处于 FAULT 时持续关闭功率级并更新指示灯；仅在按键按下、目标
 * 转速近零、母线处于恢复区间且三相电流近零时清除故障并复位控制器。
 */
void SAFE_HandleFault(void) {
    if (g_motor.state == STATE_FAULT) {
        /* 故障期间每次轮询都确保驱动使能保持关闭。 */
        HW_PWM_Disable();
        
        /* 红灯亮、绿灯灭表示故障锁存。 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // 红亮
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);   // 绿灭

        /* 人工复位需同时满足安全电流、转速及母线电压条件。 */
        uint8_t key_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET);
        float vbus = HW_ADC_GetVbus();
        if (key_pressed &&
            fabsf(g_motor.target_rpm) < 10.0f &&
            vbus > DRV_UVP_RECOVER &&
            vbus < DRV_OVP_RECOVER &&
            fabsf(g_motor.i_a) < 0.5f &&
            fabsf(g_motor.i_b) < 0.5f &&
            fabsf(g_motor.i_c) < 0.5f) {
            /* 清除故障状态、重置观测器和 PI，并将输出参考归零。 */
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
