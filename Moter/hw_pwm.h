#ifndef HW_PWM_H
#define HW_PWM_H

#include "main.h"
#include "drv_config.h"  /* 引入引脚与 PWM 频率配置 */

/* ============================================================
 * PWM 底层驱动 (直接操作寄存器，确保微秒级响应)
 * ============================================================ */
#define HW_PWM_ARR              4249        /* 20kHz @170MHz, 中心对齐 */

/* 宏观的硬件紧急刹车 (拉低 SD 引脚) */
#define HW_PWM_Disable()        do { \
                                    HAL_GPIO_WritePin(DRV_SD1_PORT, DRV_SD1_PIN, GPIO_PIN_RESET); \
                                    HAL_GPIO_WritePin(DRV_SD2_PORT, DRV_SD2_PIN, GPIO_PIN_RESET); \
                                    HAL_GPIO_WritePin(DRV_SD3_PORT, DRV_SD3_PIN, GPIO_PIN_RESET); \
                                    TIM1->CCR1 = 0; \
                                    TIM1->CCR2 = 0; \
                                    TIM1->CCR3 = 0; \
                                } while(0)

/* 宏观的硬件使能 (拉高 SD 引脚) */
#define HW_PWM_Enable()         do { \
                                    HAL_GPIO_WritePin(DRV_SD1_PORT, DRV_SD1_PIN, GPIO_PIN_SET); \
                                    HAL_GPIO_WritePin(DRV_SD2_PORT, DRV_SD2_PIN, GPIO_PIN_SET); \
                                    HAL_GPIO_WritePin(DRV_SD3_PORT, DRV_SD3_PIN, GPIO_PIN_SET); \
                                } while(0)

/* ============================================================
 * 极速写入三相占空比
 * 注意：加入强制限幅保护，防止浮点误差越界导致炸管
 * ============================================================ */
static inline void HW_PWM_SetDuty(float a, float b, float c) {
    /* 1. 限幅：确保占空比严格在 0.0 ~ 1.0 之间 */
    if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    if (c < 0.0f) c = 0.0f; else if (c > 1.0f) c = 1.0f;

    /* 2. 直接赋值寄存器 */
    TIM1->CCR1 = (uint32_t)(a * HW_PWM_ARR);
    TIM1->CCR2 = (uint32_t)(b * HW_PWM_ARR);
    TIM1->CCR3 = (uint32_t)(c * HW_PWM_ARR);
}

#endif /* HW_PWM_H */


