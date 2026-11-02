#ifndef HW_PWM_H
#define HW_PWM_H

#include "main.h"
#include "drv_config.h"  /* 引入引脚与 PWM 频率配置 */

/* TIM1 三相 PWM 底层接口；占空比使用 0.0 到 1.0 的无量纲比例。 */
#define HW_PWM_ARR              4249        /* 20kHz @170MHz, 中心对齐 */

/* 拉低半桥驱动器 SD 使能并清零三相比较值，立即禁止 PWM 输出。 */
#define HW_PWM_Disable()        do { \
                                    HAL_GPIO_WritePin(DRV_SD1_PORT, DRV_SD1_PIN, GPIO_PIN_RESET); \
                                    HAL_GPIO_WritePin(DRV_SD2_PORT, DRV_SD2_PIN, GPIO_PIN_RESET); \
                                    HAL_GPIO_WritePin(DRV_SD3_PORT, DRV_SD3_PIN, GPIO_PIN_RESET); \
                                    TIM1->CCR1 = 0; \
                                    TIM1->CCR2 = 0; \
                                    TIM1->CCR3 = 0; \
                                } while(0)

/* 拉高三个半桥驱动器 SD 使能；调用前应先准备安全占空比。 */
#define HW_PWM_Enable()         do { \
                                    HAL_GPIO_WritePin(DRV_SD1_PORT, DRV_SD1_PIN, GPIO_PIN_SET); \
                                    HAL_GPIO_WritePin(DRV_SD2_PORT, DRV_SD2_PIN, GPIO_PIN_SET); \
                                    HAL_GPIO_WritePin(DRV_SD3_PORT, DRV_SD3_PIN, GPIO_PIN_SET); \
                                } while(0)

/* 写入三相 PWM 占空比；每相先钳位到 [0,1]，再换算为 TIM1 CCR 值。 */
static inline void HW_PWM_SetDuty(float a, float b, float c) {
    /* 硬件边界保护：避免无效/越界占空比转换成超周期比较值。 */
    if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    if (c < 0.0f) c = 0.0f; else if (c > 1.0f) c = 1.0f;

    /* 将归一化占空比乘以自动重装载值，直接更新三个比较寄存器。 */
    TIM1->CCR1 = (uint32_t)(a * HW_PWM_ARR);
    TIM1->CCR2 = (uint32_t)(b * HW_PWM_ARR);
    TIM1->CCR3 = (uint32_t)(c * HW_PWM_ARR);
}

#endif /* HW_PWM_H */

