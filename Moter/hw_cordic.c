#include "hw_cordic.h"
#include "stm32g4xx.h"
#include <stdint.h>

/** @brief 使能 STM32G4 CORDIC 时钟并配置精度及双结果输出模式。 */
void HW_CORDIC_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    CORDIC->CSR = (6UL << CORDIC_CSR_PRECISION_Pos) |
                  (1UL << CORDIC_CSR_NRES_Pos);
}

/**
 * @brief 使用 CORDIC 硬件计算输入角度的正弦和余弦。
 * @param angle 输入角度 (rad)，先折返至 [-pi, pi]
 * @param p_sin 正弦输出指针
 * @param p_cos 余弦输出指针
 *
 * 角度转换为 CORDIC 所需 Q1.31 形式后写入 WDATA；等待 RRDY，
 * 再按已配置的结果顺序读取余弦和正弦并换算为浮点数。
 */
void HW_CORDIC_SinCos(float angle, float *p_sin, float *p_cos) {
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;
    float normalized_angle;
    int32_t angle_q31;
    int32_t cos_q31;
    int32_t sin_q31;

    /* 将大角度规约到硬件支持的一个完整周期内。 */
    while (angle > pi) {
        angle -= two_pi;
    }
    while (angle < -pi) {
        angle += two_pi;
    }

    /* CORDIC 角度输入以 pi 为单位，故先将弧度乘以 1/pi。 */
    normalized_angle = angle * 0.31830988618379067154f;
    if (normalized_angle >= 1.0f) {
        angle_q31 = 0x7FFFFFFF;
    } else if (normalized_angle <= -1.0f) {
        angle_q31 = (-2147483647 - 1);
    } else {
        angle_q31 = (int32_t)(normalized_angle * 2147483520.0f);
    }

    /* 写入单个角度后，阻塞等待本次双结果计算完成。 */
    CORDIC->WDATA = (uint32_t)angle_q31;
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) {
    }

    cos_q31 = (int32_t)CORDIC->RDATA;
    sin_q31 = (int32_t)CORDIC->RDATA;
    *p_cos = (float)cos_q31 * 4.656612873077392578125e-10f;
    *p_sin = (float)sin_q31 * 4.656612873077392578125e-10f;
}
