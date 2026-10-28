#include "hw_cordic.h"
#include "stm32g4xx.h"
#include <stdint.h>

void HW_CORDIC_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    CORDIC->CSR = (6UL << CORDIC_CSR_PRECISION_Pos) |
                  (1UL << CORDIC_CSR_NRES_Pos);
}

void HW_CORDIC_SinCos(float angle, float *p_sin, float *p_cos) {
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;
    float normalized_angle;
    int32_t angle_q31;
    int32_t cos_q31;
    int32_t sin_q31;

    while (angle > pi) {
        angle -= two_pi;
    }
    while (angle < -pi) {
        angle += two_pi;
    }

    normalized_angle = angle * 0.31830988618379067154f;
    if (normalized_angle >= 1.0f) {
        angle_q31 = 0x7FFFFFFF;
    } else if (normalized_angle <= -1.0f) {
        angle_q31 = (-2147483647 - 1);
    } else {
        angle_q31 = (int32_t)(normalized_angle * 2147483520.0f);
    }

    CORDIC->WDATA = (uint32_t)angle_q31;
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) {
    }

    cos_q31 = (int32_t)CORDIC->RDATA;
    sin_q31 = (int32_t)CORDIC->RDATA;
    *p_cos = (float)cos_q31 * 4.656612873077392578125e-10f;
    *p_sin = (float)sin_q31 * 4.656612873077392578125e-10f;
}
