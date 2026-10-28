#include "math_svpwm.h"
#include "hw_dsp.h"
#include "mtr_config.h"
#include <math.h>

/* ============================================================
 * 极速反 Park & SVPWM 零序注入
 * 输入：Vd, Vq (目标电压), angle (电角度), vbus (母线电压)
 * 输出：duty_a, duty_b, duty_c (归一化占空比 0.0 ~ 1.0)
 * ============================================================ */
void Math_SVPWM_Gen(float vd, float vq, float angle, float vbus, float *duty_a, float *duty_b, float *duty_c) {
    float sin_e, cos_e;
    // 调用 CORDIC 硬件加速算出 sin/cos
    cordic_sincos(angle, &sin_e, &cos_e);

    // 1. 反 Park 变换 (从 dq 到 alpha-beta)
    float v_alpha = vd * cos_e - vq * sin_e;
    float v_beta  = vd * sin_e + vq * cos_e;

    // 2. 反 Clarke 变换 (从 alpha-beta 到三相)
    // 使用最简算式，避免乘 2/3 带来的小数误差
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.86602540f * v_beta; // sqrt(3)/2
    float vc = -0.5f * v_alpha - 0.86602540f * v_beta;

    // 3. 提取最大最小值 (用于零序注入)
    float v_max = va;
    if (vb > v_max) v_max = vb;
    if (vc > v_max) v_max = vc;

    float v_min = va;
    if (vb < v_min) v_min = vb;
    if (vc < v_min) v_min = vc;

    // 4. 零序注入
    float v_common = -(v_max + v_min) * 0.5f;
    va += v_common;
    vb += v_common;
    vc += v_common;

    // 5. 计算占空比并做安全限幅
    float vbus_safe = (vbus < 10.0f) ? 10.0f : vbus; // 防止除零
    
    *duty_a = 0.5f + va / vbus_safe;
    *duty_b = 0.5f + vb / vbus_safe;
    *duty_c = 0.5f + vc / vbus_safe;

    // 强制限幅，保护 MOSFET
    if (*duty_a < 0.0f) *duty_a = 0.0f; else if (*duty_a > 1.0f) *duty_a = 1.0f;
    if (*duty_b < 0.0f) *duty_b = 0.0f; else if (*duty_b > 1.0f) *duty_b = 1.0f;
    if (*duty_c < 0.0f) *duty_c = 0.0f; else if (*duty_c > 1.0f) *duty_c = 1.0f;
}
