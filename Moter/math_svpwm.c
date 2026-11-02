#include "math_svpwm.h"

/**
 * @brief 根据静止坐标系电压指令生成三相 SVPWM 占空比。
 *
 * 输入电压应为反 Park 变换后的 alpha-beta 分量，单位为伏特；
 * vbus 为直流母线电压，单位为伏特。输出占空比范围为 [0, 1]，
 * 可直接用于三相 PWM 比较值计算。
 *
 * 本函数使用 min/max 零序注入：将三相电压最大值和最小值的中点
 * 平移到零，从而居中利用 PWM 线性调制范围。调用方负责确保
 * vbus 是有效的母线测量值；这里对过低的 vbus 仅做除数保护，
 * 不代替欠压检测。
 *
 * @param v_alpha 静止坐标系 alpha 轴电压指令 (V)
 * @param v_beta  静止坐标系 beta 轴电压指令 (V)
 * @param vbus    直流母线电压 (V)
 * @param duty_a  输出 A 相归一化占空比，不能为 NULL
 * @param duty_b  输出 B 相归一化占空比，不能为 NULL
 * @param duty_c  输出 C 相归一化占空比，不能为 NULL
 */
void Math_SVPWM_Gen(float v_alpha, float v_beta, float vbus, float *duty_a, float *duty_b, float *duty_c) {
    // 反 Clarke 变换：把 alpha-beta 电压转换为三相相电压指令。
    // 系统采用幅值不变的 Clarke 变换，因此 A 相直接等于 alpha 分量。
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.86602540f * v_beta;
    float vc = -0.5f * v_alpha - 0.86602540f * v_beta;

    // 求三相指令的最大、最小值，用于计算公共零序分量。
    float v_max = va;
    if (vb > v_max) v_max = vb;
    if (vc > v_max) v_max = vc;

    float v_min = va;
    if (vb < v_min) v_min = vb;
    if (vc < v_min) v_min = vc;

    // min/max 零序注入：将最大和最小相电压对称地移到零点两侧，
    // 保持线电压不变，并尽可能提高直流母线的电压利用率。
    float v_common = -(v_max + v_min) * 0.5f;
    va += v_common;
    vb += v_common;
    vc += v_common;

    // 保护除法的分母。正常运行时母线电压应已由上层欠压保护校验；
    // 10 V 下限只避免异常输入导致除零或占空比计算溢出。
    float vbus_safe = (vbus < 10.0f) ? 10.0f : vbus;
    float inv_vbus = 1.0f / vbus_safe;

    // 对两电平逆变器，零电压对应 50% 占空比；相电压除以母线电压
    // 得到相对占空比偏移。先复用母线倒数，避免三次浮点除法。
    *duty_a = 0.5f + va * inv_vbus;
    *duty_b = 0.5f + vb * inv_vbus;
    *duty_c = 0.5f + vc * inv_vbus;

    // 最终边界保护，确保写入定时器的比较值不会超出 PWM 周期。
    // 若指令超出可调制范围，此处会截断对应相的占空比。
    if (*duty_a < 0.0f) *duty_a = 0.0f; else if (*duty_a > 1.0f) *duty_a = 1.0f;
    if (*duty_b < 0.0f) *duty_b = 0.0f; else if (*duty_b > 1.0f) *duty_b = 1.0f;
    if (*duty_c < 0.0f) *duty_c = 0.0f; else if (*duty_c > 1.0f) *duty_c = 1.0f;
}
