#include "app_throttle.h"
#include <stdint.h>

/* 油门标定参数。中位电压及死区应按实际 ADC 测量结果校准。 */
#define THROTTLE_V_MIN      0.1f    /* 输入电压下限 (V)，低于此值按下限处理。 */
#define THROTTLE_V_MAX      3.2f    /* 输入电压上限 (V)，高于此值按上限处理。 */
#define THROTTLE_V_CENTER   1.65f   /* 机械中位电压 (V)，对应零速命令。 */
#define THROTTLE_V_DEADBAND 0.08f   /* 中位两侧死区半宽 (V)。 */
#define THROTTLE_RPM_MAX    4000.0f /* 正、反向最大目标转速绝对值 (RPM)。 */
#define THROTTLE_LPF_ALPHA  0.01f   /* 一阶低通系数；越小越平滑，响应越慢。 */

/* 1 kHz 调用的油门滤波状态。 */
static float throttle_v_filt;
static uint8_t throttle_filter_initialized;

/**
 * @brief 滤波并映射油门电压为带符号目标转速。
 *
 * 低于中位死区时输出负 RPM（反转），高于中位死区时输出正 RPM
 * （正转），死区内输出 0 RPM。首次调用直接以当前输入初始化滤波器，
 * 避免滤波状态从零电压缓慢爬升而产生错误启动命令。
 *
 * @param v_raw ADC 换算后的油门电压 (V)
 * @param target_rpm 输出目标转速 (RPM)，符号表示方向
 */
void APP_Throttle_Update(float v_raw, float *target_rpm) {
    /* 将测量值限制在标定区间，保证后续映射比例处于有效范围。 */
    if (v_raw < THROTTLE_V_MIN) {
        v_raw = THROTTLE_V_MIN;
    } else if (v_raw > THROTTLE_V_MAX) {
        v_raw = THROTTLE_V_MAX;
    }

    /* 首次采样直接初始化；后续采样使用一阶低通抑制电位器噪声。 */
    if (!throttle_filter_initialized) {
        throttle_v_filt = v_raw;
        throttle_filter_initialized = 1U;
    } else {
        throttle_v_filt = throttle_v_filt * (1.0f - THROTTLE_LPF_ALPHA) +
                          v_raw * THROTTLE_LPF_ALPHA;
    }

    /* 中位以下映射为负转速；跨度单独计算，使死区边缘映射到零。 */
    if (throttle_v_filt < THROTTLE_V_CENTER - THROTTLE_V_DEADBAND) {
        float span = THROTTLE_V_CENTER - THROTTLE_V_DEADBAND - THROTTLE_V_MIN;
        float ratio = (THROTTLE_V_CENTER - THROTTLE_V_DEADBAND - throttle_v_filt) / span;
        *target_rpm = -ratio * THROTTLE_RPM_MAX;
    /* 中位以上映射为正转速；最高端点映射到 +THROTTLE_RPM_MAX。 */
    } else if (throttle_v_filt > THROTTLE_V_CENTER + THROTTLE_V_DEADBAND) {
        float span = THROTTLE_V_MAX - THROTTLE_V_CENTER - THROTTLE_V_DEADBAND;
        float ratio = (throttle_v_filt - THROTTLE_V_CENTER - THROTTLE_V_DEADBAND) / span;
        *target_rpm = ratio * THROTTLE_RPM_MAX;
    } else {
        *target_rpm = 0.0f;
    }
}
