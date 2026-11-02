#include "app_throttle.h"
#include <stdint.h>

/* ============================================================
 * 调速器参数定义 (可根据实际电位器调整)
 * ============================================================ */
#define THROTTLE_V_MIN      0.1f    /* 起调电压 (V)，留出死区防止抖动 */
#define THROTTLE_V_MAX      3.2f    /* 满速电压 (V) */
#define THROTTLE_V_CENTER   1.65f   /* 中位电压 (V)，对应零速 */
#define THROTTLE_V_DEADBAND 0.08f   /* 中位死区半宽 (V) */
#define THROTTLE_RPM_MAX    4000.0f /* 目标最高转速 (RPM) */
#define THROTTLE_LPF_ALPHA  0.01f   /* 一阶低通滤波系数，越小越平滑 */

static float throttle_v_filt;
static uint8_t throttle_filter_initialized;

/* ============================================================
 * 处理 ADC 原始值，输出目标转速
 * ============================================================ */
void APP_Throttle_Update(float v_raw, float *target_rpm) {
    if (v_raw < THROTTLE_V_MIN) {
        v_raw = THROTTLE_V_MIN;
    } else if (v_raw > THROTTLE_V_MAX) {
        v_raw = THROTTLE_V_MAX;
    }

    if (!throttle_filter_initialized) {
        throttle_v_filt = v_raw;
        throttle_filter_initialized = 1U;
    } else {
        throttle_v_filt = throttle_v_filt * (1.0f - THROTTLE_LPF_ALPHA) +
                          v_raw * THROTTLE_LPF_ALPHA;
    }

    if (throttle_v_filt < THROTTLE_V_CENTER - THROTTLE_V_DEADBAND) {
        float span = THROTTLE_V_CENTER - THROTTLE_V_DEADBAND - THROTTLE_V_MIN;
        float ratio = (THROTTLE_V_CENTER - THROTTLE_V_DEADBAND - throttle_v_filt) / span;
        *target_rpm = -ratio * THROTTLE_RPM_MAX;
    } else if (throttle_v_filt > THROTTLE_V_CENTER + THROTTLE_V_DEADBAND) {
        float span = THROTTLE_V_MAX - THROTTLE_V_CENTER - THROTTLE_V_DEADBAND;
        float ratio = (throttle_v_filt - THROTTLE_V_CENTER - THROTTLE_V_DEADBAND) / span;
        *target_rpm = ratio * THROTTLE_RPM_MAX;
    } else {
        *target_rpm = 0.0f;
    }
}
