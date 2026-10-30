#include "app_throttle.h"

/* ============================================================
 * 调速器参数定义 (可根据实际电位器调整)
 * ============================================================ */
#define THROTTLE_V_MIN      0.1f    /* 起调电压 (V)，留出死区防止抖动 */
#define THROTTLE_V_MAX      3.2f    /* 满速电压 (V) */
#define THROTTLE_RPM_MAX    4000.0f /* 目标最高转速 (RPM) */
#define THROTTLE_LPF_ALPHA  0.01f   /* 一阶低通滤波系数，越小越平滑 */

static float throttle_v_filt = 0.0f; // 滤波后的电压状态

/* ============================================================
 * 处理 ADC 原始值，输出目标转速
 * ============================================================ */
void APP_Throttle_Update(float v_raw, float *target_rpm) {
    // 1. 一阶低通滤波 (消除电位器的高频接触噪声)
    throttle_v_filt = throttle_v_filt * (1.0f - THROTTLE_LPF_ALPHA) + v_raw * THROTTLE_LPF_ALPHA;

    // 2. 死区与映射
    if (throttle_v_filt < THROTTLE_V_MIN) {
        *target_rpm = 0.0f;
    } else if (throttle_v_filt > THROTTLE_V_MAX) {
        *target_rpm = THROTTLE_RPM_MAX;
    } else {
        float ratio = (throttle_v_filt - THROTTLE_V_MIN) / (THROTTLE_V_MAX - THROTTLE_V_MIN);
        *target_rpm = ratio * THROTTLE_RPM_MAX;
    }
}
