#ifndef SAFE_LIMITS_H
#define SAFE_LIMITS_H

/* ============================================================
 * 动态限幅器接口
 * ============================================================ */
typedef struct {
    float vbus_max;
    float vbus_min;
    float current_max;
    float current_min;
} Safe_Limits_t;

/* 动态计算安全电压和电流边界 (每1ms执行一次) */
void Safe_Limits_Update(Safe_Limits_t *limits, float vbus_voltage);

#endif