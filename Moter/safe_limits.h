#ifndef SAFE_LIMITS_H
#define SAFE_LIMITS_H

/* 动态安全边界数据结构，供调用方保存母线电压和相电流限制。 */
typedef struct {
    float vbus_max;       /* 最大允许母线电压 (V)。 */
    float vbus_min;       /* 最小允许母线电压 (V)。 */
    float current_max;    /* 最大允许电流 (A)。 */
    float current_min;    /* 最小允许电流 (A)。 */
} Safe_Limits_t;

/* 根据当前母线电压 (V) 更新安全电压/电流范围；计划按约 1 ms 调用。 */
void Safe_Limits_Update(Safe_Limits_t *limits, float vbus_voltage);

#endif