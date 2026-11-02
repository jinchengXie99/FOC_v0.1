#ifndef APP_THROTTLE_H
#define APP_THROTTLE_H

/**
 * @brief 将油门电压转换为带符号转速命令。
 * @param v_raw ADC 换算后的电压 (V)
 * @param target_rpm 输出目标转速 (RPM)：负值反转，正值正转，零值停机/制动
 */
void APP_Throttle_Update(float v_raw, float *target_rpm);

#endif
