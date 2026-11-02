#ifndef CTRL_CURRENT_LOOP_H
#define CTRL_CURRENT_LOOP_H

#include "ctrl_state_machine.h"

#define  USE_DECOUPLING  1

/**
 * @brief 执行一次 dq 电流闭环控制并更新三相 PWM 占空比。
 * @param motor 电机控制状态及 PI、观测器
 * @param vbus 实时直流母线电压 (V)
 * @param dt 电流环采样周期 (s)，通常为 FOC_DT
 */
void Ctrl_CurrentLoop_Execute(Motor_Ctrl_t *motor, float vbus, float dt);

#endif
