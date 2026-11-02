#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "ctrl_state_machine.h" /* Motor_Ctrl_t 类型定义 */
#include "ctrl_pi.h"

/* FOC 电流环周期：20 kHz，对应 50 us。 */
#define FOC_DT 0.00005f

/* 初始化全局电机状态、观测器和 PI 控制器。 */
void APP_Init(void);

/* 应用层唯一的电机状态实例。 */
extern Motor_Ctrl_t g_motor;

#endif
