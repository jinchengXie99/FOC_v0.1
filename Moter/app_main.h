#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "ctrl_state_machine.h" // 为了包含 Motor_Ctrl_t
#include "ctrl_pi.h"
#define FOC_DT 0.00005f       // 20kHz 周期

void APP_Init(void);
extern Motor_Ctrl_t g_motor;

#endif
