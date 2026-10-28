#ifndef CTRL_CURRENT_LOOP_H
#define CTRL_CURRENT_LOOP_H

#include "ctrl_state_machine.h"

#define  USE_DECOUPLING  1

void Ctrl_CurrentLoop_Execute(Motor_Ctrl_t *motor, float vbus, float dt);

#endif
