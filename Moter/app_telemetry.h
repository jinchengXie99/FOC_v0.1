#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include "ctrl_state_machine.h"

void APP_Telemetry_Send(Motor_Ctrl_t *motor);
void APP_Telemetry_TxCpltCallback(void);

#endif
