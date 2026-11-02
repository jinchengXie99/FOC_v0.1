#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include "ctrl_state_machine.h"

/* 在低频任务中打包并启动一次非阻塞 VOFA+ UART DMA 发送。 */
void APP_Telemetry_Send(Motor_Ctrl_t *motor);

/* 由 UART TX DMA 完成回调调用，释放发送缓冲区。 */
void APP_Telemetry_TxCpltCallback(void);

#endif
