#ifndef SAFE_FAULT_H
#define SAFE_FAULT_H

#include <stdint.h>

/* 故障码：由安全检查写入 motor.fault_code，并在 FAULT 状态锁存。 */
#define FAULT_NONE 		0
#define FAULT_OC_HARD   1
#define FAULT_OVP 		2
#define FAULT_UVP 		3
#define FAULT_OC_SOFT   4
#define FAULT_BRAKE_TIMEOUT 5

/* 低频母线电压与电流保护检查；返回故障码或 FAULT_NONE。 */
uint8_t SAFE_Check(float vbus, float ia, float ib, float ic);
/* ADC 高频相电流保护；发生过流时立即锁存状态并关闭 PWM。 */
uint8_t SAFE_CheckCurrentFast(float ia, float ib, float ic);
/* 主循环轮询故障指示和满足条件后的人工复位。 */
void SAFE_HandleFault(void);

#endif
