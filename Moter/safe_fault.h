#ifndef SAFE_FAULT_H
#define SAFE_FAULT_H

#include <stdint.h>

/* 故障代码 */
#define FAULT_NONE 0
#define FAULT_OC_HARD 1
#define FAULT_OVP 2
#define FAULT_UVP 3
#define FAULT_OC_SOFT 4

uint8_t SAFE_Check(float vbus, float ia, float ib, float ic);
uint8_t SAFE_CheckCurrentFast(float ia, float ib, float ic);
void SAFE_HandleFault(void);

#endif
