#ifndef HW_CORDIC_H
#define HW_CORDIC_H
#include <stdint.h>

void HW_CORDIC_Init(void);
void HW_CORDIC_SinCos(float angle, float *p_sin, float *p_cos);

#endif
