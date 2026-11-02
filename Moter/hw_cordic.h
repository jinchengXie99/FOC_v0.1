#ifndef HW_CORDIC_H
#define HW_CORDIC_H
#include <stdint.h>

/* 初始化 CORDIC 外设；应在控制算法调用正余弦之前执行一次。 */
void HW_CORDIC_Init(void);
/* angle 以弧度输入，输出 sin(angle) 与 cos(angle)。 */
void HW_CORDIC_SinCos(float angle, float *p_sin, float *p_cos);

#endif
