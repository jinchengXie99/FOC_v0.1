#ifndef MATH_SVPWM_H
#define MATH_SVPWM_H

/* alpha/beta 电压 (V) 与母线电压 (V) 转换为三相 [0,1] 占空比。 */
void Math_SVPWM_Gen(float v_alpha, float v_beta, float vbus, float *duty_a, float *duty_b, float *duty_c);

#endif
