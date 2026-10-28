#ifndef HW_ADC_H
#define HW_ADC_H

#include <stdint.h>

#define ADC_REGULAR_BUF_SIZE 2U

/* ============================================================
 * 外部引用的 DMA 缓冲区
 * 需在 adc.c 或 main.c 中定义为：
 * volatile uint16_t adc_regular_dma_buf[2] = {0, 0};
 * ============================================================ */
extern volatile uint16_t adc_regular_dma_buf[ADC_REGULAR_BUF_SIZE];
extern volatile uint8_t g_calib_done;

/* ============================================================
 * 偏置校准状态机接口
 * ============================================================ */
/* 
 * 在校准期间，传入当前注入组的原始 ADC 值。
 * 返回 1 表示校准完成，0 表示还在校准中。
 * 注意：调用此函数时，必须确保 PWM 未输出（电机静止）。
 */
uint8_t HW_ADC_CalibrateInjected(uint16_t raw_ia, uint16_t raw_ic);

/* ============================================================
 * 数据极速读取接口
 * ============================================================ */
/* 读取三相物理电流 (A) - 耗时 < 1us */
void HW_ADC_GetCurrents(float *ia, float *ib, float *ic);

/* 读取母线电压 (V) - 从 DMA 内存直接取 */
float HW_ADC_GetVbus(void);

/* 读取调速器电压 (V) - 从 DMA 内存直接取 */
float HW_ADC_GetThrottleVoltage(void);

#endif /* HW_ADC_H */
