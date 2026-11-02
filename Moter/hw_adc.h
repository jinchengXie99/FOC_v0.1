#ifndef HW_ADC_H
#define HW_ADC_H

#include <stdint.h>

#define ADC_REGULAR_BUF_SIZE 2U

/* 规则组 DMA 采样缓冲区：索引 0 为母线，索引 1 为调速器电压。 */
extern volatile uint16_t adc_regular_dma_buf[ADC_REGULAR_BUF_SIZE];
extern volatile uint8_t g_calib_done;

/* 在电机静止且 PWM 关闭时累计注入组原始采样，返回是否完成校准。 */
uint8_t HW_ADC_CalibrateInjected(uint16_t raw_ia, uint16_t raw_ic);

/* 从 ADC2 JDR1/JDR2 读取两相电流并推算第三相，输出单位 A。 */
void HW_ADC_GetCurrents(volatile float *ia, volatile float *ib, volatile float *ic);

/* 读取 DMA 通道 0 的母线电压，单位 V。 */
float HW_ADC_GetVbus(void);

/* 读取 DMA 通道 1 的调速器电压，单位 V。 */
float HW_ADC_GetThrottleVoltage(void);

#endif /* HW_ADC_H */
