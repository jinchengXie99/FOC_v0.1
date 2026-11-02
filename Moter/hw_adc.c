#include "hw_adc.h"
#include "drv_config.h"
#include "stm32g4xx.h"

/* 注入组零电流偏置校准样本数；20 kHz 下累计约 100 ms。 */
#define CALIB_SAMPLES 2000  /* 采集 2000 次求平均 (20kHz下耗时 100ms) */

volatile uint16_t adc_regular_dma_buf[ADC_REGULAR_BUF_SIZE] = {0U, 0U};
volatile uint8_t g_calib_done = 0U;

static uint32_t sum_ia = 0;
static uint32_t sum_ic = 0;
static uint16_t calib_cnt = 0;

/* 经过校准后的精准偏置值 (默认给 1.65V 对应的理论值 2048) */
static float ia_offset = 2048.0f;
static float ic_offset = 2048.0f;

/**
 * @brief 累计注入组原始电流 ADC 样本并估算零电流偏置。
 *
 * 校准完成标志置位后后续调用不再更新偏置。调用期间电机必须静止、
 * PWM 不得输出，否则偏置中将混入实际相电流。
 * @return 校准完成返回 1，尚未完成返回 0
 */
uint8_t HW_ADC_CalibrateInjected(uint16_t raw_ia, uint16_t raw_ic) {
    if (g_calib_done) {
        return 1; 
    }

    sum_ia += raw_ia;
    sum_ic += raw_ic;
    calib_cnt++;

    if (calib_cnt >= CALIB_SAMPLES) {
        /* 对两个采样通道分别取样本平均值，作为后续电流换算零点。 */
        ia_offset = (float)sum_ia / (float)CALIB_SAMPLES;
        ic_offset = (float)sum_ic / (float)CALIB_SAMPLES;
        g_calib_done = 1U;
    }
    return 0;
}

/**
 * @brief 读取 ADC2 注入组电流样本并换算为三相电流 (A)。
 *
 * Rank1/JDR1 对应 IA，Rank2/JDR2 对应 IC；IB 根据三相电流和为零
 * 的关系推算。寄存器映射必须与 CubeMX ADC 注入组配置一致。
 */
void HW_ADC_GetCurrents(volatile float *ia, volatile float *ib, volatile float *ic) {
    /* 直接读取注入数据寄存器；PA6/ADC2_IN3 为 Rank1，PA4/ADC2_IN17 为 Rank2。 */
    uint16_t raw_ia = (uint16_t)(ADC2->JDR1 & 0xFFFF);
    uint16_t raw_ic = (uint16_t)(ADC2->JDR2 & 0xFFFF);

    /* 减去自动校准零点，再按 drv_config.h 中的 A/LSB 系数换算。 */
    *ia = ((float)raw_ia - ia_offset) * DRV_CURRENT_SCALE;
    *ic = ((float)raw_ic - ic_offset) * DRV_CURRENT_SCALE;
    
    /* 两相采样方案：按 ia + ib + ic = 0 推算未采样的 B 相电流。 */
    *ib = -(*ia) - (*ic);
}

/** @brief 读取规则组 DMA 缓冲区通道 0，并换算母线电压 (V)。 */
float HW_ADC_GetVbus(void) {
    /* DMA 必须已配置为通道 0 采集母线分压。 */
    return (float)adc_regular_dma_buf[0] * DRV_VBUS_SCALE;
}

/** @brief 读取规则组 DMA 缓冲区通道 1，并换算调速器输入电压 (V)。 */
float HW_ADC_GetThrottleVoltage(void) {
    return (float)adc_regular_dma_buf[1] * DRV_THROTTLE_SCALE;
}
