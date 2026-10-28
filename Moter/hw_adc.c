#include "hw_adc.h"
#include "drv_config.h"
#include "stm32g4xx.h"

/* ============================================================
 * 内部私有变量 (用于偏置自动校准)
 * ============================================================ */
#define CALIB_SAMPLES 2000  /* 采集 2000 次求平均 (20kHz下耗时 100ms) */

volatile uint16_t adc_regular_dma_buf[ADC_REGULAR_BUF_SIZE] = {0U, 0U};
volatile uint8_t g_calib_done = 0U;

static uint32_t sum_ia = 0;
static uint32_t sum_ic = 0;
static uint16_t calib_cnt = 0;

/* 经过校准后的精准偏置值 (默认给 1.65V 对应的理论值 2048) */
static float ia_offset = 2048.0f;
static float ic_offset = 2048.0f;

/* ============================================================
 * 偏置自动校准状态机
 * 必须在 20kHz 的 ADC 中断最前端调用
 * ============================================================ */
uint8_t HW_ADC_CalibrateInjected(uint16_t raw_ia, uint16_t raw_ic) {
    if (g_calib_done) {
        return 1; 
    }

    sum_ia += raw_ia;
    sum_ic += raw_ic;
    calib_cnt++;

    if (calib_cnt >= CALIB_SAMPLES) {
        // 计算平均值并保存
        ia_offset = (float)sum_ia / (float)CALIB_SAMPLES;
        ic_offset = (float)sum_ic / (float)CALIB_SAMPLES;
        g_calib_done = 1U;
    }
    return 0;
}

/* ============================================================
 * 极速读取三相电流
 * 跳过 HAL 库的 HAL_ADCEx_InjectedGetValue() 函数，
 * 直接读寄存器 ADC2->JDRx，省去函数跳转开销。
 * ============================================================ */
void HW_ADC_GetCurrents(float *ia, float *ib, float *ic) {
    // 1. 直接读取注入数据寄存器 (对应 CubeMX 中 Rank1 和 Rank2)
    // PA6 -> ADC2_IN3 -> Rank1 -> JDR1
    // PA4 -> ADC2_IN17 -> Rank2 -> JDR2
    uint16_t raw_ia = (uint16_t)(ADC2->JDR1 & 0xFFFF);
    uint16_t raw_ic = (uint16_t)(ADC2->JDR2 & 0xFFFF);

    // 2. 减去偏置电压并乘以换算系数 (CURRENT_SCALE 在 drv_config.h 中已定义)
    *ia = ((float)raw_ia - ia_offset) * DRV_CURRENT_SCALE;
    *ic = ((float)raw_ic - ic_offset) * DRV_CURRENT_SCALE;
    
    // 3. 基尔霍夫电流定律算 B 相
    *ib = -(*ia) - (*ic);
}

/* ============================================================
 * 读取母线电压与调速器电压 (直接读 DMA 内存)
 * ============================================================ */
float HW_ADC_GetVbus(void) {
    // 读内存速度极快，无需任何等待
    return (float)adc_regular_dma_buf[0] * DRV_VBUS_SCALE;
}

float HW_ADC_GetThrottleVoltage(void) {
    return (float)adc_regular_dma_buf[1] * DRV_THROTTLE_SCALE;
}
