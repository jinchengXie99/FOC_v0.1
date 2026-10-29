#include "app_telemetry.h"
#include "usart.h"
#include <string.h>

/* ============================================================
 * VOFA+ JustFloat 协议帧定义
 * ============================================================ */
#define VOFA_CH_COUNT   6
typedef struct {
    float ch[VOFA_CH_COUNT];
    uint8_t tail[4];
} VOFA_Frame_t;

static VOFA_Frame_t tx_frame = {{0.0f}, {0x00, 0x00, 0x80, 0x7F}};
static volatile uint8_t tx_busy = 0;
static uint16_t tx_timeout_cnt = 0;

/* ============================================================
 * DMA 发送完成回调 (在 stm32g4xx_it.c 或 main.c 中调用)
 * ============================================================ */
void APP_Telemetry_TxCpltCallback(void) {
    tx_busy = 0;
    tx_timeout_cnt = 0;
}

/* ============================================================
 * 填装数据并发起 DMA 发送 (1kHz 任务中调用)
 * ============================================================ */
void APP_Telemetry_Send(Motor_Ctrl_t *motor) {
    // 1. 检查是否空闲
    if (tx_busy == 0) {
        tx_busy = 1;
        tx_timeout_cnt = 0;
        
        // 2. 填充通道 (根据你的需求调整)
        tx_frame.ch[0] = motor->target_rpm;     // 目标转速
        tx_frame.ch[1] = motor->obs.speed_rpm_filt;  // 实际转速
        tx_frame.ch[2] = motor->iq_ref;         // Iq 目标
        tx_frame.ch[3] = motor->i_q;            // Iq 实际
        tx_frame.ch[4] = motor->id_fw_ref;      // id 目标
        tx_frame.ch[5] = motor->i_d;            // i_d 实际

        // 3. 启动 DMA 发送
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&tx_frame, sizeof(VOFA_Frame_t));
    } else {
        // 商用防死锁：如果连续 5ms 没发完，强制干掉 DMA
        if (++tx_timeout_cnt > 5) {
            HAL_UART_AbortTransmit(&huart1);
            tx_busy = 0;
            tx_timeout_cnt = 0;
        }
    }
}
