#include "app_telemetry.h"
#include "usart.h"
#include <string.h>

/* VOFA+ JustFloat：6 个 32 位浮点数据后跟 4 字节帧尾。 */
#define VOFA_CH_COUNT   6
typedef struct {
    float ch[VOFA_CH_COUNT]; /* 连续发送的遥测通道。 */
    uint8_t tail[4];         /* JustFloat 帧结束标记。 */
} VOFA_Frame_t;

/* 静态发送缓冲区在 DMA 完成前必须保持有效且不可重写。 */
static VOFA_Frame_t tx_frame = {{0.0f}, {0x00, 0x00, 0x80, 0x7F}};
static volatile uint8_t tx_busy = 0;
/* 以 1 kHz 任务周期为单位累计 DMA 占用时间。 */
static uint16_t tx_timeout_cnt = 0;

/** @brief UART DMA 完成时清除发送占用标志。 */
void APP_Telemetry_TxCpltCallback(void) {
    tx_busy = 0;
    tx_timeout_cnt = 0;
}

/**
 * @brief 打包当前控制状态并尝试通过 UART DMA 发送。
 *
 * 通道顺序为：目标转速、滤波转速、Iq 参考、Iq 测量、弱磁 Id 参考、
 * Id 测量。函数仅在上一次发送已完成时启动新帧。
 *
 * @param motor 电机状态对象
 */
void APP_Telemetry_Send(Motor_Ctrl_t *motor) {
    /* DMA 正在使用静态帧缓冲区时不可覆盖该缓冲区。 */
    if (tx_busy == 0) {
        tx_busy = 1;
        tx_timeout_cnt = 0;
        
        /* 填充 VOFA+ 通道；数值单位分别为 RPM、A。 */
        tx_frame.ch[0] = motor->target_rpm;     // 目标转速
        tx_frame.ch[1] = motor->obs.speed_rpm_filt;  // 实际转速
        tx_frame.ch[2] = motor->iq_ref;         // Iq 目标
        tx_frame.ch[3] = motor->i_q;            // Iq 实际
        tx_frame.ch[4] = motor->id_fw_ref;      // id 目标
        tx_frame.ch[5] = motor->i_d;            // i_d 实际

        /* 启动异步发送，完成后由 APP_Telemetry_TxCpltCallback 释放缓冲区。 */
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&tx_frame, sizeof(VOFA_Frame_t));
    } else {
        /* 超时后中止卡住的传输，允许后续任务重新启动发送。 */
        if (++tx_timeout_cnt > 5) {
            HAL_UART_AbortTransmit(&huart1);
            tx_busy = 0;
            tx_timeout_cnt = 0;
        }
    }
}
