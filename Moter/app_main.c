#include "app_main.h"
#include "mtr_config.h"

/* 应用层共享的电机控制对象，供主程序、定时器及 ADC 回调共同使用。 */
Motor_Ctrl_t g_motor;

/**
 * @brief 初始化电机应用层控制对象及各控制器。
 *
 * 电流 PI 的动态电压限幅由 20 kHz 电流环根据实测母线电压更新，
 * 因此此处仅清零控制器状态；速度 PI 的限幅由电机配置给定。
 */
void APP_Init(void) {
    Ctrl_Observer_Init(&g_motor.obs);
    /* 速度 PI 输出为 q 轴电流参考，单位 A。 */
    Speed_PI_Init(&g_motor.pi_speed, SPD_KP, SPD_KI, MAX_IQ_CURRENT, MIN_IQ_CURRENT);
    
    /* 电流 PI 输出单位为 V；工作时的上下限由实时母线电压决定。 */
    Current_PI_Init(&g_motor.pi_id, CURRENT_KP, CURRENT_KI, 0.0f);
    Current_PI_Init(&g_motor.pi_iq, CURRENT_KP, CURRENT_KI, 0.0f);
   
    /* 上电先保持空闲，并清除外部转速命令。 */
    g_motor.state = STATE_IDLE;
    g_motor.target_rpm = 0.0f;
}
