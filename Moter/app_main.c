#include "app_main.h"
#include "mtr_config.h"

Motor_Ctrl_t g_motor;

void APP_Init(void) {
    Ctrl_Observer_Init(&g_motor.obs);
   // 初始化速度环：最大 18A，最小 -10A
    Speed_PI_Init(&g_motor.pi_speed, SPD_KP, SPD_KI, MAX_IQ_CURRENT, MIN_IQ_CURRENT);
    
    // 初始化电流环：输出的是目标电压 Vd, Vq。假设最大电压为 15V
    Current_PI_Init(&g_motor.pi_id, CURRENT_KP, CURRENT_KI, 13.5f);
    Current_PI_Init(&g_motor.pi_iq, CURRENT_KP, CURRENT_KI, 13.5f);
   
    
    g_motor.state = STATE_IDLE;
    g_motor.target_rpm = 0.0f;
}
