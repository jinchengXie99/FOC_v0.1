#include "app_main.h"
#include "mtr_config.h"

Motor_Ctrl_t g_motor;

void APP_Init(void) {
    Ctrl_Observer_Init(&g_motor.obs);
    Math_PID_Init(&g_motor.pid_spd, SPD_KP, SPD_KI, 0,
                  MAX_IQ_CURRENT, MAX_IQ_CURRENT);
    Math_PID_Init(&g_motor.pid_id,  CURRENT_KP, CURRENT_KI, 0, 1.0f, 1.0f);
    Math_PID_Init(&g_motor.pid_iq,  CURRENT_KP, CURRENT_KI, 0, 1.0f, 1.0f);
    g_motor.pid_spd.out_min = MIN_IQ_CURRENT;
    g_motor.pid_spd.integral_min = MIN_IQ_CURRENT;
    
    g_motor.state = STATE_IDLE;
    g_motor.target_rpm = 0.0f;
}
