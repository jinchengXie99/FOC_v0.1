#include "ctrl_state_machine.h"
#include "hw_pwm.h"
#include "mtr_config.h"
#include <math.h>
/* 静态变量，记录上一次的状态，用于检测状态跳变 */
static MotorState_t last_state = STATE_IDLE;
/* ============================================================
 * 1kHz 状态机调度与处理
 * ============================================================ */
void Ctrl_StateMachine_Task(Motor_Ctrl_t *motor, float vbus, float dt) {
    MotorState_t current_state = motor->state;
    (void)vbus;
    (void)dt;

    switch (current_state) {
        case STATE_IDLE:
            HW_PWM_Disable();
            if (last_state != STATE_IDLE) {
                Speed_PI_Reset(&motor->pi_speed);
                Current_PI_Reset(&motor->pi_id);
                Current_PI_Reset(&motor->pi_iq);
                motor->freq_e = 0.0f;
                motor->theta_if = 0.0f;
                motor->run_timer = 0U;
            }
            motor->iq_ref = 0.0f;
            motor->id_mtpa_ref = 0.0f;
            motor->id_fw_ref = 0.0f;
            break;

        case STATE_IF:
            // IF angle and frequency are advanced only by the ADC-rate loop.
            if (last_state != STATE_IF) {
                motor->theta_if = 0.0f;
                motor->freq_e = 0.0f;
                motor->iq_ref = IF_START_IQ;
                motor->id_mtpa_ref = 0.0f;
                motor->id_fw_ref = 0.0f;
                motor->run_timer = 0U;
                Ctrl_Observer_Init(&motor->obs);
                Speed_PI_Reset(&motor->pi_speed);
                Current_PI_Reset(&motor->pi_id);
                Current_PI_Reset(&motor->pi_iq);
                HW_PWM_SetDuty(0.5f, 0.5f, 0.5f);
                HW_PWM_Enable();
            }
            break;

        case STATE_RUN:
            if (last_state != STATE_RUN) {
                HW_PWM_Enable();
                motor->run_timer = 0;
            }
            if (motor->run_timer < 1000U) {
                motor->run_timer++;
            }
            break;

        case STATE_BRAKE:
            if (last_state != STATE_BRAKE) {
                HW_PWM_Enable();
                Speed_PI_Reset(&motor->pi_speed);
                Current_PI_Reset(&motor->pi_id);
                Current_PI_Reset(&motor->pi_iq);
                motor->run_timer = 0U;
            }
            if (motor->run_timer < 1000U) {
                motor->run_timer++;
            }
            motor->iq_ref = motor->obs.speed_rpm > 0.0f ? -fabsf(BRAKE_CURRENT) :
                            (motor->obs.speed_rpm < 0.0f ? fabsf(BRAKE_CURRENT) : 0.0f);
            motor->id_mtpa_ref = 0.0f;

            if ((fabsf(motor->obs.speed_rpm) < 20.0f && motor->run_timer > 100U) ||
                motor->run_timer >= 1000U) {
                HW_PWM_Disable();
                motor->iq_ref = 0.0f;
                motor->state = STATE_IDLE;
            }
            break;

        case STATE_FAULT:
            HW_PWM_Disable();
            if (last_state != STATE_FAULT) {
                Speed_PI_Reset(&motor->pi_speed);
                Current_PI_Reset(&motor->pi_id);
                Current_PI_Reset(&motor->pi_iq);
                motor->freq_e = 0.0f;
                motor->theta_if = 0.0f;
            }
            motor->iq_ref = 0.0f;
            motor->id_mtpa_ref = 0.0f;
            motor->id_fw_ref = 0.0f;
            break;

        case STATE_ALIGN:
        default:
            HW_PWM_Disable();
            motor->iq_ref = 0.0f;
            motor->state = STATE_IDLE;
            break;
    }

    last_state = current_state;
}
