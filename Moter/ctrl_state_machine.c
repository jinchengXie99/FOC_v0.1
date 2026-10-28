#include "ctrl_state_machine.h"
#include "hw_pwm.h"
#include "mtr_config.h"
#include <math.h>

/* ============================================================
 * 1kHz 状态机调度与处理
 * ============================================================ */
void Ctrl_StateMachine_Task(Motor_Ctrl_t *motor, float vbus, float dt) {
    (void)vbus;
    (void)dt;

    switch (motor->state) {
        case STATE_IDLE:
            // 停机状态，PWM关闭，积分清零
            HW_PWM_Disable();
            Math_PID_Reset(&motor->pid_spd);
            Math_PID_Reset(&motor->pid_id);
            Math_PID_Reset(&motor->pid_iq);
            motor->freq_e = 0.0f;
            motor->theta_if = 0.0f;
            motor->run_timer = 0U;
            motor->iq_ref = 0.0f;
            motor->id_mtpa_ref = 0.0f;
            motor->id_fw_ref = 0.0f;
            break;

        case STATE_IF:
            // 开环强拖阶段
            HW_PWM_Enable();
            motor->iq_ref = IF_START_IQ;
            motor->id_mtpa_ref = 0.0f;
            break;

        case STATE_RUN:
            // 闭环运行阶段
            HW_PWM_Enable();
            // 速度环在 1kHz 中独立调度，此处仅处理 MTPA/弱磁等慢速策略
            break;

        case STATE_BRAKE:
            HW_PWM_Enable();
            motor->iq_ref = -fabsf(BRAKE_CURRENT);
            motor->id_mtpa_ref = 0.0f;
            Math_PID_Reset(&motor->pid_spd); // 断开速度环
            break;

        case STATE_FAULT:
            // 硬件级锁死
            HW_PWM_Disable();
            motor->freq_e = 0.0f;
            motor->theta_if = 0.0f;
            break;
    }
}
