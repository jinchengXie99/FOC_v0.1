#include "ctrl_state_machine.h"
#include "hw_pwm.h"
#include "mtr_config.h"
#include "safe_fault.h"
#include <math.h>
/* 上一调度周期的状态，用于只在进入新状态时执行一次初始化动作。 */
static MotorState_t last_state = STATE_IDLE;
/**
 * @brief 以低频任务节拍执行电机状态机。
 *
 * 负责各状态的进入动作、PWM 使能管理、制动电流参考和超时处理。
 * IF 电角度/频率等高速变量由 ADC 电流环更新，不在此任务推进。
 *
 * @param motor 电机控制状态
 * @param vbus 母线电压 (V)，当前状态机实现未直接使用
 * @param dt 状态机任务周期 (s)，当前计时使用调用次数而非 dt
 */
void Ctrl_StateMachine_Task(Motor_Ctrl_t *motor, float vbus, float dt) {
    MotorState_t current_state = motor->state;
    (void)vbus;
    (void)dt;

    switch (current_state) {
        case STATE_IDLE:
            /* 停机状态始终关闭 PWM；离开运行态后清理控制器及 IF 状态。 */
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
            /* IF 角度和频率由 ADC 频率的高速控制路径推进。 */
            if (last_state != STATE_IF) {
                motor->theta_if = 0.0f;
                motor->freq_e = 0.0f;
                motor->direction = motor->direction < 0 ? -1 : 1;
                motor->iq_ref = (float)motor->direction * IF_START_IQ;
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
            /* 闭环运行期间累计毫秒级计数，供上层启动/运行逻辑使用。 */
            if (last_state != STATE_RUN) {
                HW_PWM_Enable();
                motor->run_timer = 0;
            }
            if (motor->run_timer < 1000U) {
                motor->run_timer++;
            }
            break;

        case STATE_BRAKE:
            /* 反向 q 轴电流产生制动力矩；低速稳定后停车，超时则锁故障。 */
            if (last_state != STATE_BRAKE) {
                HW_PWM_Enable();
                Speed_PI_Reset(&motor->pi_speed);
                Current_PI_Reset(&motor->pi_id);
                Current_PI_Reset(&motor->pi_iq);
                motor->run_timer = 0U;
            }
            if (motor->run_timer < BRAKE_TIMEOUT_MS) {
                motor->run_timer++;
            }
            if (fabsf(motor->obs.speed_rpm) > 20.0f) {
                motor->direction = motor->obs.speed_rpm > 0.0f ? 1 : -1;
            }
            motor->iq_ref = -(float)motor->direction * fabsf(BRAKE_CURRENT);
            motor->id_mtpa_ref = 0.0f;

            if (fabsf(motor->obs.speed_rpm) < 20.0f && motor->run_timer > 100U) {
                HW_PWM_Disable();
                motor->iq_ref = 0.0f;
                motor->state = STATE_IDLE;
            } else if (motor->run_timer >= BRAKE_TIMEOUT_MS) {
                HW_PWM_Disable();
                motor->fault_code = FAULT_BRAKE_TIMEOUT;
                motor->state = STATE_FAULT;
            }
            break;

        case STATE_FAULT:
            /* 故障锁存期间关闭 PWM 并清零电流参考，等待外部故障恢复流程。 */
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
            /* ALIGN 暂未实现，未知状态也按安全停机处理并回到 IDLE。 */
            HW_PWM_Disable();
            motor->iq_ref = 0.0f;
            motor->state = STATE_IDLE;
            break;
    }

    last_state = current_state;
}
