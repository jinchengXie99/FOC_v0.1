#ifndef CTRL_STATE_MACHINE_H
#define CTRL_STATE_MACHINE_H
#include "ctrl_observer.h"
#include "ctrl_pi.h"

/* 电机状态机状态；FAULT 为锁存态，ALIGN 当前保留但未实现。 */
typedef enum {
    STATE_IDLE = 0,     /* PWM 关闭，等待启动命令。 */
    STATE_ALIGN,        /* 转子预定位状态，目前未启用。 */
    STATE_IF,           /* 开环电角度/频率强拖启动。 */
    STATE_RUN,          /* 观测器角度驱动的无感闭环。 */
    STATE_BRAKE,        /* 有源制动，等待近零速或超时。 */
    STATE_FAULT         /* 故障锁存，PWM 关闭。 */
} MotorState_t;

/* 电机控制共享状态。volatile 成员会在中断与低频任务间共享。 */
typedef struct {
    volatile MotorState_t state;
    Ctrl_Observer_t obs;     /* 磁链观测器及 PLL 状态。 */
    Speed_PI_t   pi_speed;   /* 低频速度环，输出 q 轴电流参考 (A)。 */
    Current_PI_t pi_id;      /* d 轴电流 PI，输出 d 轴电压 (V)。 */
    Current_PI_t pi_iq;      /* q 轴电流 PI，输出 q 轴电压 (V)。 */

    /* 电流反馈单位 A；电压指令/反馈单位 V。 */
    volatile float i_a, i_b, i_c;
    volatile float i_alpha, i_beta;
    volatile float i_d, i_q;
    volatile float v_d, v_q;
    volatile float v_alpha, v_beta;

    /* 控制参考：target_rpm 为机械 RPM，其余电流参考单位 A。 */
    volatile float target_rpm;
    volatile float iq_ref;
    volatile float id_mtpa_ref;
    volatile float id_fw_ref;

    /* 开环强拖状态：theta_if 为电角度 rad，freq_e 为电频率 Hz；
     * direction 取 +1/-1，run_timer 按状态任务调用次数计数。 */
    volatile float theta_if;
    volatile float freq_e;
    volatile int8_t direction;
    volatile uint16_t run_timer;

    /* 当前锁存的故障码，编码定义见 safe_fault.h。 */
    volatile uint8_t fault_code;
} Motor_Ctrl_t;

/* 低频状态机任务；vbus (V) 与 dt (s) 暂保留为接口参数。 */
void Ctrl_StateMachine_Task(Motor_Ctrl_t *motor, float vbus, float dt);

#endif
