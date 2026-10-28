#ifndef CTRL_STATE_MACHINE_H
#define CTRL_STATE_MACHINE_H
#include "ctrl_observer.h"
#include "math_pid.h"

/* ============================================================
 * 电机主状态枚举
 * ============================================================ */
typedef enum {
    STATE_IDLE = 0,     // 停机
    STATE_ALIGN,        // 预定位 (可选)
    STATE_IF,           // 开环强拖
    STATE_RUN,          // 无感闭环
    STATE_BRAKE,        // 刹车
    STATE_FAULT         // 故障锁存
} MotorState_t;

/* ============================================================
 * 电机核心主句柄 (所有相关变量集中于此)
 * ============================================================ */
typedef struct {
    volatile MotorState_t state;
    Ctrl_Observer_t obs;    // 观测器句柄
    Math_PID_t pid_spd;     // 速度环
    Math_PID_t pid_id;      // D轴电流环
    Math_PID_t pid_iq;      // Q轴电流环

    // 电流与电压反馈 (物理量 A / V)
    float i_a, i_b, i_c;
    float i_alpha, i_beta;
    float i_d, i_q;
    float v_d, v_q;
    float v_alpha, v_beta;

    // 参考值
    float target_rpm;
    volatile float iq_ref;
    volatile float id_mtpa_ref;
    volatile float id_fw_ref;

    // IF 启动状态
    volatile float theta_if;
    volatile float freq_e;
    volatile uint8_t run_timer;

    // 故障标志
    uint8_t fault_code;
} Motor_Ctrl_t;

void Ctrl_StateMachine_Task(Motor_Ctrl_t *motor, float vbus, float dt);

#endif
