#include "ctrl_current_loop.h"
#include "ctrl_pi.h"
#include "math_svpwm.h"
#include "hw_pwm.h"
#include "hw_dsp.h"
#include "drv_config.h"
#include "mtr_config.h"
#include "safe_fault.h"
#include <math.h>

/**
 * @brief 执行一次同步旋转坐标系 dq 电流环。
 *
 * 流程为：检查母线电压、Park 变换、计算 d/q 电压 PI、加入交叉耦合
 * 解耦、对 dq 电压矢量限幅、反 Park 变换并生成 SVPWM 占空比。
 * 本函数在 ADC 注入转换完成中断中调用，通常以 20 kHz 运行。
 *
 * @param motor 电机状态、反馈量、参考量和控制器
 * @param vbus 当前直流母线电压 (V)
 * @param dt 电流环采样周期 (s)
 */
void Ctrl_CurrentLoop_Execute(Motor_Ctrl_t *motor, float vbus, float dt) {
    /* 电压无效或低于欠压门限时，不继续计算控制量，直接锁故障并关断功率级。 */
    if (vbus < DRV_UVP_THRESHOLD || isnan(vbus) || isinf(vbus)) {
        motor->fault_code = FAULT_UVP;
        motor->state = STATE_FAULT;
        HW_PWM_Disable();
        return;
    }

    /*
     * SVPWM 线性区的 dq 电压矢量幅值上限为 Vbus/sqrt(3)。
     * 先按实时母线电压更新两个 PI 的输出上下限，再进行后续矢量限幅。
     */
    float v_limit = vbus * 0.57735027f;
    motor->pi_id.out_max = v_limit;
    motor->pi_id.out_min = -v_limit;
    motor->pi_iq.out_max = v_limit;
    motor->pi_iq.out_min = -v_limit;

    /* 使用当前估计电角度计算 Park/反 Park 所需的正余弦。 */
    float sin_e, cos_e;
    cordic_sincos(motor->obs.theta_est, &sin_e, &cos_e);

    /* Park 变换：将静止坐标系电流投影到转子磁场 d/q 轴。 */
    float i_d_meas =  motor->i_alpha * cos_e + motor->i_beta * sin_e;
    float i_q_meas = -motor->i_alpha * sin_e + motor->i_beta * cos_e;

    motor->i_d = i_d_meas;
    motor->i_q = i_q_meas;

    /*
     * MTPA 和弱磁都可能给出负 d 轴电流。两者同时存在时选取
     * 绝对值较大的负向参考，确保弱磁需求不会被 MTPA 覆盖。
     */
    float id_target = motor->id_mtpa_ref; 
    if (fabsf(motor->id_fw_ref) > fabsf(id_target)) {
        id_target = motor->id_fw_ref; 
    }

    /* d、q 轴误差单位均为 A。 */
    float err_d = id_target - i_d_meas;
    float err_q = motor->iq_ref - i_q_meas;

    /* PI 根据电流误差生成电压指令，输出单位为 V。 */
    float v_d_cmd = Current_PI_Calc(&motor->pi_id, id_target, i_d_meas, dt);
    float v_q_cmd = Current_PI_Calc(&motor->pi_iq, motor->iq_ref, i_q_meas, dt);

    #if USE_DECOUPLING
        /*
         * 前馈解耦补偿：抵消 dq 坐标旋转产生的交叉耦合电压，
         * 并补偿永磁体反电动势；omega 为电角速度 (rad/s)。
         */
        v_d_cmd -= motor->obs.omega_est * MOTOR_LS * i_q_meas;
        v_q_cmd += motor->obs.omega_est * MOTOR_LS * i_d_meas;
        v_q_cmd += motor->obs.omega_est * MOTOR_FLUX_LINKAGE; 
    #endif

    /*
     * 对 dq 电压矢量做圆形限幅。先比较模长平方，未饱和时避免
     * 执行平方根；只有发生饱和时才求实际模长并同比缩放 d/q 分量。
     */
    float v_mag_sq = v_d_cmd * v_d_cmd + v_q_cmd * v_q_cmd;
    
    if (v_mag_sq > v_limit * v_limit) {
        float scale = v_limit / HW_DSP_Sqrt(v_mag_sq);
        v_d_cmd *= scale;
        v_q_cmd *= scale;
        
        /*
         * 电压饱和时回算积分状态，避免 PI 内部积分继续把输出推向饱和；
         * 回算后再限制积分量，防止母线电压变化时保留过大的积分值。
         */
        motor->pi_id.integral = v_d_cmd - (motor->pi_id.kp * err_d);
        motor->pi_iq.integral = v_q_cmd - (motor->pi_iq.kp * err_q);
        
        // 防止积分器越界
        if (motor->pi_id.integral > v_limit) motor->pi_id.integral = v_limit;
        if (motor->pi_id.integral < -v_limit) motor->pi_id.integral = -v_limit;
        if (motor->pi_iq.integral > v_limit) motor->pi_iq.integral = v_limit;
        if (motor->pi_iq.integral < -v_limit) motor->pi_iq.integral = -v_limit;
    }

    motor->v_d = v_d_cmd;
    motor->v_q = v_q_cmd;
    motor->v_alpha = v_d_cmd * cos_e - v_q_cmd * sin_e;
    motor->v_beta = v_d_cmd * sin_e + v_q_cmd * cos_e;

    /* 将 dq 电压反变换至 alpha-beta，供观测器和 SVPWM 共用。 */
    float duty_a, duty_b, duty_c;

    /* 由静止坐标系电压直接生成三相归一化占空比。 */
    Math_SVPWM_Gen(motor->v_alpha, motor->v_beta, vbus,
                   &duty_a, &duty_b, &duty_c);

    /* 写入 TIM1 三相比较寄存器；底层函数会再次将占空比限制到 [0,1]。 */
    HW_PWM_SetDuty(duty_a, duty_b, duty_c);
}
