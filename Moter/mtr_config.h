#ifndef MTR_CONFIG_H
#define MTR_CONFIG_H

/* ============================================================
 * 电机物理参数 (严禁在此处定义变量，只允许宏定义)
 * ============================================================ */
#define MOTOR_POLE_PAIRS        7.0f        			/* 极对数 */
#define MOTOR_RS                0.3298f* 0.5f     /* 相电阻 (Ω) */
#define MOTOR_LD                0.0000829f* 0.5f  	/* D轴电感 (H) */
#define MOTOR_LQ                0.0000829f* 0.5f  	/* Q轴电感 (H) */
#define MOTOR_LS                ((MOTOR_LD + MOTOR_LQ)* 0.5f ) /* 观测器平均电感 */
//#define MOTOR_FLUX_LINKAGE      0.0027f     /* 永磁体磁链 (Wb) */
/* ============================================================
 *  永磁体磁链自动计算 (基于实测反电动势) 
 * ============================================================ */
/* --- 反电动势实测数据 --- */  
#define MOTOR_VPP_LINE          11.198f      /* 线反电动势峰峰值 (V) */
#define MOTOR_F_E               203.73f     /* 反电动势测试频率 (Hz) */
/* 磁链计算常数 推导过程： 1 / (4 * sqrt(3) * pi) = 4 * 1.7320508 * 3.14159265 = 21.765592
 * 1 / 21.765592 ≈ 0.04594407 */
#define FLUX_CALC_CONST         (0.04594407f)
/* 永磁体磁链计算公式：Ψ_f = Vpp_line / (4 * sqrt(3) * pi * f_e) */
#define CALC_PM_FLUX(Vpp, f_e)  (((float)(Vpp) * FLUX_CALC_CONST) / ((float)(f_e) > 0.1f ? (float)(f_e) : 0.1f))
#define MOTOR_FLUX_LINKAGE      CALC_PM_FLUX(MOTOR_VPP_LINE, MOTOR_F_E) 

/* ============================================================
 * 控制环路参数
 * ============================================================ */
#define CURRENT_BANDWIDTH        8000.0f    /* 电流环带宽 (rad/s) */
#define CURRENT_KP              (MOTOR_LS * CURRENT_BANDWIDTH)
#define CURRENT_KI              (MOTOR_RS * CURRENT_BANDWIDTH)

#define SPD_KP                  0.008f 	  /* 速度环 Kp (A/RPM)    0.00810*/
#define SPD_KI                  0.02f     /* 速度环 Ki (A/(RPM*s))0.02 */
#define MAX_IQ_CURRENT          8.0f       /* 正向电流上限 (A) */
#define MIN_IQ_CURRENT          -8.0f      /* 反向电流上限 (A) */

#define OBSERVER_GAIN           800.0f      /* VESC观测器增益 */
#define PLL_KP                  50.0f       /* PLL 比例 */
#define PLL_KI                  6000.0f     /* PLL 积分 */

#define IF_START_FREQ           10.0f       /* 强拖切入频率 (Hz) */
#define IF_RAMP_RATE            20.0f       /* 强拖频率爬坡 (Hz/s) */
#define IF_START_IQ             0.5f        /* 强拖启动电流 (A) */
#define BRAKE_CURRENT           0.5f        /* 制动电流 (A) */

#define FW_VOLTAGE_THRESHOLD    0.93f       /* 弱磁触发电压利用率 */
#define FW_MAX_CURRENT          3.0f        /* 最大弱磁电流 */
#define FW_RAMP_STEP            0.01f       /* 弱磁爬坡步长 */

#endif /* MTR_CONFIG_H */
