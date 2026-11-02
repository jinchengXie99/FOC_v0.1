#ifndef MTR_CONFIG_H
#define MTR_CONFIG_H

/* ============================================================
 * 电机物理参数 (严禁在此处定义变量，只允许宏定义)
 * ============================================================ */
#define MOTOR_POLE_PAIRS        7.0f        			/* 电机极对数，用于电角度/机械转速换算。 */
#define MOTOR_RS                0.3298f* 0.5f     /* 定子相电阻 (Ω)，按当前标定折算。 */
#define MOTOR_LD                0.0000829f* 0.5f  	/* d 轴电感 (H)。 */
#define MOTOR_LQ                0.0000829f* 0.5f  	/* q 轴电感 (H)。 */
#define MOTOR_LS                ((MOTOR_LD + MOTOR_LQ)* 0.5f ) /* 观测器使用的 d/q 平均电感 (H)。 */
//#define MOTOR_FLUX_LINKAGE      0.0027f     /* 永磁体磁链 (Wb) */
/* ============================================================
 *  永磁体磁链自动计算 (基于实测反电动势) 
 * ============================================================ */
/* --- 反电动势实测数据 --- */  
#define MOTOR_VPP_LINE          11.198f      /* 线反电动势峰峰值 (V) */
#define MOTOR_F_E               203.73f     /* 反电动势测试频率 (Hz) */
/* 反电动势换算常数：4*sqrt(3)*pi ≈ 21.765592，其倒数约为 0.04594407。 */
#define FLUX_CALC_CONST         (0.04594407f)
/* 由线电压峰峰值和电频率估算永磁体磁链 Ψf (Wb)。 */
#define CALC_PM_FLUX(Vpp, f_e)  (((float)(Vpp) * FLUX_CALC_CONST) / ((float)(f_e) > 0.1f ? (float)(f_e) : 0.1f))
#define MOTOR_FLUX_LINKAGE      CALC_PM_FLUX(MOTOR_VPP_LINE, MOTOR_F_E) 

/* ============================================================
 * 控制环路参数
 * ============================================================ */
#define CURRENT_BANDWIDTH        8000.0f    /* 电流环设计带宽 (rad/s)，用于计算 PI 系数。 */
#define CURRENT_KP              (MOTOR_LS * CURRENT_BANDWIDTH)
#define CURRENT_KI              (MOTOR_RS * CURRENT_BANDWIDTH)

#define SPD_KP                  0.008f 	  /* 速度环 Kp (A/RPM)    0.00810*/
#define SPD_KI                  0.02f     /* 速度环 Ki (A/(RPM*s))0.02 */
#define MAX_IQ_CURRENT          8.0f       /* 正向电流上限 (A) */
#define MIN_IQ_CURRENT          -8.0f      /* 反向电流上限 (A) */

#define OBSERVER_GAIN           800.0f      /* 磁链观测器非线性误差反馈增益。 */
#define PLL_KP                  50.0f       /* PLL 相位误差比例增益。 */
#define PLL_KI                  6000.0f     /* PLL 相位误差积分增益。 */

#define IF_START_FREQ           0.0f       /* IF 强拖结束并切入闭环的电频率门限 (Hz)。 */
#define IF_RAMP_RATE            20.0f       /* 强拖频率爬坡 (Hz/s) */
#define IF_START_IQ             0.5f        /* 强拖启动电流 (A) */
#define BRAKE_CURRENT           0.5f        /* 制动电流 (A) */
#define BRAKE_TIMEOUT_MS        3000U       /* 制动超时后关断并锁定故障 */

#define FW_VOLTAGE_THRESHOLD    0.93f       /* 弱磁触发电压利用率 */
#define FW_MAX_CURRENT          3.0f        /* 最大弱磁电流 */
#define FW_RAMP_STEP            0.01f       /* 每次弱磁算法调用允许的 d 轴电流参考变化量 (A)。 */

#endif /* MTR_CONFIG_H */
