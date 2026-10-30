#ifndef DRV_CONFIG_H
#define DRV_CONFIG_H

/* ============================================================
 * 硬件版本与系统时钟配置
 * ============================================================ */
#define DRV_HW_VERSION          "HS_V2.0"       /* 浩盛单路电机开发板 V2.0 */
#define DRV_MCU_CLOCK           170000000UL     /* 系统主频 170MHz */

/* ============================================================
 * PWM 驱动配置 (TIM1 - 20kHz 中心对齐)
 * ============================================================ */
#define DRV_PWM_FREQ_HZ         20000.0f        /* PWM 控制频率 20kHz */
#define DRV_PWM_ARR             4249            /* 自动重装载值 (170M / (2 * 20k) = 4250) */
#define DRV_PWM_DEADTIME_NS     0               /* TIM1 内部死区补偿，由于 EG2104 自带硬件死区，这里为 0 */

/* ============================================================
 * ADC 电流采样配置 (注入组 - 双电阻采样)
 * 硬件：EG2104 下桥 + 5mΩ 采样电阻 + COS722MR 运放(10倍) + 1.65V 偏置
 * ============================================================ */
#define DRV_ADC_VREF            3.3f            /* ADC 参考电压 */
#define DRV_ADC_RESOLUTION      4096.0f         /* 12位 ADC 分辨率 */

#define DRV_SHUNT_RESISTOR      0.005f          /* 采样电阻 5mΩ (R28, R29) */
#define DRV_OPAMP_GAIN          10.0f           /* COS722MR 运放增益 10倍 */

/* 
 * 电流换算系数推导：
 * 分辨率 = Vref / ADC_Resolution / (Rshunt * Gain)
 * LSB = 3.3 / 4096 / (0.005 * 10) ≈ 0.016113 A/LSB
 * 实际使用中，建议配合标定代码动态获取偏置值，此宏仅作理论参考
 */
#define DRV_CURRENT_SCALE       (DRV_ADC_VREF / DRV_ADC_RESOLUTION / (DRV_SHUNT_RESISTOR * DRV_OPAMP_GAIN))
#define DRV_CURRENT_OFFSET_NOM  2048.0f         /* 1.65V 对应的零点理论 ADC 值 */

/* ============================================================
 * ADC 母线电压采样配置 (规则组)
 * 硬件：R83(100k) / R56(4.7k) 分压，D15(TVS) 钳位
 * ============================================================ */
#define DRV_VBUS_R_TOP          100000.0f       /* R83 阻值 (Ω) */
#define DRV_VBUS_R_BOT          4700.0f         /* R56 阻值 (Ω) */

/* 
 * 母线电压换算系数推导：
 * 分压比 = R_BOT / (R_TOP + R_BOT) = 4.7k / 104.7k ≈ 0.04489
 * Vbus_SCALE = (3.3 / 4096) / 0.04489 ≈ 0.017947 V/LSB
 */
#define DRV_VBUS_SCALE          ((DRV_ADC_VREF / DRV_ADC_RESOLUTION) / (DRV_VBUS_R_BOT / (DRV_VBUS_R_TOP + DRV_VBUS_R_BOT)))

/* ============================================================
 * 滑动变阻器 (调速器) 配置 (规则组)
 * ============================================================ */
#define DRV_THROTTLE_SCALE      (DRV_ADC_VREF / DRV_ADC_RESOLUTION) /* 调速器电压换算 V/LSB */

/* ============================================================
 * 硬件保护与限幅阈值 
 * ============================================================ */
#define DRV_OC_HARD_LIMIT       25.0f           /* 硬件级过流阈值 (A) - 立即关断 */
#define DRV_OC_SOFT_LIMIT       20.0f           /* 软件级过流阈值 (A) - 连续触发关断 */
#define DRV_OC_DEBOUNCE_CNT     5               /* 软过流消抖次数 (20kHz下 = 250us) */

#define DRV_OVP_THRESHOLD       40.0f           /* 过压保护阈值 (V) (TVS 40V动作，留余量) */
#define DRV_OVP_RECOVER         35.0f           /* 过压恢复阈值 (V) */
#define DRV_UVP_THRESHOLD       10.0f           /* 欠压保护阈值 (V) */
#define DRV_UVP_RECOVER         12.0f           /* 欠压恢复阈值 (V) */

/* ============================================================
 * 引脚与控制宏定义 (与 CubeMX Pinout 严格对应)
 * ============================================================ */
/* EG2104 使能引脚 (SD) */
#define DRV_SD1_PORT            GPIOB
#define DRV_SD1_PIN             GPIO_PIN_0
#define DRV_SD2_PORT            GPIOA
#define DRV_SD2_PIN             GPIO_PIN_1
#define DRV_SD3_PORT            GPIOA
#define DRV_SD3_PIN             GPIO_PIN_2
#define DRV_SD4_PORT            GPIOA
#define DRV_SD4_PIN             GPIO_PIN_0

/* 状态 LED */
#define DRV_LED_R_PORT          GPIOC
#define DRV_LED_R_PIN           GPIO_PIN_15
#define DRV_LED_G_PORT          GPIOC
#define DRV_LED_G_PIN           GPIO_PIN_14
#define DRV_LED_B_PORT          GPIOC
#define DRV_LED_B_PIN           GPIO_PIN_13

/* 按键 */
#define DRV_KEY1_PORT           GPIOC
#define DRV_KEY1_PIN            GPIO_PIN_9
#define DRV_KEY2_PORT           GPIOB
#define DRV_KEY2_PIN            GPIO_PIN_12
/* ============================================================
 * 通信配置
 * ============================================================ */
#define DRV_UART_BAUDRATE       2000000UL        /* VOFA+ 调试串口波特率 */

#endif /* DRV_CONFIG_H */
