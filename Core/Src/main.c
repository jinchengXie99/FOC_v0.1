/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ============================================================
 * 紧急停止宏：原子化关断所有功率输出
 * ============================================================ */
#define FOC_EMERGENCY_STOP()                                       \
    do {                                                           \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);      \
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);      \
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);      \
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);      \
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);           \
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);           \
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);           \
        freq_e  = 0.0f;                                            \
        v_amp   = 0.0f;                                            \
        theta_e = 0.0f;                                            \
    } while (0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ============================================================
 * 1. 电机物理参数
 * ============================================================ */
#define MOTOR_RS            0.0513f/2.0f        /* 定子相电阻 (Ω) */
#define MOTOR_LS            0.0000328f/2.0f     /* 定子相电感 (H) */
#define MOTOR_POLE_PAIRS    1.0f                /* 极对数 */

/* --- 反电动势实测数据 --- */
#define MOTOR_VPP_LINE      10.1f              /* 线反电动势峰峰值 (V) */
#define MOTOR_F_E           410.2f             /* 反电动势测试频率 (Hz) */

/* --- 磁链计算常数 --- */
/* 推导过程： 1 / (4 * sqrt(3) * pi) 
 * 4 * 1.7320508 * 3.14159265 = 21.765592
 * 1 / 21.765592 ≈ 0.04594407 */
#define FLUX_CALC_CONST     (0.04594407f)

/* --- 永磁体磁链计算宏 --- */
/* 公式：Ψ_f = Vpp_line / (4 * sqrt(3) * pi * f_e) */
#define CALC_PM_FLUX(Vpp, f_e)  (((float)(Vpp) * FLUX_CALC_CONST) / ((float)(f_e) > 0.1f ? (float)(f_e) : 0.1f))

/* 最终参与 FOC 运算的磁链值 (Wb) */
#define MOTOR_FLUX_LINKAGE  CALC_PM_FLUX(MOTOR_VPP_LINE, MOTOR_F_E) 

/* ============================================================
 * 2. 系统控制参数
 * ============================================================ */
#define FOC_FREQ            20000.0f            /* FOC控制频率 20kHz */
#define FOC_DT              (1.0f / FOC_FREQ)   /* 控制周期 50us */

/* --- 启动与切入参数 --- */
#define IF_START_FREQ        50.0f               /* I/F 强拖目标频率 (Hz) */
#define IF_RAMP_RATE         15.0f               /* I/F 加速斜坡 (Hz/s) */
#define IF_START_IQ          2.0f                /* I/F 启动给定电流 (A) */

/* --- VESC 非线性磁链观测器参数 --- */
#define OBSERVER_GAIN       1200.0f              /* 观测器增益，典型 500~2000 */

/* --- VESC 电流环参数 (带宽法) --- */
#define CURRENT_BANDWIDTH   3000.0f              /* 电流环带宽 (rad/s) */
#define USE_DECOUPLING      1                   /* 1: 启用交叉耦合解耦, 0: 禁用 */

/* VESC 核心公式：根据带宽自动计算 PI 增益 (V/A) */
#define CURRENT_KP  (MOTOR_LS * CURRENT_BANDWIDTH)
#define CURRENT_KI  (MOTOR_RS * CURRENT_BANDWIDTH)

/* --- 速度环 PI 参数 --- */
#define SPD_KP              0.01f       
#define SPD_KI              0.0001f      
#define SPD_TARGET_RPM      30000.0f             /* 目标机械转速 (RPM) */

/* --- PLL 锁相环参数 (用于从磁链提取角度) --- */
#define PLL_KP              50.0f               
#define PLL_KI              3000.0f             

/* ============================================================
 * 3. 运行时状态变量
 * ============================================================ */
/* 电流变量 */
volatile float ia = 0.0f, ib = 0.0f, ic = 0.0f;
volatile float i_alpha = 0.0f, i_beta = 0.0f;
volatile float i_d = 0.0f, i_q = 0.0f;

/* 电压变量 */
volatile float v_d = 0.0f, v_q = 0.0f;
volatile float v_alpha_ref = 0.0f, v_beta_ref = 0.0f;

/* 角度与速度变量 */
volatile float theta_e = 0.0f;                  /* 当前输出电角度 */
volatile float theta_est = 0.0f;                /* 观测角度 */
volatile float theta_if = 0.0f;                 /* I/F强制角度 */
volatile float omega_e_est = 0.0f;              /* 估算电角速度 */
volatile float speed_rpm_est = 0.0f;            /* 估算机械转速 */
volatile float freq_e = 0.0f;                   /* I/F当前频率 */

float spd_target_rpm = SPD_TARGET_RPM;
volatile uint16_t run_timer = 0;                /* 切入闭环后的计时器 */

/* VESC 观测器内部磁链状态 */
static float obs_x1 = 0.0f;           
static float obs_x2 = 0.0f;           
static float pll_integral = 0.0f;      /* PLL 积分器 */

/* 控制器积分器状态 */
static float integral_spd = 0.0f;
static float integral_d = 0.0f;
static float integral_q = 0.0f;
static float iq_ref = 0.0f;

/* 控制状态机 */
typedef enum {
    STATE_IF = 0,       
    STATE_RUN = 1       
} MotorState_t;
volatile MotorState_t motor_state = STATE_IF;

/* ============================================================
 * 4. 硬件采样与保护参数
 * ============================================================ */
float ia_offset = 1971.0f; 
float ic_offset = 1991.0f;
#define CURRENT_SCALE       0.016113f          /* 电流采样换算系数 */

#define VBUS_SCALE          0.017947f          /* 母线电压换算系数 */
volatile float vbus_voltage = 24.0f; 
volatile float v_amp = 0.0f;

/* 故障阈值设定 */
#define OC_HARD_LIMIT_A     30.0f
#define OC_SOFT_LIMIT_A     18.0f
#define OC_DEBOUNCE_CNT     5
#define OVP_THRESHOLD       30.0f    
#define OVP_RECOVER         29.0f    
#define UVP_THRESHOLD       10.0f    
#define UVP_RECOVER         12.0f    

/* 故障代码定义 */
#define FAULT_NONE          0
#define FAULT_OC_HARD       1       
#define FAULT_OC_IA         2       
#define FAULT_OC_IB         3       
#define FAULT_OC_IC         4       
#define FAULT_OVP           5        
#define FAULT_UVP           6        

volatile uint8_t  g_fault_code   = FAULT_NONE;  
volatile uint8_t  g_fault_latch  = 0;           
volatile uint8_t  oc_debounce    = 0;           

/* ============================================================
 * 5. 串口通信 (VOFA+ JustFloat)
 * ============================================================ */
typedef struct {
    float ch_data[6];  
    uint8_t tail[4];   
} VOFA_Frame_t;

VOFA_Frame_t vofa_tx_frame = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0x00, 0x00, 0x80, 0x7F}};
volatile uint8_t vofa_tx_busy = 0; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  
  /* USER CODE BEGIN 2 */
    /* 1. ADC 校准 */
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    
    /* 2. 启动 1kHz 速度环定时器 */
    HAL_TIM_Base_Start_IT(&htim2);

    /* 3. 配置 ADC 注入触发点 (PWM 峰值处，下桥全开) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (htim1.Instance->ARR - 1)); 
    
    /* 4. 初始化 PWM 占空比为 0 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* 5. 启动 ADC 注入转换（等待触发） */
    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    /* 6. 启动 TIM1 四路 PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); 

    /* 7. 使能 EG2104 驱动 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
    
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    /* --- 故障恢复逻辑 --- */
    if (g_fault_latch)
    {
        uint8_t current_clear = (fabsf(ia) < 0.5f) && (fabsf(ib) < 0.5f) && (fabsf(ic) < 0.5f);
        uint8_t voltage_safe = (vbus_voltage < OVP_RECOVER) && (vbus_voltage > UVP_RECOVER);
        uint8_t key_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET);

        if (current_clear && voltage_safe && key_pressed)
        {
            g_fault_code  = FAULT_NONE;
            g_fault_latch = 0;
            oc_debounce   = 0;
            theta_e       = 0.0f;
            freq_e        = 0.0f;
            motor_state   = STATE_IF; 

            /* 复位观测器内部状态 */
            pll_integral = 0.0f;
            omega_e_est  = 0.0f;
            obs_x1       = 0.0f;
            obs_x2       = 0.0f;

            /* 重新使能驱动 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
            
            HAL_Delay(200); 
        }   
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief 定时器周期回调（1kHz） - 速度环与保护
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* ========================================================
         * 1. 速度环 PI 计算
         * ======================================================== */
        if (motor_state == STATE_RUN)
        {
             if (run_timer < 65535) run_timer++; 
            
            /* 阶段 A：纯电流闭环缓冲期 (500ms) */
            if (run_timer < 200) 
            {		
                iq_ref = IF_START_IQ; 
                integral_spd = IF_START_IQ; 
            }
             /* 阶段 B：速度环正式接管 */
            else
            {
                float err_spd = spd_target_rpm - speed_rpm_est;
                
                /* 1. 拦截异常与死区 */
                if (isnan(err_spd) || isinf(err_spd)) err_spd = 0.0f;
                if (err_spd > -20.0f && err_spd < 20.0f) err_spd = 0.0f; // 扩大死区，容许 20 RPM 的误差
                
                /* 2. 误差限幅 */
                if (err_spd > 1000.0f)  err_spd = 1000.0f;  
                if (err_spd < -1000.0f) err_spd = -1000.0f;
                
                /* 3. 积分累加与抗饱和 (Anti-Windup) */
                integral_spd += SPD_KI * err_spd;
                if (integral_spd > 15.0f)  integral_spd = 15.0f; // 积分上限压到 3A，防止过热
                if (integral_spd < -15.0f) integral_spd = -15.0f;
                
                /* 4. 计算目标电流 */
                float iq_target = SPD_KP * err_spd + integral_spd;
                
                /* 5. 输出限幅 */
                if (iq_target > 15.0f)  iq_target = 15.0f;
                if (iq_target < -15.0f) iq_target = -15.0f;
                
                /* 6. 直接赋值，取消缓慢斜坡（因为有了速度滤波，PI 本身就会平滑输出） */
								
                iq_ref = iq_target;
            }
        }
        else
        {
            iq_ref = IF_START_IQ;
            integral_spd = 0.0f; 
            run_timer = 0;
        }
				if (motor_state == STATE_IF) 
				{
						/* 强拖阶段，强制计算速度，但【绝对不要】清空 obs_x1 和 obs_x2！ */
						omega_e_est = 6.28318531f * freq_e; 
						speed_rpm_est = (omega_e_est / MOTOR_POLE_PAIRS) * (60.0f / 6.28318531f);
				}
				
        
       

        /* ========================================================
         * 2. 母线电压采样与保护
         * ======================================================== */
        uint16_t raw_vbus = 0;
        HAL_ADC_Start(&hadc2);
        raw_vbus = HAL_ADC_GetValue(&hadc2);
        float vbus_raw = (float)raw_vbus * VBUS_SCALE;
        vbus_voltage = vbus_voltage * 0.9f + vbus_raw * 0.1f;

        if (g_fault_latch == 0)
        {
            if (vbus_voltage > OVP_THRESHOLD || vbus_voltage < UVP_THRESHOLD)
            {
                FOC_EMERGENCY_STOP();
                g_fault_code = (vbus_voltage > OVP_THRESHOLD) ? FAULT_OVP : FAULT_UVP;
                g_fault_latch = 1;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
            }
        }

        /* ========================================================
         * 3. VOFA+ 串口波形上传 (带超时防死锁)
         * ======================================================== */
        static uint16_t uart_tx_timeout = 0;
        if (vofa_tx_busy == 0) 
        {
            vofa_tx_busy = 1;
            uart_tx_timeout = 0; 
            
            vofa_tx_frame.ch_data[0] = spd_target_rpm;
            vofa_tx_frame.ch_data[1] = speed_rpm_est;
            vofa_tx_frame.ch_data[2] = iq_ref;
            vofa_tx_frame.ch_data[3] = i_q;
            vofa_tx_frame.ch_data[4] = v_d; 
            vofa_tx_frame.ch_data[5] = v_q;
            
            if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&vofa_tx_frame, sizeof(VOFA_Frame_t)) != HAL_OK) {
                vofa_tx_busy = 0; 
            }
        }
        else
        {
            uart_tx_timeout++;
            if (uart_tx_timeout > 5) 
            {
                HAL_UART_AbortTransmit(&huart1);
                vofa_tx_busy = 0;
                uart_tx_timeout = 0;
            }
        }
    }
}

/**
  * @brief 串口 DMA 发送完成回调
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        vofa_tx_busy = 0;
    }
}

/**
  * @brief 注入转换完成回调（20kHz） - 核心 FOC 与 VESC 观测器
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2) return;
        
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); 
    
    /* ========================================================
     * 0. 电流采样与换算
     * ======================================================== */
    uint16_t raw_rank1 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);  
    uint16_t raw_rank2 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);  
        
    ia = ((float)raw_rank1 - ia_offset) * CURRENT_SCALE;
    ic = ((float)raw_rank2 - ic_offset) * CURRENT_SCALE;
    ib = -ia - ic;

    /* ========================================================
     * 1. 故障锁存检查
     * ======================================================== */
    if (g_fault_latch)
    {
        HAL_ADCEx_InjectedStart_IT(hadc);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); 
        return;
    }
    
    /* ========================================================
     * 2. 过流检测 (硬阈值 + 软阈值)
     * ======================================================== */
    float fabs_ia = fabsf(ia);
    float fabs_ib = fabsf(ib);
    float fabs_ic = fabsf(ic);
    float i_max   = fabs_ia;
    if (fabs_ib > i_max) i_max = fabs_ib;
    if (fabs_ic > i_max) i_max = fabs_ic;

    if (i_max > OC_HARD_LIMIT_A)
    {
        FOC_EMERGENCY_STOP();
        g_fault_code  = FAULT_OC_HARD;
        g_fault_latch = 1;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);   
        HAL_ADCEx_InjectedStart_IT(hadc);
        return;
    }

    if (i_max > OC_SOFT_LIMIT_A)
    {
        oc_debounce++;
        if (oc_debounce >= OC_DEBOUNCE_CNT)
        {
            FOC_EMERGENCY_STOP();
            if      (fabs_ia > OC_SOFT_LIMIT_A) g_fault_code = FAULT_OC_IA;
            else if (fabs_ib > OC_SOFT_LIMIT_A) g_fault_code = FAULT_OC_IB;
            else                                g_fault_code = FAULT_OC_IC;
            
            g_fault_latch = 1;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);  
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_ADCEx_InjectedStart_IT(hadc);
            return;
        }
    }
    else
    {
        if (oc_debounce > 0) oc_debounce--;
    }

    /* ========================================================
     * 3. Clarke & Park 变换
     * ======================================================== */
    /* 
     * Clarke 变换 (等幅值):
     * I_alpha = Ia
     * I_beta  = (Ia + 2*Ib) / sqrt(3)
     */
    float sin_e = sinf(theta_e);
    float cos_e = cosf(theta_e);

    i_alpha = ia;
    i_beta  = (ia + 2.0f * ib) * 0.57735027f; 

    /* 
     * Park 变换:
     * Id =  I_alpha * cos(e) + I_beta * sin(e)
     * Iq = -I_alpha * sin(e) + I_beta * cos(e)
     */
    i_d =  i_alpha * cos_e + i_beta * sin_e;
    i_q = -i_alpha * sin_e + i_beta * cos_e;

    /* ========================================================
     * 4. VESC 非线性磁链观测器 (Ortega-Praly)
     * ======================================================== */
    if (motor_state == STATE_IF) 
    {
        /* I/F 强拖阶段，强制角度与速度 */
        omega_e_est = 6.28318531f * freq_e; 
        speed_rpm_est = (omega_e_est / MOTOR_POLE_PAIRS) * (60.0f / 6.28318531f);
//        obs_x1 = 0.0f; 
//        obs_x2 = 0.0f;
    }
    else /* STATE_RUN */
    {
        /*
         * VESC 观测器核心公式：
         * 定义状态变量 X = [x1, x2]^T （定子磁链）
         * 误差 e = λ^2 - (x1 - Ls*I_alpha)^2 - (x2 - Ls*I_beta)^2
         * dx1/dt = V_alpha - Rs*I_alpha + γ * e * (x1 - Ls*I_alpha)
         * dx2/dt = V_beta  - Rs*I_beta  + γ * e * (x2 - Ls*I_beta)
         */
        float L_ia = MOTOR_LS * i_alpha;
        float L_ib = MOTOR_LS * i_beta;
        
        /* 1. 磁链误差计算 */
        float lambda_sq = MOTOR_FLUX_LINKAGE * MOTOR_FLUX_LINKAGE;
        float x1_ia = obs_x1 - L_ia;
        float x2_ib = obs_x2 - L_ib;
        float err = lambda_sq - (x1_ia * x1_ia + x2_ib * x2_ib);
        
        /* 2. 状态变量积分（增益自适应） */
        float gain = OBSERVER_GAIN;
        obs_x1 += (v_alpha_ref - MOTOR_RS * i_alpha + gain * err * x1_ia) * FOC_DT;
        obs_x2 += (v_beta_ref  - MOTOR_RS * i_beta  + gain * err * x2_ib) * FOC_DT;
        
        /* 3. 提取转子磁链 (去除电感项) */
        /* Ψ_alpha = x1 - Ls*I_alpha, Ψ_beta = x2 - Ls*I_beta */
        float flux_alpha = obs_x1 - L_ia;
        float flux_beta  = obs_x2 - L_ib;
        
        /* 4. PLL 锁相环提取速度与角度 */
        float phase_err = -flux_alpha * sin_e + flux_beta * cos_e;
        
        /* 【核心修复】将误差除以磁链幅值，归一化为角度差的正弦值 (范围 -1 ~ 1) 
           这样 PLL_KP=30 和 PLL_KI=1500 才能真正发挥出强大的追踪能力 */
        phase_err = phase_err / MOTOR_FLUX_LINKAGE;

        /* PLL 积分 */
        pll_integral += phase_err * PLL_KI * FOC_DT;
        if (pll_integral > 3000.0f)  pll_integral = 3000.0f;
        if (pll_integral < -3000.0f) pll_integral = -3000.0f;
        
        /* 估算角速度 */
        omega_e_est = pll_integral + phase_err * PLL_KP;
        
        /* 物理限幅与 NaN 保护 */
        if (isnan(omega_e_est) || isinf(omega_e_est)) omega_e_est = 0.0f;
//        if (omega_e_est > 6000.0f)  omega_e_est = 6000.0f;
//        if (omega_e_est < -6000.0f) omega_e_est = -6000.0f;
        
       speed_rpm_est = (omega_e_est / MOTOR_POLE_PAIRS) * (60.0f / 6.28318531f);

    }

    /* ========================================================
     * 5. 状态机切换与角度更新
     * ======================================================== */
    if (motor_state == STATE_IF)
    {
        /* 频率爬坡 */
        if (freq_e < IF_START_FREQ) {
            freq_e += IF_RAMP_RATE * FOC_DT;
        }
        
        /* 强制角度积分 */
        theta_if += 6.28318531f * freq_e * FOC_DT;
        if (theta_if >= 6.28318531f) theta_if -= 6.28318531f;
        
        theta_e = theta_if; 

                /* 切入条件判定 */
        if (freq_e >= IF_START_FREQ)
        {
            speed_rpm_est = (freq_e * 60.0f) / MOTOR_POLE_PAIRS;
            integral_spd  = IF_START_IQ; 
            
            /* 无缝接管：只同步 PLL 积分器与角度 */
            pll_integral = 6.28318531f * freq_e;
            theta_est    = theta_if; 
            
            /* 【核心修复】删掉以下所有强行赋初值的代码！
             * 让 obs_x1 和 obs_x2 保持原有积分状态，观测器会利用闭环的电压和电流自然收敛
             */
            // float flux_alpha_init = MOTOR_FLUX_LINKAGE * cosf(theta_if);
            // float flux_beta_init  = MOTOR_FLUX_LINKAGE * sinf(theta_if);
            // obs_x1 = flux_alpha_init + MOTOR_LS * i_alpha;
            // obs_x2 = flux_beta_init  + MOTOR_LS * i_beta;
            
            /* 重置电流环积分，防止电流冲击 */
            integral_d = v_d - CURRENT_KP * (0.0f - i_d);
            integral_q = v_q - CURRENT_KP * (iq_ref - i_q);
            
            run_timer    = 0; 
            motor_state  = STATE_RUN;
        }
    }
    else /* STATE_RUN */
    {
        /* 正常积分解算观测器角度 */
        theta_est += omega_e_est * FOC_DT;
        if (theta_est >= 6.28318531f) theta_est -= 6.28318531f;
        if (theta_est < 0.0f)         theta_est += 6.28318531f;
        
        theta_e = theta_est;
    }

    /* ========================================================
     * 6. VESC 电流环：PI + 解耦 + 电压圆限幅
     * ======================================================== */
    sin_e = sinf(theta_e);
    cos_e = cosf(theta_e);
    i_d =  i_alpha * cos_e + i_beta * sin_e;
    i_q = -i_alpha * sin_e + i_beta * cos_e;

    /* 误差计算 (Id_ref 设为 0) */
    float err_d = 0.0f - i_d; 
    float err_q = iq_ref - i_q;

    /* 1. PI 控制（使用带宽法增益） */
    v_d = CURRENT_KP * err_d + integral_d;
    v_q = CURRENT_KP * err_q + integral_q;

    /* 2. 交叉耦合解耦（VESC 默认启用） */
    #if USE_DECOUPLING
        /* 解耦公式：Vd -= ω * Ls * Iq, Vq += ω * Ls * Id */
        float decouple_d = -omega_e_est * MOTOR_LS * i_q;
        float decouple_q =  omega_e_est * MOTOR_LS * i_d;
        
        v_d += decouple_d;
        v_q += decouple_q;
    #endif

    /* 3. 积分器更新与抗饱和（电压圆限幅） */
    float v_mag = sqrtf(v_d * v_d + v_q * v_q);
    float v_limit = vbus_voltage * 0.57735f; /* SVPWM 线性区最大相电压 (Vbus / sqrt(3)) */
    
    if (v_mag > v_limit) 
    {
        /* 电压矢量饱和，按比例缩小输出 (圆限幅)，且停止积分累加 */
        float scale = v_limit / v_mag;
        v_d *= scale;
        v_q *= scale;
    }
    else
    {
        /* 未饱和时正常积分 */
        integral_d += CURRENT_KI * err_d * FOC_DT;
        integral_q += CURRENT_KI * err_q * FOC_DT;
    }
    
    /* 积分器独立限幅（防止深度饱和后恢复缓慢） */
    if (integral_d > v_limit)  integral_d = v_limit;
    if (integral_d < -v_limit) integral_d = -v_limit;
    if (integral_q > v_limit)  integral_q = v_limit;
    if (integral_q < -v_limit) integral_q = -v_limit;

    /* ========================================================
     * 7. 反 Park 变换 & SVPWM 零序注入
     * ======================================================== */
    /* 
     * 反 Park 变换:
     * V_alpha = Vd * cos(e) - Vq * sin(e)
     * V_beta  = Vd * sin(e) + Vq * cos(e)
     */
    v_alpha_ref = v_d * cos_e - v_q * sin_e;
    v_beta_ref  = v_d * sin_e + v_q * cos_e;

    /* 
     * 反 Clarke 变换 (得到三相参考):
     * Va = V_alpha
     * Vb = -0.5*V_alpha + sqrt(3)/2 * V_beta
     * Vc = -0.5*V_alpha - sqrt(3)/2 * V_beta
     */
    float va_ref = v_alpha_ref;
    float vb_ref = -0.5f * v_alpha_ref + 0.86602540f * v_beta_ref;
    float vc_ref = -0.5f * v_alpha_ref - 0.86602540f * v_beta_ref;

    /* 提取最大最小值进行零序注入 (等效 SVPWM) */
    float v_max_svpwm = va_ref, v_min_svpwm = va_ref;
    if (vb_ref > v_max_svpwm) v_max_svpwm = vb_ref;
    if (vc_ref > v_max_svpwm) v_max_svpwm = vc_ref;
    if (vb_ref < v_min_svpwm) v_min_svpwm = vb_ref;
    if (vc_ref < v_min_svpwm) v_min_svpwm = vc_ref;
    
    /* 零序注入公式：V_common = -(V_max + V_min) / 2 */
    float v_common = -(v_max_svpwm + v_min_svpwm) * 0.5f;
    va_ref += v_common;
    vb_ref += v_common;
    vc_ref += v_common;

    /* ========================================================
     * 8. 占空比计算与硬件寄存器更新
     * ======================================================== */
    float vbus_safe = vbus_voltage;
    if (vbus_safe < 10.0f) vbus_safe = 10.0f; /* 防止除零 */

    /* 占空比公式：duty = 0.5 + V_phase / Vbus */
    float duty_a = 0.5f + va_ref / vbus_safe;
    float duty_b = 0.5f + vb_ref / vbus_safe;
    float duty_c = 0.5f + vc_ref / vbus_safe;

    /* 占空比限幅 */
    if (duty_a < 0.0f) duty_a = 0.0f; else if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f; else if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f; else if (duty_c > 1.0f) duty_c = 1.0f;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a * (float)htim1.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_b * (float)htim1.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_c * (float)htim1.Instance->ARR));

    /* 重新武装 ADC，等待下一个 PWM 触发 */
    HAL_ADCEx_InjectedStart_IT(hadc);
    
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
