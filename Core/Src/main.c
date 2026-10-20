/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
#include <math.h>       /* 提供 sinf()、sqrtf() 等浮点数学函数 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ============================================================
 * 紧急停止宏：原子化关断所有功率输出
 * ============================================================
 * 动作序列：
 *   1. 拉低所有 EG2104 的 SD 引脚（硬件级关断，几十 ns 生效）
 *   2. 三相 PWM 占空比清零（上桥全关、下桥全开，安全态）
 *   3. 清除 V/F 状态，防止故障清除后残留角度/幅值导致冲击
 * 注意：SD 拉低是首选动作，比清 CCR 更快更可靠。
 * ============================================================ */
#define FOC_EMERGENCY_STOP()                                       \
    do {                                                           \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);      /* SD1 */ \
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);      /* SD2 */ \
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);      /* SD3 */ \
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);      /* SD4 */ \
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
 * I/F 开环控制参数（频率开环，电流闭环）
 * ============================================================ */

/* 目标电频率（Hz）：决定了强制旋转磁场的速度 */
#define TARGET_FREQ_E       60.0f

/* 频率斜坡速率（Hz/s）：防止加速过快导致失步 */
#define RAMP_RATE           15.0f

/* 目标力矩电流 Iq (A)：决定了电机出力大小。
 * 调试时从 1A~2A 开始，带载能力不足再往上加，注意不要超电机额定电流 */
#define IF_IQ_REF           1.5f

/* 目标励磁电流 Id (A)：一般设为 0 */
#define IF_ID_REF           0.0f

/* 电流环 PI 参数（初始推荐值，实际需根据电机 Ls、Rs 微调）
 * 这里的单位是 伏特 / 安培 */
#define PI_KP               1.0f
#define PI_KI               0.001f

/* 电压限幅：防止积分饱和和过调制 ( Vbus / √3 ) */
#define V_OUT_MAX           12.0f 
/* ============================================================
 * V/F 开环控制参数（按需调整）
 * ============================================================ */

/* 电机极对数：暂定 7 对极，后续实测修正 */
#define POLE_PAIRS          7.0f

/* FOC 控制频率 = PWM 频率 = 20kHz，周期 50μs */
#define FOC_FREQ            20000.0f
#define FOC_DT              (1.0f / FOC_FREQ)

/* V/F 比：每 1Hz 电频率对应的相电压幅值（V/Hz）
 * 依据电机额定电压与额定频率确定。
 * 例如：额定电压 24V，额定电频率 100Hz → VF_RATIO ≈ 0.24
 * 首次调试建议从小值开始，观察电流后再加大 */
#define VF_RATIO            0.5f

/* 目标电频率（Hz）：电机实际机械转速 = freq_e / POLE_PAIRS / 60 */
//#define TARGET_FREQ_E       30.0f

/* 频率斜坡速率（Hz/s）：防止直接跳频导致电流冲击 */
//#define RAMP_RATE           20.0f

/* 母线电压（V）：若启用 ADC 母线采样，可在运行时动态刷新 */
#define VBUS_NOMINAL        24.0f

/* SPWM 线性区最大相电压幅值 = Vbus / √3，再乘 0.95 留余量 */
#define V_PHASE_MAX         (VBUS_NOMINAL * 0.5773f * 0.95f)

/* 电流采样换算系数：I = (raw - offset) × CURRENT_SCALE
 * 推导：3.3V / 4096 / (5mΩ × 10) ≈ 0.01611 A/LSB */
#define CURRENT_SCALE       0.016113f

/* ADC 零点偏置（1.65V 对应 2048）：后续标定后替换 */
#define IA_OFFSET_DEFAULT   	1991.0f
#define IC_OFFSET_DEFAULT    1971.0f

/* ============================================================
 * 运行时状态变量
 * ============================================================ */

/* 电流采样 */
volatile float ia = 0.0f;       /* A 相电流 (A) */
volatile float ib = 0.0f;       /* B 相电流 (A) */
volatile float ic = 0.0f;       /* C 相电流 (A) */
/* 坐标变换中间变量 (方便 VOFA+ 观察) */
volatile float i_alpha = 0.0f, i_beta = 0.0f;
volatile float i_d = 0.0f, i_q = 0.0f;
volatile float v_d = 0.0f, v_q = 0.0f;
volatile float v_alpha = 0.0f, v_beta = 0.0f;
/* I/F 状态 */
volatile float theta_e = 0.0f;  /* 强制电角度 */
volatile float freq_e  = 0.0f;  /* 当前电频率 */

/* PI 积分累加项 */
static float integral_d = 0.0f;
static float integral_q = 0.0f;
/* V/F 状态 */
//volatile float theta_e = 0.0f;  /* 当前电角度 (rad)，0~2π */
//volatile float freq_e  = 0.0f;  /* 当前电频率 (Hz) */
volatile float v_amp   = 0.0f;  /* 当前相电压幅值 (V) */

/* 偏置（后续标定后更新） */
float ia_offset = IA_OFFSET_DEFAULT;
float ic_offset = IC_OFFSET_DEFAULT;

/* ============================================================
 * 过流保护参数（按电机额定电流调整，务必保守）
 * ============================================================
 * 参考：硬件理论可测 ±33A（5mΩ × 增益 10，1.65V 偏置）
 * 建议软阈值 = 电机额定电流 × 1.5，硬阈值 = 电机额定电流 × 2.5
 * ============================================================ */

/* 硬阈值：任一相瞬时超过此值，立即切断，无消抖 */
#define OC_HARD_LIMIT_A     15.0f

/* 软阈值：任一相超过此值连续 OC_DEBOUNCE_CNT 次才触发 */
#define OC_SOFT_LIMIT_A     8.0f
#define OC_DEBOUNCE_CNT     5

/* 故障状态定义 */
#define FAULT_NONE          0
#define FAULT_OC_HARD       1       /* 硬阈值过流 */
#define FAULT_OC_IA         2       /* A 相软过流 */
#define FAULT_OC_IB         3       /* B 相软过流 */
#define FAULT_OC_IC         4       /* C 相软过流 */
#define FAULT_OVP           5        /* 过压故障 */
#define FAULT_UVP           6        /* 欠压故障 */

/* 运行时故障状态（volatile 保证中断与主循环可见性） */
volatile uint8_t  g_fault_code   = FAULT_NONE;  /* 故障码 */
volatile uint8_t  g_fault_latch  = 0;           /* 1 = 已锁存，禁止输出 */
volatile uint8_t  oc_debounce    = 0;           /* 软阈值消抖计数器 */

/* -------- 母线电压采样与保护参数 -------- */
/* 计算推导：3.3V / 4096 / (4.7 / (100 + 4.7)) ≈ 0.017947 */
#define VBUS_SCALE          0.017947f 

/* 保护阈值与回差（务必根据实际电源和电机额定电压调整） */
#define OVP_THRESHOLD       30.0f    /* 过压阈值 (V) - 考虑到TVS在40V动作，设36V较安全 */
#define OVP_RECOVER         29.0f    /* 过压恢复阈值 (V) */
#define UVP_THRESHOLD       10.0f    /* 欠压阈值 (V) - 低于此值可能失控 */
#define UVP_RECOVER         12.0f    /* 欠压恢复阈值 (V) */


/* 运行时母线电压变量 */
volatile float vbus_voltage = 24.0f; /* 上电初值假设为 24V，防止上电瞬间误报欠压 */

/*---------- 定义串口数据jastFloat 协议帧 ------------*/
// 定义一个完整的帧结构，利用结构体保证内存布局紧凑
typedef struct {
    float ch_data[5];  // 发送 Ia, Ib, Ic 3个通道
    uint8_t tail[4];   // 填充固定帧尾 0x00, 0x00, 0x80, 0x7F
} VOFA_Frame_t;

VOFA_Frame_t vofa_tx_frame = {{0.0f, 0.0f, 0.0f}, {0x00, 0x00, 0x80, 0x7F}};

volatile uint8_t vofa_tx_busy = 0; // DMA 发送忙标志

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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
		// 1. ADC2 校准
	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
			Error_Handler();
	}
	/* --- 启动 TIM2 1kHz 中断（用于过欠压保护和速度环） --- */
		HAL_TIM_Base_Start_IT(&htim2);

	// 2. 设置 TIM1 CH4 比较值，用于触发 ADC
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (htim1.Instance->ARR - 1)); // ARR - 1

	// 3. 设置三相 PWM 初始占空比为 0（或 50% 视情况）
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	// 4. 启动 ADC2 注入转换（等待触发）
	if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
			Error_Handler();
	}

	// 5. 启动 TIM1 所有通道 PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // 用于触发 ADC

	// 6. 使能 EG2104 SD 引脚（根据电机类型）
	// 三相电机：使能 SD1, SD2, SD3；步进电机：再使能 SD4
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // SD1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // SD2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // SD3
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // SD4 步进时用
		HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
	
		/* ============================================================
     * 故障恢复：按下 KEY1 且故障已清除时恢复运行
     * ============================================================ */
    if (g_fault_latch)
		{
			/* 检查电流是否归零 */
			uint8_t current_clear = (fabsf(ia) < 0.5f) && (fabsf(ib) < 0.5f) && (fabsf(ic) < 0.5f);
			
			/* 检查电压是否回到安全范围（带回差，防止在临界点反复跳变） */
			uint8_t voltage_safe = (vbus_voltage < OVP_RECOVER) && (vbus_voltage > UVP_RECOVER);

		 	uint8_t key_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET);

			if (current_clear && voltage_safe && key_pressed)
				{
						g_fault_code  = FAULT_NONE;
						g_fault_latch = 0;
						oc_debounce   = 0;
						theta_e       = 0.0f;
						freq_e        = 0.0f;
						v_amp         = 0.0f;

						/* 重新使能 EG2104 */
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

						/* 恢复绿灯 */
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
						
						HAL_Delay(200); /* 按键消抖 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief 定时器周期回调（1kHz）
  * @note  在此处理母线电压读取、过欠压保护及速度环
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* ========================================================
         * 第 1 步：软件触发 ADC2 规则组并读取母线电压
         * ======================================================== */
        uint16_t raw_vbus = 0;
        
        /* 手动开启规则组转换 */
       HAL_ADC_Start(&hadc2);
			
			// 串口发送ia ib ic 数据 1Khz
			// 检查 DMA 是否空闲，避免覆盖正在发送的缓冲区
			if (vofa_tx_busy == 0) {
					vofa_tx_busy = 1;
					// 填充数据通道
					vofa_tx_frame.ch_data[0] = ia;
					vofa_tx_frame.ch_data[1] = ib;
					vofa_tx_frame.ch_data[2] = ic;
					vofa_tx_frame.ch_data[3] = i_q;
					vofa_tx_frame.ch_data[4] = i_d;
			
					// 启动 DMA 发送，一帧长度为 sizeof(VOFA_Frame_t) = 16 字节
					if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&vofa_tx_frame, sizeof(VOFA_Frame_t)) != HAL_OK) {
							vofa_tx_busy = 0; // 发送失败，释放忙标志
					}
				}
        /* 等待转换完成（1kHz周期下有充足时间，超时设置 1ms 足够） */

       raw_vbus = HAL_ADC_GetValue(&hadc2);
        
        /* 计算实际电压并做一阶低通滤波 */
        float vbus_raw = (float)raw_vbus * VBUS_SCALE;
        vbus_voltage = vbus_voltage * 0.9f + vbus_raw * 0.1f;

        /* ========================================================
         * 第 2 步：过欠压保护逻辑（仅在未锁存故障时检测）
         * ======================================================== */
        if (g_fault_latch == 0)
        {
            /* 2.1 过压检测 */
            if (vbus_voltage > OVP_THRESHOLD)
            {
                FOC_EMERGENCY_STOP();
                g_fault_code  = FAULT_OVP;
                g_fault_latch = 1;
                
                /* 红灯亮，蓝绿灭 */
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
            }
            /* 2.2 欠压检测 */
            else if (vbus_voltage < UVP_THRESHOLD)
            {
                FOC_EMERGENCY_STOP();
                g_fault_code  = FAULT_UVP;
                g_fault_latch = 1;
                
                /* 红灯亮，蓝绿灭 */
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
            }
        }
			
		
			}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        vofa_tx_busy = 0; // DMA 发送完成，允许下一帧数据覆盖缓冲区
    }
}

/* ============================================================
 * 故障锁存与指示（内联在中断中，无函数调用）
 * ============================================================ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2) return;
		
		HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET); // 用于测量运行时间
	
		/* -------- 第 0 步：故障锁存检查 --------
     * 若已锁存故障，不再执行任何输出，仅重新武装 ADC 并返回。
     * 必须手动把 g_fault_latch = 0 才能恢复输出。 */
		if (g_fault_latch)
    {
        HAL_ADCEx_InjectedStart_IT(hadc);
        return;
    }
    /* -------- 第 1 步：读取 ADC 原始值 -------- */
    uint16_t raw_rank1 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);  /* PA6 */
    uint16_t raw_rank2 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);  /* PA4 */
		
    /* -------- 第 2 步：换算为物理电流 -------- */
    ia = ((float)raw_rank1 - ia_offset) * CURRENT_SCALE;
    ic = ((float)raw_rank2 - ic_offset) * CURRENT_SCALE;
    ib = -ia - ic;

    /* ============================================================
     * 第 3 步：过流检测（关键保护逻辑）
     * ============================================================ */
	
    /* 3.1 求三相电流绝对值，取最大值用于统一判断 */
    float fabs_ia = fabsf(ia);
    float fabs_ib = fabsf(ib);
    float fabs_ic = fabsf(ic);
    float i_max   = fabs_ia;
    if (fabs_ib > i_max) i_max = fabs_ib;
    if (fabs_ic > i_max) i_max = fabs_ic;

    /* 3.2 硬阈值：瞬时超过立即关断，不消抖 */
    if (i_max > OC_HARD_LIMIT_A)
    {
        FOC_EMERGENCY_STOP();
        g_fault_code  = FAULT_OC_HARD;
        g_fault_latch = 1;

        /* 红灯常亮告警（假设低电平点亮，若极性相反请改为 SET） */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
       
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);   /* 绿灭 */

        HAL_ADCEx_InjectedStart_IT(hadc);
        return;
    }

    /* 3.3 软阈值：连续超限才触发，抑制噪声误报 */
    if (i_max > OC_SOFT_LIMIT_A)
    {
        oc_debounce++;
        if (oc_debounce >= OC_DEBOUNCE_CNT)
        {
            FOC_EMERGENCY_STOP();
            /* 记录是哪一相最先超限，便于诊断 */
            if      (fabs_ia > OC_SOFT_LIMIT_A) g_fault_code = FAULT_OC_IA;
            else if (fabs_ib > OC_SOFT_LIMIT_A) g_fault_code = FAULT_OC_IB;
            else                                 g_fault_code = FAULT_OC_IC;
            g_fault_latch = 1;

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);  /* 红亮 */
       
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

            HAL_ADCEx_InjectedStart_IT(hadc);
            return;
        }
    }
    else
    {
        /* 未超限：消抖计数器缓慢归零，避免计数器残留导致误报 */
        if (oc_debounce > 0) oc_debounce--;
    }
	
     /* ============================================================
     * 第 4 步：执行 I/F 控制（电流闭环，角度开环）
     * ============================================================ */

    /* 4.1 频率斜坡与强制角度更新 */
    if (freq_e < TARGET_FREQ_E)
    {
        freq_e += RAMP_RATE * FOC_DT;
        if (freq_e > TARGET_FREQ_E) freq_e = TARGET_FREQ_E;
    }
    
    /* 更新强制电角度 (弧度) */
    theta_e += 6.28318531f * freq_e * FOC_DT;
    if (theta_e >= 6.28318531f) theta_e -= 6.28318531f;

    /* 预计算三角函数 (反 Park 和 Park 变换都要用) */
    float sin_e = sinf(theta_e);
    float cos_e = cosf(theta_e);

    /* 4.2 Clarke 变换 (等幅值变换)：Ia, Ib, Ic -> I_alpha, I_beta */
    /* 假设 ia + ib + ic = 0 */
    i_alpha = ia;
    i_beta  = (ia + 2.0f * ib) * 0.57735027f;  /* 1/√3 = 0.57735 */

    /* 4.3 Park 变换：I_alpha, I_beta -> I_d, I_q */
    i_d =  i_alpha * cos_e + i_beta * sin_e;
    i_q = -i_alpha * sin_e + i_beta * cos_e;

    /* 4.4 电流环 PI 控制器 */
    float err_d = IF_ID_REF - i_d;
    float err_q = IF_IQ_REF - i_q;

    /* 积分项累加 (带抗饱和) */
    integral_d += PI_KI * err_d;
    integral_q += PI_KI * err_q;

    /* 积分限幅：防止 PI 输出无限大 */
    if (integral_d > V_OUT_MAX)  integral_d = V_OUT_MAX;
    if (integral_d < -V_OUT_MAX) integral_d = -V_OUT_MAX;
    if (integral_q > V_OUT_MAX)  integral_q = V_OUT_MAX;
    if (integral_q < -V_OUT_MAX) integral_q = -V_OUT_MAX;

    /* 计算输出电压 V_d, V_q */
    v_d = PI_KP * err_d + integral_d;
    v_q = PI_KP * err_q + integral_q;

    /* 输出限幅 */
    if (v_d > V_OUT_MAX)  v_d = V_OUT_MAX;
    if (v_d < -V_OUT_MAX) v_d = -V_OUT_MAX;
    if (v_q > V_OUT_MAX)  v_q = V_OUT_MAX;
    if (v_q < -V_OUT_MAX) v_q = -V_OUT_MAX;

    /* 4.5 反 Park 变换：V_d, V_q -> V_alpha, V_beta */
    v_alpha = v_d * cos_e - v_q * sin_e;
    v_beta  = v_d * sin_e + v_q * cos_e;

    /* 4.6 反 Clarke 变换（为了 SVPWM 注入零序） */
    /* V_a, V_b, V_c 表示三相相电压参考值 */
    float va_ref = v_alpha;
    float vb_ref = -0.5f * v_alpha + 0.86602540f * v_beta; /* √3/2 = 0.866 */
    float vc_ref = -0.5f * v_alpha - 0.86602540f * v_beta;

    /* 4.7 SVPWM 零序注入 (等效 SVPWM 算法，母线利用率最高) */
    float v_max = va_ref, v_min = va_ref;
    if (vb_ref > v_max) v_max = vb_ref;
    if (vc_ref > v_max) v_max = vc_ref;
    if (vb_ref < v_min) v_min = vb_ref;
    if (vc_ref < v_min) v_min = vc_ref;
    
    float v_common = -(v_max + v_min) * 0.5f;
    va_ref += v_common;
    vb_ref += v_common;
    vc_ref += v_common;

    /* 4.8 转占空比并写入 CCR */
    float vbus_safe = vbus_voltage;
    if (vbus_safe < 10.0f) vbus_safe = 10.0f; 

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

    /* 4.9 重新武装 ADC2 */
    HAL_ADCEx_InjectedStart_IT(hadc);
		
			
		
		HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
