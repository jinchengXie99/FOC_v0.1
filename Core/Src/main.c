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
#include "tim.h"
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
#define VF_RATIO            0.028f

/* 目标电频率（Hz）：电机实际机械转速 = freq_e / POLE_PAIRS / 60 */
#define TARGET_FREQ_E       60.0f

/* 频率斜坡速率（Hz/s）：防止直接跳频导致电流冲击 */
#define RAMP_RATE           20.0f

/* 母线电压（V）：若启用 ADC 母线采样，可在运行时动态刷新 */
#define VBUS_NOMINAL        24.0f

/* SPWM 线性区最大相电压幅值 = Vbus / √3，再乘 0.95 留余量 */
#define V_PHASE_MAX         (VBUS_NOMINAL * 0.5773f * 0.95f)

/* 电流采样换算系数：I = (raw - offset) × CURRENT_SCALE
 * 推导：3.3V / 4096 / (5mΩ × 10) ≈ 0.01611 A/LSB */
#define CURRENT_SCALE       0.016113f

/* ADC 零点偏置（1.65V 对应 2048）：后续标定后替换 */
#define IA_OFFSET_DEFAULT   1991.0f
#define IC_OFFSET_DEFAULT    1971.0f

/* ============================================================
 * 运行时状态变量
 * ============================================================ */

/* 电流采样 */
volatile float ia = 0.0f;       /* A 相电流 (A) */
volatile float ib = 0.0f;       /* B 相电流 (A) */
volatile float ic = 0.0f;       /* C 相电流 (A) */

/* V/F 状态 */
volatile float theta_e = 0.0f;  /* 当前电角度 (rad)，0~2π */
volatile float freq_e  = 0.0f;  /* 当前电频率 (Hz) */
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

/* 运行时故障状态（volatile 保证中断与主循环可见性） */
volatile uint8_t  g_fault_code   = FAULT_NONE;  /* 故障码 */
volatile uint8_t  g_fault_latch  = 0;           /* 1 = 已锁存，禁止输出 */
volatile uint8_t  oc_debounce    = 0;           /* 软阈值消抖计数器 */


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
  MX_TIM1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
		// 1. ADC2 校准
	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
			Error_Handler();
	}

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
		
        /* 检查故障是否真的消失了：三相电流都应接近 0 */
        uint8_t current_clear =
            (fabsf(ia) < 0.5f) &&
            (fabsf(ib) < 0.5f) &&
            (fabsf(ic) < 0.5f);

        /* KEY1 按下（低电平有效） */
      static  uint8_t key_pressed =0; key_pressed =(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET);

        if (current_clear && key_pressed)
        {
            /* 复位所有故障状态 */
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

            /* 红灯灭、绿灯亮表示恢复正常 */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

            /* 消抖延时，防止按键抖动重复触发 */
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
/* USER CODE BEGIN 4 */

/* ============================================================
 * 故障锁存与指示（内联在中断中，无函数调用）
 * ============================================================ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2) return;

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
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* 蓝灭 */
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
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
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
     * 第 4 步：过流保护通过，执行正常 V/F 控制
     * ============================================================ */

    /* 4.1 频率斜坡 */
    if (freq_e < TARGET_FREQ_E)
    {
        freq_e += RAMP_RATE * FOC_DT;
        if (freq_e > TARGET_FREQ_E) freq_e = TARGET_FREQ_E;
    }

    /* 4.2 幅值计算与限幅 */
    v_amp = freq_e * VF_RATIO;
    if (v_amp > V_PHASE_MAX) v_amp = V_PHASE_MAX;

    /* 4.3 角度更新与归一化 */
    theta_e += 6.28318531f * freq_e * FOC_DT;
    if (theta_e >= 6.28318531f) theta_e -= 6.28318531f;

    /* 4.4 三相正弦 */
    float va = v_amp * sinf(theta_e);
    float vb = v_amp * sinf(theta_e - 2.09439510f);
    float vc = v_amp * sinf(theta_e + 2.09439510f);

    /* 4.5 零序注入（等效 SVPWM） */
    float v_max = va, v_min = va;
    if (vb > v_max) v_max = vb;
    if (vc > v_max) v_max = vc;
    if (vb < v_min) v_min = vb;
    if (vc < v_min) v_min = vc;
    float v_common = -(v_max + v_min) * 0.5f;
    va += v_common;
    vb += v_common;
    vc += v_common;

    /* 4.6 转占空比并限幅 */
   static float duty_a = 0;
   static float duty_b = 0;
   static float duty_c = 0;
		duty_a = 0.5f + va / VBUS_NOMINAL;
		duty_b = 0.5f + vb / VBUS_NOMINAL;
    duty_c = 0.5f + vc / VBUS_NOMINAL;
    if (duty_a < 0.0f) duty_a = 0.0f; else if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f; else if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f; else if (duty_c > 1.0f) duty_c = 1.0f;

    /* 4.7 写入 CCR */
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a * (float)htim1.Instance->ARR));
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_b * (float)htim1.Instance->ARR));
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_c * (float)htim1.Instance->ARR));

    /* 4.8 重新武装 ADC2 */
    HAL_ADCEx_InjectedStart_IT(hadc);
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
