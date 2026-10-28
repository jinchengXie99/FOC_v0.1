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
#include "drv_config.h"       /* 硬件配置层 */
#include "mtr_config.h"       /* 电机配置层 */
#include "hw_cordic.h"        /* CORDIC 极速加速层 */
#include "hw_pwm.h"           /* PWM 底层驱动层 */
#include "hw_adc.h"           /* ADC 底层驱动层 */
#include "app_main.h"         /* 应用层总调度 */
#include "app_throttle.h"     /* 调速器映射 */
#include "app_telemetry.h"    /* VOFA+ 遥测 */
#include "safe_fault.h"       /* 安全保护层 */
#include "ctrl_observer.h"    /* 观测器 */
#include "ctrl_current_loop.h"/* 电流环 */
#include "ctrl_state_machine.h"/* 状态机 */
#include "ctrl_advanced.h"    /* MTPA 与弱磁算法 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
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
    /* 1. 框架与硬件加速器初始化 */
    HW_CORDIC_Init();       /* 启动 CORDIC 硬件三角函数加速 */
    HW_PWM_Disable();       /* 强制拉低 SD 引脚，确保上电安全 */
    APP_Init();             /* 初始化内部 PID、观测器、状态机等结构体 */

    /* 2. 启动 ADC 偏置校准 (驱动保持关闭，TIM1 仅提供 ADC 触发) */
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    
    /* 配置 TIM1_CH4 比较值，并确保三相 PWM 初始占空比为零。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, DRV_PWM_ARR - 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);

    /* 母线与油门 ADC 连续采样。 */
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_regular_dma_buf,
                          ADC_REGULAR_BUF_SIZE) != HAL_OK) {
        Error_Handler();
    }

    /* 先启动 TIM1 产生注入组触发，功率驱动仍由 SD 引脚禁用。 */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    
    /* 启动注入组 ADC 中断 */
    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    /* 3. 等待 2000 次偏置采样；转换未触发时超时退出而不是永久卡住。 */
    uint32_t calib_start_tick = HAL_GetTick();
    while (g_calib_done == 0) {
        if ((HAL_GetTick() - calib_start_tick) >= 500U) {
            Error_Handler();
        }
        HAL_Delay(1);
    }

    /* 4. 上电保持空闲；TIM2 只会在油门有效且母线正常时启动电机。 */
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    /* 6. 状态指示灯 */
    HAL_GPIO_WritePin(DRV_LED_R_PORT, DRV_LED_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV_LED_G_PORT, DRV_LED_G_PIN, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 主循环只负责兜底任务：故障恢复检测、按键扫描、低频任务 */
    SAFE_HandleFault();

    /* 可以在这里加入 LED 闪烁、OLED 刷新等不需要硬实时性的代码 */
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

/* ============================================================
 * 中断回调路由中心
 * 严格按照"大厂规范"：中断里只做路由，具体逻辑下沉到各模块
 * ============================================================ */

/**
  * @brief 20kHz 极速电流环中断 (由 TIM1_CH4 触发)
  * 包含：偏置校准、Clarke变换、观测器更新、电流环执行
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2) return;

    // 1. 读取原始 ADC 值并执行偏置校准
    uint16_t raw_ia = (uint16_t)ADC2->JDR1;
    uint16_t raw_ic = (uint16_t)ADC2->JDR2;

    if (HW_ADC_CalibrateInjected(raw_ia, raw_ic) == 0) {
        return; // 校准未完成，直接返回
    }

    // 2. 换算物理电流
    HW_ADC_GetCurrents(&g_motor.i_a, &g_motor.i_b, &g_motor.i_c);

    if (g_motor.state == STATE_FAULT || g_motor.state == STATE_IDLE) {
        return;
    }
		
    if (SAFE_CheckCurrentFast(g_motor.i_a, g_motor.i_b, g_motor.i_c) != FAULT_NONE) {
        return;
    }

   

    // 3. Clarke 变换 (计算 I_alpha, I_beta)
    g_motor.i_alpha = g_motor.i_a;
    g_motor.i_beta  = (g_motor.i_a + 2.0f * g_motor.i_b) * 0.57735027f;

    // 4. VESC 观测器更新 (计算角度与速度)
    Ctrl_Observer_Update(&g_motor.obs, g_motor.v_alpha, g_motor.v_beta,
                         g_motor.i_alpha, g_motor.i_beta, FOC_DT);

    // 5. IF 角度与频率必须按电流环周期更新。
    if (g_motor.state == STATE_IF) {
        if (g_motor.freq_e < IF_START_FREQ) {
            g_motor.freq_e += IF_RAMP_RATE * FOC_DT;
            if (g_motor.freq_e > IF_START_FREQ) {
                g_motor.freq_e = IF_START_FREQ;
            }
        }
        g_motor.theta_if += 6.28318531f * g_motor.freq_e * FOC_DT;
        if (g_motor.theta_if >= 6.28318531f) {
            g_motor.theta_if -= 6.28318531f;
        }
        g_motor.obs.theta_est = g_motor.theta_if;

        if (g_motor.freq_e >= IF_START_FREQ) {
            Ctrl_Observer_ForceAlign(&g_motor.obs, g_motor.theta_if,
                                     g_motor.freq_e, g_motor.i_alpha,
                                     g_motor.i_beta);
            g_motor.run_timer = 0U;
            g_motor.state = STATE_RUN;
        }
    }

    // 6. 执行 20kHz 极速电流环 (Park/反Park/SVPWM)
    if (g_motor.state == STATE_IF || g_motor.state == STATE_RUN ||
        g_motor.state == STATE_BRAKE) {
        Ctrl_CurrentLoop_Execute(&g_motor, HW_ADC_GetVbus(), FOC_DT);
    }
}

/**
  * @brief 1kHz 任务调度中断
  * 包含：故障检测、调速器映射、状态机流转、速度环、遥测发送
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        float vbus = HW_ADC_GetVbus();

        // 1. 安全与故障拦截
        if (g_motor.state != STATE_FAULT) {
            uint8_t fault = SAFE_Check(vbus, g_motor.i_a, g_motor.i_b, g_motor.i_c);
            if (fault != FAULT_NONE) {
                g_motor.fault_code = fault;
                g_motor.state = STATE_FAULT;
                HW_PWM_Disable();
            }
        }

        // 2. 滑动变阻器电压处理与目标转速映射
        float throttle_v = HW_ADC_GetThrottleVoltage();
        APP_Throttle_Update(throttle_v, &g_motor.target_rpm);

        // 油门归零时安全停机；空闲时只在母线正常且油门有效时启动。
        if (g_motor.state != STATE_FAULT) {
            if (g_motor.target_rpm <= 0.0f) {
                g_motor.state = STATE_IDLE;
            } else if (g_motor.state == STATE_IDLE &&
                       vbus > DRV_UVP_RECOVER &&
                       vbus < DRV_OVP_RECOVER) {
                HW_PWM_SetDuty(0.5f, 0.5f, 0.5f);
                g_motor.state = STATE_IF;
            }
        }

        // 3. 核心状态机流转
        Ctrl_StateMachine_Task(&g_motor, vbus, 0.001f);

        // 4. 速度环与高级算法 (仅在闭环运行下生效)
        if (g_motor.state == STATE_RUN) {
            if (g_motor.run_timer < 10U) {
                g_motor.run_timer++;
                g_motor.pid_spd.integral = IF_START_IQ;
                g_motor.iq_ref = IF_START_IQ;
            } else {
                float speed_error = g_motor.target_rpm - g_motor.obs.speed_rpm;
                if (isnan(speed_error) || isinf(speed_error)) {
                    speed_error = 0.0f;
                }
                if (fabsf(speed_error) < 20.0f) {
                    speed_error = 0.0f;
                } else if (speed_error > 2000.0f) {
                    speed_error = 2000.0f;
                } else if (speed_error < -2000.0f) {
                    speed_error = -2000.0f;
                }
                float speed_target = g_motor.obs.speed_rpm + speed_error;
                g_motor.iq_ref = Math_PID_Calc(&g_motor.pid_spd, speed_target,
                                               g_motor.obs.speed_rpm, 0.001f);
            }

            // 高级算法：MTPA 与 弱磁计算
            g_motor.id_mtpa_ref = CTRL_Calc_MTPA(g_motor.iq_ref);
            g_motor.id_fw_ref = CTRL_Calc_FieldWeakening(
                g_motor.v_d, g_motor.v_q, vbus,
                g_motor.i_q, g_motor.iq_ref,
                g_motor.id_fw_ref, 0.001f
            );
        }

        // 5. VOFA+ 遥测数据打包与 DMA 发送
        APP_Telemetry_Send(&g_motor);
    }
}

/**
  * @brief 串口 DMA 发送完成回调
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        APP_Telemetry_TxCpltCallback();
    }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
