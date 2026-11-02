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
  * @brief 应用入口：初始化 HAL、时钟、外设和电机控制模块后进入主循环。
  * @retval int 标准 C 入口返回值；本固件正常情况下不会返回。
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* 初始化 HAL 库和系统节拍，再配置 MCU 时钟树。 */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* 配置 SYSCLK/HCLK/PCLK 为 170 MHz（详见 SystemClock_Config）。 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* 按 CubeMX 配置初始化 GPIO、DMA、定时器、ADC 和串口外设。 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
    /* 初始化 CORDIC、保持功率驱动关闭，并建立应用控制器初始状态。 */
    HW_CORDIC_Init();       /* 启动 CORDIC 硬件三角函数加速 */
    HW_PWM_Disable();       /* 强制拉低 SD 引脚，确保上电安全 */
    APP_Init();             /* 初始化内部 PID、观测器、状态机等结构体 */

    /*
     * 先校准 ADC2；随后 TIM1 产生注入组触发，但外部 SD 使能保持低电平，
     * 避免校准期间逆变器向电机输出功率。
     */
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    
    /* CH4 用作 ADC 注入转换触发；CH1-CH3 初始 CCR 清零。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, DRV_PWM_ARR - 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);

    /* 规则组 DMA 连续更新母线电压和调速器电压两个采样值。 */
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_regular_dma_buf,
                          ADC_REGULAR_BUF_SIZE) != HAL_OK) {
        Error_Handler();
    }

    /* 启动 TIM1 三相 PWM 计数及 CH4 触发，驱动功率级仍处于禁用状态。 */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    
    /* ADC 注入转换完成后进入 20 kHz 电流控制回调。 */
    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    /* 等待 ADC 高频回调累计 2000 组电流零点样本，最多等待 500 ms。 */
    uint32_t calib_start_tick = HAL_GetTick();
    while (g_calib_done == 0) {
        if ((HAL_GetTick() - calib_start_tick) >= 500U) {
            Error_Handler();
        }
        HAL_Delay(1);
    }

    /* 启动 TIM2 低频任务；电机仍保持 IDLE，满足输入和母线条件后才启动。 */
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    /* 初始化状态灯：红灯灭、绿灯亮，表示系统已完成启动。 */
    HAL_GPIO_WritePin(DRV_LED_R_PORT, DRV_LED_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV_LED_G_PORT, DRV_LED_G_PIN, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 非硬实时主循环执行故障恢复轮询；FOC 和 1 kHz 调度由中断负责。 */
    SAFE_HandleFault();

    /* 可以在这里加入 LED 闪烁、OLED 刷新等不需要硬实时性的代码 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief 配置外部高速时钟及 PLL，建立 170 MHz 系统时钟。
  * @retval None
  *
  * SYSCLK、AHB、APB1、APB2 均使用 PLL 输出并设置相应 Flash 等待周期。
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
 * ADC 注入转换完成回调是高速电流控制入口：依次完成零点校准、
 * 电流换算、保护、Clarke 变换、观测器/IF 状态更新和电流环执行。
 * ============================================================ */

/**
  * @brief TIM1_CH4 触发 ADC 注入转换完成后执行高速电流控制。
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2) return;

    /* 读取两路注入组原始样本，先完成启动阶段的零电流偏置累计。 */
    uint16_t raw_ia = (uint16_t)ADC2->JDR1;
    uint16_t raw_ic = (uint16_t)ADC2->JDR2;

    if (HW_ADC_CalibrateInjected(raw_ia, raw_ic) == 0) {
        return; /* 校准期间不使用尚未确认的电流偏置执行控制。 */
    }

    /* 将 ADC 样本换算为 IA、IB、IC (A)。 */
    HW_ADC_GetCurrents(&g_motor.i_a, &g_motor.i_b, &g_motor.i_c);

    if (g_motor.state == STATE_FAULT) {
        return;
    }
		
    if (SAFE_CheckCurrentFast(g_motor.i_a, g_motor.i_b, g_motor.i_c) != FAULT_NONE) {
        return;
    }
    if (g_motor.state == STATE_IDLE) {
        return;
    }

    /* Clarke 变换：三相电流 (A) 映射到静止坐标系 alpha-beta (A)。 */
    g_motor.i_alpha = g_motor.i_a;
    g_motor.i_beta  = (g_motor.i_a + 2.0f * g_motor.i_b) * 0.57735027f;

    /* 以本周期电压/电流反馈更新磁链观测器及 PLL。 */
    Ctrl_Observer_Update(&g_motor.obs, g_motor.v_alpha, g_motor.v_beta,
                         g_motor.i_alpha, g_motor.i_beta, FOC_DT);

    /* IF 启动时按 FOC_DT 积分电频率得到电角度，并同步观测器状态。 */
    if (g_motor.state == STATE_IF) {
        float target_freq = (float)g_motor.direction * IF_START_FREQ;
        if (g_motor.direction > 0 && g_motor.freq_e < target_freq) {
            g_motor.freq_e += IF_RAMP_RATE * FOC_DT;
            if (g_motor.freq_e > target_freq) g_motor.freq_e = target_freq;
        } else if (g_motor.direction < 0 && g_motor.freq_e > target_freq) {
            g_motor.freq_e -= IF_RAMP_RATE * FOC_DT;
            if (g_motor.freq_e < target_freq) g_motor.freq_e = target_freq;
        }
        g_motor.theta_if += 6.28318531f * g_motor.freq_e * FOC_DT;
        if (g_motor.theta_if >= 6.28318531f) {
            g_motor.theta_if -= 6.28318531f;
        } else if (g_motor.theta_if < 0.0f) {
            g_motor.theta_if += 6.28318531f;
        }
        g_motor.obs.theta_est = g_motor.theta_if;

        if (fabsf(g_motor.freq_e) >= IF_START_FREQ) {
            Ctrl_Observer_ForceAlign(&g_motor.obs, g_motor.theta_if,
                                     g_motor.freq_e, g_motor.i_alpha,
                                     g_motor.i_beta);
            g_motor.run_timer = 0U;
            g_motor.state = STATE_RUN;
        }
    }

    /* IF、闭环 RUN 和 BRAKE 状态均运行高速 dq 电流环及 SVPWM。 */
    if (g_motor.state == STATE_IF || g_motor.state == STATE_RUN ||
        g_motor.state == STATE_BRAKE) {
        Ctrl_CurrentLoop_Execute(&g_motor, HW_ADC_GetVbus(), FOC_DT);
    }
}

/**
  * @brief TIM2 低频任务入口，通常按 1 kHz 调用。
  *
  * 依次执行保护检查、调速器映射、状态转换、速度及高级算法更新和遥测。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        float vbus = HW_ADC_GetVbus();

        /* 低频冗余保护；检测到故障即锁存状态并关闭功率级。 */
        if (g_motor.state != STATE_FAULT) {
            uint8_t fault = SAFE_Check(vbus, g_motor.i_a, g_motor.i_b, g_motor.i_c);
            if (fault != FAULT_NONE) {
                g_motor.fault_code = fault;
                g_motor.state = STATE_FAULT;
                HW_PWM_Disable();
            }
        }

        /* 读取调速器 ADC 电压并映射为有符号机械转速目标 (RPM)。 */
        float throttle_v = HW_ADC_GetThrottleVoltage();
        float target_rpm;
        APP_Throttle_Update(throttle_v, &target_rpm);
        g_motor.target_rpm = target_rpm;

        /*
         * 中位命令进入有源制动；变向先减速到近零，再切换转向启动，
         * 避免在明显反向转速下直接施加新方向的启动电流。
         */
        if (g_motor.state != STATE_FAULT) {
            int8_t target_direction = target_rpm > 0.0f ? 1 : -1;
            float speed_abs = fabsf(g_motor.obs.speed_rpm);

            if (target_rpm == 0.0f) {
                if (g_motor.state == STATE_IF || g_motor.state == STATE_RUN) {
                    g_motor.state = STATE_BRAKE;
                }
            } else if (g_motor.state == STATE_IDLE &&
                       vbus > DRV_UVP_RECOVER &&
                       vbus < DRV_OVP_RECOVER) {
                if (speed_abs > 20.0f) {
                    g_motor.state = STATE_BRAKE;
                } else {
                    g_motor.direction = target_direction;
                    g_motor.state = STATE_IF;
                }
            } else if (g_motor.state == STATE_IF &&
                       target_direction != g_motor.direction) {
                g_motor.state = STATE_BRAKE;
            } else if (g_motor.state == STATE_RUN &&
                       target_direction != (g_motor.obs.speed_rpm >= 0.0f ? 1 : -1)) {
                g_motor.state = speed_abs <= 20.0f ? STATE_IDLE : STATE_BRAKE;
            }
        }

        /* 执行 IDLE/IF/RUN/BRAKE/FAULT 的进入动作及状态内处理。 */
        Ctrl_StateMachine_Task(&g_motor, vbus, 0.001f);

        /* 速度环、MTPA 和弱磁仅在无感闭环 RUN 状态更新。 */
        if (g_motor.state == STATE_RUN) {
					
            if (g_motor.run_timer <= 20U) {
                g_motor.pi_speed.integral = (float)g_motor.direction * IF_START_IQ;
                g_motor.iq_ref = (float)g_motor.direction * IF_START_IQ;
            } else {
							
                /* 1 kHz 采样下 alpha=0.01 的一阶低通，近似截止频率约 1.6 Hz。 */
								g_motor.obs.speed_rpm_filt = g_motor.obs.speed_rpm_filt * 0.99f + g_motor.obs.speed_rpm * 0.01f;
							  float speed_actual = g_motor.obs.speed_rpm_filt;
                /* 速度误差与目标、反馈均为机械 RPM。 */
                float speed_error = g_motor.target_rpm - speed_actual;
                if (isnan(speed_actual) || isinf(speed_actual) ||
                    isnan(speed_error) || isinf(speed_error)) {
                    g_motor.iq_ref = 0.0f;
                    Speed_PI_Reset(&g_motor.pi_speed);
                } else {
                    if (fabsf(speed_error) < 10.0f) {
                        speed_error = 0.0f;
                    } 
//										else if (speed_error > 1000.0f) {
//                        speed_error = 1000.0f;
//                    } else if (speed_error < -1000.0f) {
//                        speed_error = -1000.0f;
//                    }

                    /* 死区处理后重构速度目标，由速度 PI 输出 q 轴电流参考。 */
                    float speed_target  = speed_actual + speed_error;
                    g_motor.iq_ref = Speed_PI_Calc(&g_motor.pi_speed,
                                                   speed_target,
                                                   speed_actual,
                                                   0.001f);
                }
            }

                /* 分别计算 MTPA 和弱磁 d 轴电流参考 (A)。 */
								g_motor.id_mtpa_ref = CTRL_Calc_MTPA(g_motor.iq_ref);
								g_motor.id_fw_ref = CTRL_Calc_FieldWeakening(g_motor.v_d, g_motor.v_q, vbus,
									g_motor.i_q, g_motor.iq_ref,
									g_motor.id_fw_ref, 0.001f);
        }

        /* 将当前电机量打包为遥测帧；发送状态由遥测模块管理。 */
        APP_Telemetry_Send(&g_motor);
    }
}

/**
  * @brief USART1 DMA 发送完成后通知遥测模块释放发送缓冲区。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        APP_Telemetry_TxCpltCallback();
    }
}

/* USER CODE END 4 */

/**
  * @brief HAL 初始化或外设启动失败后的安全停机循环。
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
  /* 可在此处加入断言位置记录；当前实现不执行额外恢复动作。 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
