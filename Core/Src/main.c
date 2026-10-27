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
/* 求绝对值最大的那个 */
#define UTILS_MAX_ABS(a, b) ((fabsf(a) > fabsf(b)) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ============================================================
 * 1. 电机物理参数
 * ============================================================ */
#define MOTOR_RS            0.3644f/2.0f        /* 定子相电阻 (Ω) */
#define MOTOR_LD            0.0000829f/2.0f     /* D轴电感 (H) */
#define MOTOR_LQ            0.0000829f/2.0f     /* Q轴电感 (H) 注：表贴式Ld=Lq, 内置式Lq>Ld */
#define MOTOR_LS            MOTOR_LQ            /* 观测器使用的平均电感 */
#define MOTOR_POLE_PAIRS    7.0f                /* 极对数 */

/* --- 反电动势实测数据 --- */
#define MOTOR_VPP_LINE      11.98f              
#define MOTOR_F_E           203.73f             
#define FLUX_CALC_CONST     (0.04594407f)
#define CALC_PM_FLUX(Vpp, f_e)  (((float)(Vpp) * FLUX_CALC_CONST) / ((float)(f_e) > 0.1f ? (float)(f_e) : 0.1f))
#define MOTOR_FLUX_LINKAGE  CALC_PM_FLUX(MOTOR_VPP_LINE, MOTOR_F_E) 

/* ============================================================
 * 2. 系统控制参数
 * ============================================================ */
#define FOC_FREQ            20000.0f            /* FOC控制频率 20kHz */
#define FOC_DT              (1.0f / FOC_FREQ)   /* 控制周期 50us */

/* --- 启动与切入参数 --- */
#define IF_START_FREQ        20.0f               
#define IF_RAMP_RATE         15.0f               
#define IF_START_IQ          0.5f                

/* --- VESC 非线性磁链观测器参数 --- */
#define OBSERVER_GAIN       500.0f              

/* --- 电流环参数 (带宽法) --- */
#define CURRENT_BANDWIDTH   10000.0f             
#define USE_DECOUPLING      1                   
#define CURRENT_KP  (MOTOR_LS * CURRENT_BANDWIDTH)
#define CURRENT_KI  (MOTOR_RS * CURRENT_BANDWIDTH)

/* --- 速度环 PI 参数 --- */
#define SPD_KP              0.0005f       
#define SPD_KI              0.00001f      
#define SPD_TARGET_RPM      1000.0f             

/* --- PLL 锁相环参数 --- */
#define PLL_KP              50.0f                
#define PLL_KI              2000.0f             

/* --- 弱磁 (Field Weakening) 参数 --- */
#define FW_VOLTAGE_THRESHOLD 0.93f              /* 电压利用率超过 90% 开始弱磁 */
#define FW_MAX_CURRENT       3.0f               /* 最大去磁电流 (A) */
#define FW_RAMP_STEP         0.01f              /* 弱磁电流爬坡步长 */

/* ============================================================
 * 3. 控制状态机与系统运行变量
 * ============================================================ */
/* 控制模式状态机 */
typedef enum {
    CTRL_STOP = 0,      /* 停机，关断输出 */
    CTRL_SPEED,         /* 速度闭环运行 */
    CTRL_BRAKE          /* 刹车模式 */
} ControlState_t;
volatile ControlState_t ctrl_state = CTRL_SPEED;  /* 默认启动速度模式 */
volatile float brake_current_target = 2.0f;       /* 刹车电流目标 */
/* ============================================================
 * 滑动变阻器（调速器）参数配置
 * ============================================================ */
#define THROTTLE_V_MIN      0.1f    /* 最低有效电压 (V)，低于此值视为停机 */
#define THROTTLE_V_MAX      3.2f    /* 最高有效电压 (V)，达到此值对应最高转速 */
#define THROTTLE_RPM_MAX    5000.0f /* 对应的最高机械转速 (RPM)，根据你的电机修改 */

volatile float throttle_voltage = 0.0f;        /* 滤波后的滑动变阻器电压 */
volatile float throttle_target_rpm = 0.0f;     /* 映射后的目标转速 */
/* FOC 内部状态机 */
typedef enum {
    STATE_IF = 0,       
    STATE_RUN = 1       
} MotorState_t;
volatile MotorState_t motor_state = STATE_IF;

/* 电流变量 */
volatile float ia = 0.0f, ib = 0.0f, ic = 0.0f;
volatile float i_alpha = 0.0f, i_beta = 0.0f;
volatile float i_d = 0.0f, i_q = 0.0f;

/* 电压变量 */
volatile float v_d = 0.0f, v_q = 0.0f;
volatile float v_alpha_ref = 0.0f, v_beta_ref = 0.0f;

/* 角度与速度变量 */
volatile float theta_e = 0.0f;                  
volatile float theta_est = 0.0f;                
volatile float theta_if = 0.0f;                 
volatile float omega_e_est = 0.0f;              
volatile float speed_rpm_est = 0.0f;            
volatile float freq_e = 0.0f;                   

float spd_target_rpm = SPD_TARGET_RPM;
volatile uint16_t run_timer = 0;                

/* VESC 观测器内部磁链状态 */
static float obs_x1 = 0.0f;           
static float obs_x2 = 0.0f;           
static float pll_integral = 0.0f;      

/* 控制器积分器状态 */
static float integral_spd = 0.0f;
static float integral_d = 0.0f;
static float integral_q = 0.0f;
static float iq_ref = 0.0f;

/* MTPA 与 弱磁目标 */
volatile float id_mtpa_ref = 0.0f;   
volatile float id_fw_ref = 0.0f;     

/* ============================================================
 * 4. 偏置电流自动校准参数
 * ============================================================ */
float ia_offset = 1973.0f; 
float ic_offset = 1991.0f;
volatile uint8_t g_calib_state = 0;  /* 0: 未校准, 1: 校准中, 2: 校准完成 */
#define CALIB_SAMPLES 2000           /* 采集 2000 次求平均（100ms） */
uint32_t calib_ia_sum = 0;
uint32_t calib_ic_sum = 0;
uint16_t calib_cnt = 0;

/* ============================================================
 * 5. 硬件采样与保护参数
 * ============================================================ */
#define CURRENT_SCALE       0.016113f          /* 电流采样换算系数 */
#define VBUS_SCALE          0.017947f          /* 母线电压换算系数 */
volatile float vbus_voltage = 24.0f; 
volatile float v_amp = 0.0f;


#define ADC_REGULAR_BUF_SIZE 2   
volatile uint16_t adc_regular_dma_buf[ADC_REGULAR_BUF_SIZE]; // [0]母线 [1]通道12

/* 故障阈值设定 */
#define OC_HARD_LIMIT_A     25.0f
#define OC_SOFT_LIMIT_A     20.0f
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
 * 6. 串口通信 (VOFA+ JustFloat)
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
    /* 1. ADC 校准 */
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    
    /* 2. 启动 1kHz 速度环定时器 */
    HAL_TIM_Base_Start_IT(&htim2);

    /* 3. 配置 ADC 注入触发点 (PWM 峰值处，下桥全开) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (htim1.Instance->ARR - 1)); 
    
    /* 4. 初始化 PWM 占空比为 0 (三相下桥全开，电机短路但无电流) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* 5. 启动 TIM1 四路 PWM (此时因为 SD 引脚未使能，功率管不会真的输出) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); 

    /* 6. 启动 ADC 注入转换，开始自动校准偏置电流 */
    g_calib_state = 1; 
    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
        Error_Handler();
    }
		HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_regular_dma_buf, ADC_REGULAR_BUF_SIZE);
    /* 7. 等待校准完成 (主循环阻塞等待 100ms 左右) */
    while (g_calib_state != 2) {
        HAL_Delay(1);
    }

    /* 8. 校准完成，使能 EG2104 驱动进入正常工作状态 */
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
            ctrl_state    = CTRL_SPEED; /* 故障恢复后默认回速度模式 */

            pll_integral = 0.0f;
            omega_e_est  = 0.0f;
            obs_x1       = 0.0f;
            obs_x2       = 0.0f;
            integral_spd = 0.0f;
            integral_d   = 0.0f;
            integral_q   = 0.0f;

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
  * @brief 定时器周期回调（1kHz） - 速度环、MTPA与控制状态机
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* ========================================================
         * 1. 状态机与电流参考计算
         * ======================================================== */
        if (ctrl_state == CTRL_STOP)
        {
            iq_ref = 0.0f;
            id_mtpa_ref = 0.0f;
            integral_spd = 0.0f;
            integral_q = 0.0f;
            integral_d = 0.0f;
            
            // 完全停机，关断 SD 引脚
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        }
        else if (ctrl_state == CTRL_BRAKE)
        {
            // 刹车模式：使能驱动，注入反向 Iq 电流
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
            
            iq_ref = -fabsf(brake_current_target); 
            id_mtpa_ref = 0.0f;
            integral_spd = 0.0f; // 断开速度环介入
        }
        else // CTRL_SPEED
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
            
            if (motor_state == STATE_RUN)
            {
                if (run_timer < 65535) run_timer++; 
                
                if (run_timer < 10) 
                {		
                    iq_ref = IF_START_IQ; 
                    integral_spd = IF_START_IQ; 
                }
                else
                {
                    float err_spd = spd_target_rpm - speed_rpm_est;
                    if (isnan(err_spd) || isinf(err_spd)) err_spd = 0.0f;
                    if (err_spd > -20.0f && err_spd < 20.0f) err_spd = 0.0f; 
                    if (err_spd > 2000.0f)  err_spd = 2000.0f;  
                    if (err_spd < -2000.0f) err_spd = -2000.0f;
                    
                    integral_spd += SPD_KI * err_spd;
                    if (integral_spd > 18.0f)  integral_spd = 18.0f; 
                    if (integral_spd < -10.0f) integral_spd = -10.0f;
                    
                    float iq_target = SPD_KP * err_spd + integral_spd;
                    if (iq_target > 18.0f)  iq_target = 18.0f;
                    if (iq_target < -10.0f) iq_target = -10.0f;
                    
                    iq_ref = iq_target;
                }
            }
            else // STATE_IF
            {
                iq_ref = IF_START_IQ;
                integral_spd = 0.0f; 
                run_timer = 0;
                omega_e_est = 6.28318531f * freq_e; 
                speed_rpm_est = (omega_e_est / MOTOR_POLE_PAIRS) * (60.0f / 6.28318531f);
            }

            /* ========================================================
             * MTPA 算法 (仅速度模式下参与)
             * ======================================================== */
            float l_diff = MOTOR_LQ - MOTOR_LD;
            if (fabsf(l_diff) > 1e-7f && fabsf(iq_ref) > 0.1f)
            {
                // MTPA 公式 (简化)：Id = (λ - sqrt(λ² + 8(Lq-Ld)²Iq²)) / (4(Lq-Ld))
                id_mtpa_ref = (MOTOR_FLUX_LINKAGE - sqrtf(MOTOR_FLUX_LINKAGE * MOTOR_FLUX_LINKAGE + 
                              8.0f * l_diff * l_diff * iq_ref * iq_ref)) / (4.0f * l_diff);
            }
            else
            {
                id_mtpa_ref = 0.0f; 
            }
        }
        
       /* ========================================================
         * 2. 母线电压 & 滑动变阻器调速采样
         * ======================================================== */
        // 【重要】这里不要再去调 HAL_ADC_Start/GetValue，直接读 DMA 内存！
        uint16_t raw_vbus    = adc_regular_dma_buf[0]; // Rank1: 母线电压
        uint16_t raw_throttle = adc_regular_dma_buf[1]; // Rank2: 通道12 调速器

        /* 1. 母线电压换算与滤波 */
        float vbus_raw = (float)raw_vbus * VBUS_SCALE;
        vbus_voltage = vbus_voltage * 0.9f + vbus_raw * 0.1f;

        /* 2. 滑动变阻器电压换算与滤波 (假设ADC参考电压3.3V) */
        float v_throttle_raw = (float)raw_throttle * (3.3f / 4096.0f);
        // 一阶低通滤波，系数 0.1f 可减小电位器抖动带来的转速突变
        throttle_voltage = throttle_voltage * 0.95f + v_throttle_raw * 0.05f;

        /* 3. 电压 -> 目标转速 线性映射 (带死区) */
        if (throttle_voltage < THROTTLE_V_MIN) 
        {
            // 电位器处于最低位，目标转速设为 0
            throttle_target_rpm = 0.0f; 
        }
        else if (throttle_voltage > THROTTLE_V_MAX) 
        {
            // 超过最高电压，限制在最大转速
            throttle_target_rpm = THROTTLE_RPM_MAX; 
        }
        else 
        {
            // 线性映射计算
            float ratio = (throttle_voltage - THROTTLE_V_MIN) / (THROTTLE_V_MAX - THROTTLE_V_MIN);
            throttle_target_rpm = ratio * THROTTLE_RPM_MAX;
        }

        /* 4. 更新全局目标转速 */
        // 注意：这会覆盖你原来写死的 spd_target_rpm，使其完全由电位器控制
        spd_target_rpm = trunc(throttle_target_rpm);
				
				
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

        /* 3. VOFA+ 串口波形上传 */
        static uint16_t uart_tx_timeout = 0;
        if (vofa_tx_busy == 0) 
        {
            vofa_tx_busy = 1;
            uart_tx_timeout = 0; 
            
            vofa_tx_frame.ch_data[0] = spd_target_rpm;
            vofa_tx_frame.ch_data[1] = speed_rpm_est;
            vofa_tx_frame.ch_data[2] = iq_ref;
            vofa_tx_frame.ch_data[3] = i_q;
            vofa_tx_frame.ch_data[4] = id_mtpa_ref + id_fw_ref; // 最终 Id 目标
            vofa_tx_frame.ch_data[5] = i_d;
            
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
  * @brief 注入转换完成回调（20kHz） - 核心 FOC、偏置校准、弱磁
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2) return;
        
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); 
    
    /* ========================================================
     * 0. 读取 ADC 原始值
     * ======================================================== */
    uint16_t raw_rank1 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);  
    uint16_t raw_rank2 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);  

    /* ========================================================
     * 1. 上电偏置自动校准逻辑
     * ======================================================== */
    if (g_calib_state == 1)
    {
        calib_ia_sum += raw_rank1;
        calib_ic_sum += raw_rank2;
        calib_cnt++;

        if (calib_cnt >= CALIB_SAMPLES)
        {
            ia_offset = (float)calib_ia_sum / (float)CALIB_SAMPLES;
            ic_offset = (float)calib_ic_sum / (float)CALIB_SAMPLES;
            g_calib_state = 2; 
        }

        HAL_ADCEx_InjectedStart_IT(hadc); 
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); 
        return; 
    }

    /* ========================================================
     * 2. 正常运行：换算为物理电流
     * ======================================================== */
    ia = ((float)raw_rank1 - ia_offset) * CURRENT_SCALE;
    ic = ((float)raw_rank2 - ic_offset) * CURRENT_SCALE;
    ib = -ia - ic;

    /* ========================================================
     * 3. 故障锁存与过流检测
     * ======================================================== */
    if (g_fault_latch)
    {
        HAL_ADCEx_InjectedStart_IT(hadc);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); 
        return;
    }
    
    float i_max = fmaxf(fabsf(ia), fmaxf(fabsf(ib), fabsf(ic)));
    if (i_max > OC_HARD_LIMIT_A)
    {
        FOC_EMERGENCY_STOP();
        g_fault_code = FAULT_OC_HARD; g_fault_latch = 1;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);   
        HAL_ADCEx_InjectedStart_IT(hadc); return;
    }
    if (i_max > OC_SOFT_LIMIT_A)
    {
        if (++oc_debounce >= OC_DEBOUNCE_CNT)
        {
            FOC_EMERGENCY_STOP();
            if      (fabsf(ia) > OC_SOFT_LIMIT_A) g_fault_code = FAULT_OC_IA;
            else if (fabsf(ib) > OC_SOFT_LIMIT_A) g_fault_code = FAULT_OC_IB;
            else                                  g_fault_code = FAULT_OC_IC;
            g_fault_latch = 1;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);  
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_ADCEx_InjectedStart_IT(hadc); return;
        }
    }
    else if (oc_debounce > 0) oc_debounce--;

    /* ========================================================
     * 4. Clarke & Park 变换
     * ======================================================== */
    float sin_e = sinf(theta_e);
    float cos_e = cosf(theta_e);

    i_alpha = ia;
    i_beta  = (ia + 2.0f * ib) * 0.57735027f; 

    i_d =  i_alpha * cos_e + i_beta * sin_e;
    i_q = -i_alpha * sin_e + i_beta * cos_e;

    /* ========================================================
     * 5. VESC 非线性磁链观测器
     * ======================================================== */
    if (motor_state == STATE_IF) 
    {
        omega_e_est = 6.28318531f * freq_e; 
        speed_rpm_est = (omega_e_est / MOTOR_POLE_PAIRS) * (60.0f / 6.28318531f);
        obs_x1 = 0.0f; 
        obs_x2 = 0.0f;
    }
    else 
    {
        float L_ia = MOTOR_LS * i_alpha;
        float L_ib = MOTOR_LS * i_beta;
        
        float lambda_sq = MOTOR_FLUX_LINKAGE * MOTOR_FLUX_LINKAGE;
        float x1_ia = obs_x1 - L_ia;
        float x2_ib = obs_x2 - L_ib;
        float err = lambda_sq - (x1_ia * x1_ia + x2_ib * x2_ib);
        
        float gain = OBSERVER_GAIN;
        obs_x1 += (v_alpha_ref - MOTOR_RS * i_alpha + gain * err * x1_ia) * FOC_DT;
        obs_x2 += (v_beta_ref  - MOTOR_RS * i_beta  + gain * err * x2_ib) * FOC_DT;
        
        float flux_alpha = obs_x1 - L_ia;
        float flux_beta  = obs_x2 - L_ib;
        
        float phase_err = -flux_alpha * sin_e + flux_beta * cos_e;
        phase_err = phase_err / MOTOR_FLUX_LINKAGE;

        pll_integral += phase_err * PLL_KI * FOC_DT;
        if (pll_integral > 3000.0f)  pll_integral = 3000.0f;
        if (pll_integral < -3000.0f) pll_integral = -3000.0f;
        
        omega_e_est = pll_integral + phase_err * PLL_KP;
        
        if (isnan(omega_e_est) || isinf(omega_e_est)) omega_e_est = 0.0f;
        if (omega_e_est > 6000.0f)  omega_e_est = 6000.0f;
        if (omega_e_est < -6000.0f) omega_e_est = -6000.0f;
        
        speed_rpm_est = (omega_e_est / MOTOR_POLE_PAIRS) * (60.0f / 6.28318531f);
    }

    /* ========================================================
     * 6. 状态机切换与角度更新
     * ======================================================== */
    if (motor_state == STATE_IF)
    {
        if (freq_e < IF_START_FREQ) freq_e += IF_RAMP_RATE * FOC_DT;
        
        theta_if += 6.28318531f * freq_e * FOC_DT;
        if (theta_if >= 6.28318531f) theta_if -= 6.28318531f;
        theta_e = theta_if; 

        if (freq_e >= IF_START_FREQ)
        {
            speed_rpm_est = (freq_e * 60.0f) / MOTOR_POLE_PAIRS;
            integral_spd  = IF_START_IQ; 
            pll_integral = 6.28318531f * freq_e;
            theta_est    = theta_if; 
            
            integral_d = v_d - CURRENT_KP * (0.0f - i_d);
            integral_q = v_q - CURRENT_KP * (iq_ref - i_q);
            
            run_timer = 0; 
            motor_state = STATE_RUN;
        }
    }
    else 
    {
        theta_est += omega_e_est * FOC_DT;
        if (theta_est >= 6.28318531f) theta_est -= 6.28318531f;
        if (theta_est < 0.0f) theta_est += 6.28318531f;
        theta_e = theta_est;
    }

    /* ========================================================
     * 7. VESC 弱磁 (Field Weakening)
     * ======================================================== */
    float v_mag = sqrtf(v_d * v_d + v_q * v_q);
    float v_limit = vbus_voltage * 0.57735f; 
    float duty_ratio = v_mag / v_limit;
    
    float id_fw_target = 0.0f;
    if (duty_ratio > FW_VOLTAGE_THRESHOLD && motor_state == STATE_RUN && ctrl_state == CTRL_SPEED) 
    {
        float ratio = (duty_ratio - FW_VOLTAGE_THRESHOLD) / (1.0f - FW_VOLTAGE_THRESHOLD);
        if (ratio > 1.0f) ratio = 1.0f;
        
        id_fw_target = -ratio * FW_MAX_CURRENT;
        
        /* 防失控回退 */
        float i_err_backoff = (i_q - iq_ref) / FW_MAX_CURRENT;
        if (i_err_backoff > 0.0f) {
            id_fw_target += i_err_backoff * FW_MAX_CURRENT;
            if (id_fw_target > 0.0f) id_fw_target = 0.0f;
        }
    }
    
    /* 弱磁平滑斜坡 */
    if (id_fw_target < id_fw_ref) {
        id_fw_ref -= FW_RAMP_STEP;
        if (id_fw_ref < id_fw_target) id_fw_ref = id_fw_target;
    } else if (id_fw_target > id_fw_ref) {
        id_fw_ref += FW_RAMP_STEP;
        if (id_fw_ref > id_fw_target) id_fw_ref = id_fw_target;
    }

    /* MTPA 与 弱磁 叠加 (取绝对值最大的那个) */
    float id_ref = UTILS_MAX_ABS(id_mtpa_ref, id_fw_ref);

    /* ========================================================
     * 8. 电流环 PI
     * ======================================================== */
    sin_e = sinf(theta_e);
    cos_e = cosf(theta_e);
    i_d =  i_alpha * cos_e + i_beta * sin_e;
    i_q = -i_alpha * sin_e + i_beta * cos_e;

    float err_d = id_ref - i_d; 
    float err_q = iq_ref - i_q;

    v_d = CURRENT_KP * err_d + integral_d;
    v_q = CURRENT_KP * err_q + integral_q;

    #if USE_DECOUPLING
        v_d += -omega_e_est * MOTOR_LS * i_q;
        v_q +=  omega_e_est * MOTOR_LS * i_d;
    #endif

    /* 积分与抗饱和 */
    if (v_mag > v_limit) {
        float scale = v_limit / v_mag;
        v_d *= scale;
        v_q *= scale;
    } else {
        integral_d += CURRENT_KI * err_d * FOC_DT;
        integral_q += CURRENT_KI * err_q * FOC_DT;
    }
    
    if (integral_d > v_limit)  integral_d = v_limit;
    if (integral_d < -v_limit) integral_d = -v_limit;
    if (integral_q > v_limit)  integral_q = v_limit;
    if (integral_q < -v_limit) integral_q = -v_limit;

    /* ========================================================
     * 9. 反 Park 变换 & SVPWM
     * ======================================================== */
    v_alpha_ref = v_d * cos_e - v_q * sin_e;
    v_beta_ref  = v_d * sin_e + v_q * cos_e;

    float va_ref = v_alpha_ref;
    float vb_ref = -0.5f * v_alpha_ref + 0.86602540f * v_beta_ref;
    float vc_ref = -0.5f * v_alpha_ref - 0.86602540f * v_beta_ref;

    float v_max_svpwm = va_ref, v_min_svpwm = va_ref;
    if (vb_ref > v_max_svpwm) v_max_svpwm = vb_ref;
    if (vc_ref > v_max_svpwm) v_max_svpwm = vc_ref;
    if (vb_ref < v_min_svpwm) v_min_svpwm = vb_ref;
    if (vc_ref < v_min_svpwm) v_min_svpwm = vc_ref;
    
    float v_common = -(v_max_svpwm + v_min_svpwm) * 0.5f;
    va_ref += v_common;
    vb_ref += v_common;
    vc_ref += v_common;

    /* ========================================================
     * 10. 占空比计算与硬件寄存器更新
     * ======================================================== */
    float vbus_safe = (vbus_voltage < 10.0f) ? 10.0f : vbus_voltage;

    float duty_a = 0.5f + va_ref / vbus_safe;
    float duty_b = 0.5f + vb_ref / vbus_safe;
    float duty_c = 0.5f + vc_ref / vbus_safe;

    if (duty_a < 0.0f) duty_a = 0.0f; else if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f; else if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f; else if (duty_c > 1.0f) duty_c = 1.0f;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a * (float)htim1.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_b * (float)htim1.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_c * (float)htim1.Instance->ARR));

//    HAL_ADCEx_InjectedStart_IT(hadc);
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
