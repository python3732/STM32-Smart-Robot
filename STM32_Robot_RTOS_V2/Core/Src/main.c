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
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
// 定义 PWM 计数器和速度变量
volatile int pwm_count = 0;    // 计数器 0-100
volatile int speed_left = 0;   // 左轮速度 0-100
volatile int speed_right = 0;  // 右轮速度 0-100
extern volatile int car_mode;
uint8_t rx_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  
  /* USER CODE BEGIN 2 */

  // ==========================================
  // 1. 启动基础定时器和蓝牙
  // ==========================================
  HAL_TIM_Base_Start_IT(&htim6); // 启动电机 SoftPWM 心脏
  HAL_TIM_Base_Start(&htim2);    // 启动超声波秒表
  HAL_UART_Receive_IT(&huart3, &rx_data, 1); // 启动蓝牙接收

  // ==========================================
  // 2. 核心配置：强行设置 PD12 为“复用推挽输出”
  // ==========================================
  __HAL_RCC_GPIOD_CLK_ENABLE();  // 开启 GPIOD 时钟
  __HAL_RCC_AFIO_CLK_ENABLE();   // 开启 AFIO 时钟

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_12;            // 指定 PD12
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // ⚠️必须是 AF_PP
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速模式
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);       // 应用配置

  // ==========================================
  // 3. 开启 TIM4 完全重映射并启动 PWM
  // ==========================================
  __HAL_AFIO_REMAP_TIM4_ENABLE();             // 开启重映射
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   // 启动 PWM 信号

  // ==========================================
  // 4. 舵机上电自检 (安全角度版)
  // ==========================================
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);  // 左看
  HAL_Delay(500);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);  // 右看
  HAL_Delay(500);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500);  // 回正
  HAL_Delay(200);

  // ==========================================
  // 5. 初始化全局变量
  // ==========================================
  speed_left = 50;
  speed_right = 50;
  car_mode = 0; // 默认遥控模式

  // 设置默认电机方向 (往前)
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);    // AIN1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // AIN2
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);    // BIN1
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);  // BIN2

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 1. 告诉 main.c，队列句柄是在 freertos.c 里定义的
extern osMessageQId BluetoothQueueHandle;

// 2. 串口接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3) 
    {
        osMessagePut(BluetoothQueueHandle, rx_data, 0);
        HAL_UART_Receive_IT(&huart3, &rx_data, 1);
    }
}

// 3. 微秒级延时函数
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0); 
    while(__HAL_TIM_GET_COUNTER(&htim2) < us); 
}

// 4. 获取距离函数
float Get_Distance(void)
{
    float distance = 0;
    uint32_t time_cnt = 0;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); 
    delay_us(20); 
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);

    uint32_t timeout = 1000000;
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
        timeout--;
        if(timeout == 0) return 999.0; 
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0); 

    timeout = 1000000;
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
    {
        timeout--;
        if(timeout == 0) return 999.0;
    }

    time_cnt = __HAL_TIM_GET_COUNTER(&htim2);
    distance = time_cnt / 58.0f;

    return distance;
}

// 🔥🔥 电机驱动中断 (心脏跳动逻辑) 🔥🔥
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }

// 2. 🔥 电机 SoftPWM 驱动 (修复引脚定义版)
  if (htim->Instance == TIM6)
  {
      pwm_count++;
      if(pwm_count >= 100) pwm_count = 0;

      // === 左电机 (EN_A) ===
      // 使用宏定义 EN_A_Pin，而不是猜的数字
      if(pwm_count < speed_left) 
          HAL_GPIO_WritePin(GPIOD, EN_A_Pin, GPIO_PIN_SET); 
      else 
          HAL_GPIO_WritePin(GPIOD, EN_A_Pin, GPIO_PIN_RESET);

      // === 右电机 (EN_B) ===
      // 使用宏定义 EN_B_Pin
      if(pwm_count < speed_right) 
          HAL_GPIO_WritePin(GPIOD, EN_B_Pin, GPIO_PIN_SET); 
      else 
          HAL_GPIO_WritePin(GPIOD, EN_B_Pin, GPIO_PIN_RESET);
  }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
