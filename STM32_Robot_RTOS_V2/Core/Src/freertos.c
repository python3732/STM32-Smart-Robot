/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  * Author             : Leilhaofan (Resume Aligned Architecture Version)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "i2c.h" 
#include <stdlib.h> // 为了用 atof 和 atoi
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
/* USER CODE BEGIN Variables */
extern volatile int speed_left;
extern volatile int speed_right;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim4; 
extern float Get_Distance(void); 

// === 全局状态 ===
volatile int car_mode = 0; 
float global_gyro_offset = 0; 

// 🔥🔥 全局调参变量 (蓝牙可改) 🔥🔥
volatile float Kp_Track = 1.1f;  
volatile float Kd_Track = 0.2f;  
volatile int base_speed = 40;    

// ==========================================================
// 【架构设计】: 任务间共享数据 (Logic -> Motor)
// ==========================================================
volatile int shared_tracking_error = 0;   // 循迹误差
volatile float shared_angle_z = 0;        // 陀螺仪角度
volatile float shared_target_heading = 0; // 目标航向

// 控制权覆写标志：当 Logic 任务正在执行耗时的避障转弯时，接管电机控制
volatile int logic_motor_override = 0;    
volatile int override_speed_l = 0;
volatile int override_speed_r = 0;
/* USER CODE END Variables */
osThreadId Task_MotorHandle;
osThreadId Task_LogicHandle;
osThreadId Task_ComHandle;
osMessageQId BluetoothQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Servo_Turn(int angle);
void MPU_Init(void);
int16_t MPU_Get_Gyro_Z(void);
void Auto_Turn(float target_angle); 
int Get_Tracking_Error(void); 
void HAL_Motor_Mapping(int motor_l, int motor_r); // 新增：HAL层映射
/* USER CODE END FunctionPrototypes */

void StartMotorTask(void const * argument);
void StartLogicTask(void const * argument);
void StartComTask(void const * argument);

void MX_FREERTOS_Init(void); 

/* GetIdleTaskMemory prototype */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  // [简历对标]：利用消息队列实现指令解耦
  osMessageQDef(BluetoothQueue, 16, uint8_t);
  BluetoothQueueHandle = osMessageCreate(osMessageQ(BluetoothQueue), NULL);

  // [简历对标]：构建三层任务架构
  // 1. 驱动/控制任务 (High Priority) - 确保 10ms 硬实时
  osThreadDef(Task_Motor, StartMotorTask, osPriorityHigh, 0, 256);
  Task_MotorHandle = osThreadCreate(osThread(Task_Motor), NULL);

  // 2. 逻辑/感知任务 (Normal Priority) - 处理传感器与决策
  osThreadDef(Task_Logic, StartLogicTask, osPriorityNormal, 0, 1024);
  Task_LogicHandle = osThreadCreate(osThread(Task_Logic), NULL);

  // 3. 通信任务 (Below Normal Priority) - 解析蓝牙
  osThreadDef(Task_Com, StartComTask, osPriorityBelowNormal, 0, 256);
  Task_ComHandle = osThreadCreate(osThread(Task_Com), NULL);
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  驱动层任务 (Motor Task) 
  * [简历对标]：确保 10ms 运动控制周期的硬实时响应
  */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  // 使用 FreeRTOS 原生绝对延时变量
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 严格 10ms 周期

  int last_tracking_error = 0;
  float Kp_Obstacle = 2.0f; // 避障模式直行纠偏

  // 初始化上次唤醒时间
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
      // --- A. 检查控制权是否被 Logic 任务接管 ---
      if (logic_motor_override == 1)
      {
          HAL_Motor_Mapping(override_speed_l, override_speed_r);
      }
      // --- B. 正常闭环控制逻辑 ---
      else 
      {
          if (car_mode == 1) 
          {
              // [模式1] 避障：基于陀螺仪走直线
              float error = shared_angle_z - shared_target_heading;
              int turn_adjust = (int)(error * Kp_Obstacle);

              int motor_l = 50 - turn_adjust;
              int motor_r = 50 + turn_adjust;

              HAL_Motor_Mapping(motor_l, motor_r);
          }
          else if (car_mode == 2)
          {
              // [模式2] 循迹：[简历对标] PD闭环结合非线性死区穿越补偿
              int current_error = shared_tracking_error;
              
              // 1. PD 计算
              int pid_out = (int)(Kp_Track * current_error + Kd_Track * (current_error - last_tracking_error));
              last_tracking_error = current_error;

              // 2. 计算速度
              int motor_l = base_speed - pid_out; 
              int motor_r = base_speed + pid_out;
              
              // 3. 非线性死区穿越补偿 (Dead-zone compensation)
              int dead_zone = 25;     
              int violent_kick = -35; // 强行越过电机静摩擦力死区
              
              if (motor_l < dead_zone && motor_l > -dead_zone) motor_l = violent_kick; 
              if (motor_r < dead_zone && motor_r > -dead_zone) motor_r = violent_kick; 
              
              // 4. 执行映射
              HAL_Motor_Mapping(motor_l, motor_r);
          }
          else if (car_mode == 0 || car_mode == 3 || car_mode == 4)
          {
               // 其他模式的动作由 Logic 覆写，这里保持0输出
               HAL_Motor_Mapping(0, 0);
          }
      }

      // [核心高光]：条件编译保证完美兼容
      // 如果 CubeMX 中启用了 vTaskDelayUntil (INCLUDE_vTaskDelayUntil == 1)，则使用绝对硬实时
      // 否则回退到普通的 osDelay() 软实时，防止编译报错 "Undefined symbol"
#if (INCLUDE_vTaskDelayUntil == 1)
      vTaskDelayUntil(&xLastWakeTime, xFrequency); 
#else
      osDelay(10); 
#endif
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartLogicTask */
/**
  * @brief 逻辑感知任务 (Logic Task)
  * [简历对标]：采用重心法处理传感器数据，决策自动避障
  */
/* USER CODE END Header_StartLogicTask */
void StartLogicTask(void const * argument)
{
  /* USER CODE BEGIN StartLogicTask */
  
  // 避障模式变量
  volatile float dis_front = 0, dis_left = 0, dis_right = 0;
  float safe_distance = 15.0f;  

  // 时间积分变量
  uint32_t last_tick = 0, now_tick = 0;
  float dt = 0;

  // 1. 初始化传感器
  MPU_Init();
  osDelay(500); 

  char msg[64] = "System Ready! Calibrating...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

  // 陀螺仪校准
  float sum = 0;
  for(int i = 0; i < 100; i++) {
      sum += MPU_Get_Gyro_Z();
      osDelay(5); 
  }
  global_gyro_offset = sum / 100.0f; 

  last_tick = HAL_GetTick();

  for(;;)
  {
      // --- 1. 更新传感器数据 (陀螺仪积分) ---
      int16_t gyro_z_raw = MPU_Get_Gyro_Z();
      float gyro_z_corrected = gyro_z_raw - global_gyro_offset;
      if(gyro_z_corrected > -50 && gyro_z_corrected < 50) gyro_z_corrected = 0;

      now_tick = HAL_GetTick();
      dt = (now_tick - last_tick) / 1000.0f;
      last_tick = now_tick;
      if(dt > 0.1f) dt = 0.01f;

      shared_angle_z += (gyro_z_corrected / 131.0f) * dt; 

      // --- 2. 状态机决策 ---
      if (car_mode == 1) // 自动避障
      {
          dis_front = Get_Distance(); // 内部含超时保险丝

          if (dis_front > 25.0f) {
              logic_motor_override = 0; // 释放控制权给 Motor 任务的 PD 算法
          } else {
              // 遇到障碍：开启 Override，接管电机控制
              logic_motor_override = 1; 
              override_speed_l = 0; override_speed_r = 0;
              osDelay(500);

              // 摇头侦察 (调用相对延时，Motor任务依然在后台10ms运行，只是输出为0)
              Servo_Turn(160); osDelay(600); dis_left = Get_Distance();
              Servo_Turn(20);  osDelay(600); dis_right = Get_Distance();
              Servo_Turn(90);  osDelay(300);

              if (dis_left < safe_distance && dis_right < safe_distance) {
                  // 死胡同退后
                  override_speed_l = -50; override_speed_r = -50; osDelay(500);
                  override_speed_l = 0;   override_speed_r = 0;   osDelay(200);
                  Auto_Turn(-90.0f); 
                  shared_angle_z -= 90.0f; shared_target_heading -= 90.0f;
              }
              else if (dis_left > dis_right) {
                  Auto_Turn(90.0f); 
                  shared_angle_z += 90.0f; shared_target_heading += 90.0f;
              } else {
                  Auto_Turn(-90.0f); 
                  shared_angle_z -= 90.0f; shared_target_heading -= 90.0f;
              }
              last_tick = HAL_GetTick(); // 重置时间
          }
      }
      else if (car_mode == 2) // PID循迹
      {
          logic_motor_override = 0; // 控制权交给 Motor 任务
          // [简历对标]：采用重心法处理传感器数据
          shared_tracking_error = Get_Tracking_Error();
      }
      
      // Logic任务只负责决策更新，延时随意，不影响底层硬实时
      osDelay(10); 
  }
  /* USER CODE END StartLogicTask */
}

/* USER CODE BEGIN Header_StartComTask */
/* [简历对标]：利用消息队列实现指令解耦 */
/* USER CODE END Header_StartComTask */
void StartComTask(void const * argument)
{
  /* USER CODE BEGIN StartComTask */
  uint8_t rx_byte;
  osEvent evt;
  static char cmd_buffer[20]; 
  static int cmd_index = 0;
  char echo_msg[64]; // 恢复变量用于串口反馈

  for(;;)
  {
    // 阻塞等待消息队列
    evt = osMessageGet(BluetoothQueueHandle, osWaitForever);
    if (evt.status == osEventMessage)
    {
        rx_byte = (uint8_t)evt.value.v;

        // 1. 单字符指令
        switch (rx_byte)
        {
            case 'A': car_mode = 1; break; 
            case 'M': car_mode = 0; logic_motor_override = 1; override_speed_l=0; override_speed_r=0; break; 
            case 'T': car_mode = 2; break; 
            case 'S': if(car_mode==0) { override_speed_l=0; override_speed_r=0; } break;
            case 'G': if(car_mode==0) { override_speed_l=50; override_speed_r=50; } break;
            case 'B': if(car_mode==0) { override_speed_l=-50; override_speed_r=-50; } break;
            case 'L': if(car_mode==0) { override_speed_l=0; override_speed_r=60; } break;
            case 'R': if(car_mode==0) { override_speed_l=60; override_speed_r=0; } break;
            case 'U': Servo_Turn(160); break;
            case 'I': Servo_Turn(90); break;
            case 'O': Servo_Turn(20); break;
        }

        // 2. 调参指令解析 (解决 echo_msg 报错并增加蓝牙数据回传反馈)
        if (rx_byte == '#' || rx_byte == '\n') {
            cmd_buffer[cmd_index] = '\0'; 
            if (cmd_buffer[0] == 'P' || cmd_buffer[0] == 'p') {
                Kp_Track = atof(&cmd_buffer[1]);
                sprintf(echo_msg, "Set P = %.2f OK\r\n", Kp_Track);
                HAL_UART_Transmit(&huart3, (uint8_t*)echo_msg, strlen(echo_msg), 100);
            }
            else if (cmd_buffer[0] == 'D' || cmd_buffer[0] == 'd') {
                Kd_Track = atof(&cmd_buffer[1]);
                sprintf(echo_msg, "Set D = %.2f OK\r\n", Kd_Track);
                HAL_UART_Transmit(&huart3, (uint8_t*)echo_msg, strlen(echo_msg), 100);
            }
            else if (cmd_buffer[0] == 'V' || cmd_buffer[0] == 'v') {
                base_speed = atoi(&cmd_buffer[1]);
                sprintf(echo_msg, "Set Speed = %d OK\r\n", base_speed);
                HAL_UART_Transmit(&huart3, (uint8_t*)echo_msg, strlen(echo_msg), 100);
            }
            cmd_index = 0; memset(cmd_buffer, 0, 20);
        } else {
            if (rx_byte != 'A' && rx_byte != 'M' && rx_byte != 'T' && rx_byte != 'S' && rx_byte != 'G' && rx_byte != '\r') {
                if (cmd_index < 19) cmd_buffer[cmd_index++] = rx_byte;
            }
        }
    }
  }
  /* USER CODE END StartComTask */
}

/* USER CODE BEGIN 4 */

/**
 * @brief [简历对标]：通过 HAL 层映射实现软件级通道互换以修正硬件接线错误
 */
void HAL_Motor_Mapping(int motor_l, int motor_r)
{
    // 1. 安全限幅
    if(motor_l > 90) motor_l = 90; if(motor_l < -90) motor_l = -90;
    if(motor_r > 90) motor_r = 90; if(motor_r < -90) motor_r = -90;

    // 2. 软件级通道映射 (解决硬件排线接反问题)
    // 假设原本设计：L控制AIN，R控制BIN。
    // 硬件接线时：L接到了BIN，R接到了AIN。
    // 这里利用软件解耦，将逻辑信号重定向到正确的物理引脚。
    
    // --- 逻辑左轮 映射到 物理BIN (G9/D6) ---
    if(motor_l >= 0) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);   // 正转
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
        speed_left = motor_l; // 传入全局变量给 SoftPWM
    } else {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET); // 反转
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
        speed_left = -motor_l;
    }

    // --- 逻辑右轮 映射到 物理AIN (D0/C11) ---
    if(motor_r >= 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);    // 正转
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
        speed_right = motor_r;
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);  // 反转
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
        speed_right = -motor_r;
    }
}

// ============================================
// 下方保留原有的基础函数 (舵机、MPU、重心法、转弯等)
// ============================================

void Servo_Turn(int angle)
{
    if(angle < 10) angle = 10;
    if(angle > 170) angle = 170;
    int pwm_val = 500 + (angle * 2000 / 180);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_val);
}

void MPU_Init(void)
{
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

int16_t MPU_Get_Gyro_Z(void)
{
    uint8_t data[2] = {0, 0}; 
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x47, I2C_MEMADD_SIZE_8BIT, data, 2, 10);
    if (status != HAL_OK)
    {
        HAL_I2C_DeInit(&hi2c1); HAL_I2C_Init(&hi2c1); return 0; 
    }
    return (data[0] << 8) | data[1];
}

void Auto_Turn(float target_angle)
{
    float accumulated_angle = 0;
    float dt = 0;
    uint32_t last_tick = HAL_GetTick();
    float target_abs = (target_angle > 0) ? target_angle : -target_angle;

    // [简历对标]：软件保险丝机制 (Timeout) 防止任务死锁
    uint32_t start_time = HAL_GetTick(); 

    float stop_offset = 35.0f; 
    if (target_abs < stop_offset) stop_offset = 0;

    int current_speed = 60;

    // 接管控制权，直接调用映射函数
    logic_motor_override = 1; 

    if(target_angle > 0) { 
        override_speed_l = -current_speed; override_speed_r = current_speed; // 左转
    } else { 
        override_speed_l = current_speed; override_speed_r = -current_speed; // 右转
    }

    while(accumulated_angle < (target_abs - stop_offset)) 
    {
        // Timeout 保护：2000ms 强制退出
        if ((HAL_GetTick() - start_time) > 2000) break;

        int16_t gyro = MPU_Get_Gyro_Z();
        float speed = gyro - global_gyro_offset; 
        if(speed > -50 && speed < 50) speed = 0;
        
        uint32_t now_tick = HAL_GetTick();
        dt = (now_tick - last_tick) / 1000.0f;
        last_tick = now_tick;
        if(dt > 0.1f) dt = 0.01f;

        float angle_step = (speed / 131.0f) * dt; 
        if(angle_step < 0) angle_step = -angle_step;
        accumulated_angle += angle_step;

        // 减速缓冲
        if ((target_abs - accumulated_angle) < 40.0f) {
            override_speed_l = (target_angle > 0) ? -40 : 40;
            override_speed_r = (target_angle > 0) ? 40 : -40;
        }
        
        osDelay(5); 
    }

    override_speed_l = 0; override_speed_r = 0;
    osDelay(800); 
}

// [简历对标]：重心法处理传感器数据
int Get_Tracking_Error(void)
{
    uint8_t sensor = (uint8_t)(GPIOF->IDR & 0x00FF);
    static int last_known_error = 0;
    
    int error_sum = 0;   
    int sensor_count = 0; 
    
    if (sensor & 0x01) { error_sum -= 40; sensor_count++; } 
    if (sensor & 0x02) { error_sum -= 30; sensor_count++; } 
    if (sensor & 0x04) { error_sum -= 20; sensor_count++; } 
    if (sensor & 0x08) { error_sum -= 10; sensor_count++; } 
    if (sensor & 0x10) { error_sum += 10; sensor_count++; } 
    if (sensor & 0x20) { error_sum += 20; sensor_count++; } 
    if (sensor & 0x40) { error_sum += 30; sensor_count++; } 
    if (sensor & 0x80) { error_sum += 40; sensor_count++; } 
    
    if (sensor_count == 0) return last_known_error; 

    int final_error = error_sum / sensor_count; // 求重心
    last_known_error = final_error;
    return final_error;
}
/* USER CODE END 4 */
