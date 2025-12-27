/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  * Author             : Gemini & User (Final Version)
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
// 0: 遥控 (Manual)
// 1: 避障 (Auto Obstacle)
// 2: 循迹 (PID Tracking)
// 3: 调试 (Sensor Debug)
volatile int car_mode = 0; 
float global_gyro_offset = 0; 

// 🔥🔥 全局调参变量 (蓝牙可改) 🔥🔥
volatile float Kp_Track = 7.0f;  // P: 转向力度
volatile float Kd_Track = 2.5f;  // D: 抑制震荡
volatile int base_speed = 30;    // V: 基础速度
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

  osMessageQDef(BluetoothQueue, 16, uint8_t);
  BluetoothQueueHandle = osMessageCreate(osMessageQ(BluetoothQueue), NULL);

  osThreadDef(Task_Motor, StartMotorTask, osPriorityHigh, 0, 256);
  Task_MotorHandle = osThreadCreate(osThread(Task_Motor), NULL);

  // 1024 栈空间，防止溢出
  osThreadDef(Task_Logic, StartLogicTask, osPriorityNormal, 0, 1024);
  Task_LogicHandle = osThreadCreate(osThread(Task_Logic), NULL);

  osThreadDef(Task_Com, StartComTask, osPriorityBelowNormal, 0, 256);
  Task_ComHandle = osThreadCreate(osThread(Task_Com), NULL);
}

/* USER CODE BEGIN Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

/* USER CODE BEGIN Header_StartLogicTask */
/* USER CODE BEGIN StartLogicTask */
void StartLogicTask(void const * argument)
{
  // === 1. 通用变量定义 ===
  int16_t gyro_z_raw = 0;
  float angle_z = 0;
  char msg[64];
  int i; 

  // === 2. 避障模式变量 ===
  volatile float dis_front = 0;
  volatile float dis_left = 0;
  volatile float dis_right = 0;
  float safe_distance = 15.0f;    
  float target_heading = 0; 
  float Kp_Obstacle = 2.0f; // 避障走直线用的 P

  // === 3. 循迹模式变量 ===
  int tracking_error = 0;
  int last_tracking_error = 0;
  // 注意：Kp_Track, Kd_Track, base_speed 现在是全局变量，在文件顶部定义
  // 这样 StartComTask 才能通过蓝牙修改它们

  // === 4. 积分时间变量 ===
  uint32_t last_tick = 0;
  uint32_t now_tick = 0;
  float dt = 0;
  float scale_factor = 1.0f; 

  // === 5. 初始化流程 ===
  MPU_Init();
  osDelay(500); 

  sprintf(msg, "System Ready! Calibrating...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

  // 陀螺仪零偏校准
  float sum = 0;
  for(i = 0; i < 100; i++)
  {
      sum += MPU_Get_Gyro_Z();
      osDelay(5); 
  }
  global_gyro_offset = sum / 100.0f; 

  sprintf(msg, "Offset: %.1f. Waiting for Cmd.\r\n", global_gyro_offset);
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
  
  last_tick = HAL_GetTick();

  // === 6. 主循环 ===
  for(;;)
  {
    // ---------------------------
    // A. 陀螺仪后台持续积分
    // ---------------------------
    gyro_z_raw = MPU_Get_Gyro_Z();
    float gyro_z_corrected = gyro_z_raw - global_gyro_offset;
    if(gyro_z_corrected > -50 && gyro_z_corrected < 50) gyro_z_corrected = 0;

    now_tick = HAL_GetTick();
    dt = (now_tick - last_tick) / 1000.0f;
    last_tick = now_tick;
    if(dt > 0.1f) dt = 0.01f;

    angle_z += (gyro_z_corrected / 131.0f) * dt * scale_factor; 

    // ---------------------------
    // B. 模式状态机
    // ---------------------------
    if (car_mode == 0)
    {
        // === 模式 0: 遥控待机 ===
        // 什么都不做，完全听 StartComTask 的蓝牙指令
    }
    else if (car_mode == 1)
    {
        // ===========================
        //      模式 1: 自动避障
        // ===========================
        dis_front = Get_Distance();

        if (dis_front > 25.0f)
        {
            // === 直行 + 陀螺仪纠偏 ===
            float error = angle_z - target_heading;
            int turn_adjust = (int)(error * Kp_Obstacle);

            int left_motor = 50 - turn_adjust;
            int right_motor = 50 + turn_adjust;

            // 限幅
            if(left_motor > 90) left_motor = 90; if(left_motor < 0) left_motor = 0;
            if(right_motor > 90) right_motor = 90; if(right_motor < 0) right_motor = 0;

            speed_left = left_motor; speed_right = right_motor;
            
            // 前进方向
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);    
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); 
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);    
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);  
        }
        else
        {
            // === 遇到障碍处理 ===
            speed_left = 0; speed_right = 0;
            osDelay(500);

            // 摇头侦察
            Servo_Turn(160); osDelay(600); dis_left = Get_Distance();
            Servo_Turn(20);  osDelay(600); dis_right = Get_Distance();
            Servo_Turn(90);  osDelay(300);

            if (dis_left < safe_distance && dis_right < safe_distance)
            {
                // === 死胡同：倒车掉头 ===
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); // 左退
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET); // 右退
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
                
                speed_left = 50; speed_right = 50; HAL_Delay(500);
                speed_left = 0; speed_right = 0; HAL_Delay(200);

                // 右转 90 度 + 更新坐标
                Auto_Turn(-90.0f); 
                angle_z -= 90.0f; target_heading -= 90.0f; last_tick = HAL_GetTick();
            }
            else if (dis_left > dis_right)
            {
                // 左转 90 度
                Auto_Turn(90.0f);
                angle_z += 90.0f; target_heading += 90.0f; last_tick = HAL_GetTick();
            }
            else
            {
                // 右转 90 度
                Auto_Turn(-90.0f);
                angle_z -= 90.0f; target_heading -= 90.0f; last_tick = HAL_GetTick();
            }
        }
    }
    else if (car_mode == 2)
    {
        // ===========================
        //      模式 2: 智能循迹 (含动态减速)
        // ===========================
        
        // 1. 获取误差
        int raw_error = Get_Tracking_Error();
        
        // (可选) 可以在这里加简单的消抖，但通常直接用反应最快
        tracking_error = raw_error; 

        // 2. PID 计算
        // pid_out 代表“转向力度”
        int pid_out = (int)(Kp_Track * tracking_error + Kd_Track * (tracking_error - last_tracking_error));
        last_tracking_error = tracking_error;

        // 🔥🔥【核心优化】动态基础速度 🔥🔥
        // 原理：如果转向力度(pid_out)很大，说明弯很急，必须减速！
        // 算法：实际基准速度 = 设定基准速度 - (转向力度 * 系数)
        // 系数 0.5 意味着：如果 pid_out 是 40 (急转)，速度就减掉 20。
        int dynamic_base = base_speed - (abs(pid_out) / 2);
        
        // 兜底保护：速度不能减到 0 以下，否则车就停在弯道上了
        // 给 15 作为最低蠕动速度
        if (dynamic_base < 15) dynamic_base = 15; 

        // 3. 计算最终电机速度
        // 左轮 = 动态基准 - 转向力度
        // 右轮 = 动态基准 + 转向力度
        // (注：如果之前的方向反了，请交换这里的加减号)
        int motor_l = dynamic_base - pid_out; 
        int motor_r = dynamic_base + pid_out;
        
        // 4. 安全限幅
        if(motor_l > 90) motor_l = 90; if(motor_l < -90) motor_l = -90;
        if(motor_r > 90) motor_r = 90; if(motor_r < -90) motor_r = -90;
        
        // 5. 写入电机 (支持原地反转 Tank Turn)
        // 如果 motor_l 是负数，说明需要左轮倒转，辅助急转弯
        
        // --- 左轮控制 ---
        if(motor_l >= 0) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);    // 正转
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); 
            speed_left = motor_l;
        } else {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);  // 反转
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); 
            speed_left = -motor_l; // 取绝对值
        }

        // --- 右轮控制 ---
        if(motor_r >= 0) {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);    // 正转
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
            speed_right = motor_r;
        } else {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);  // 反转
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
            speed_right = -motor_r; // 取绝对值
        }
    }
    else if (car_mode == 3)
    {
        // ===========================
        //      模式 3: 传感器调试
        // ===========================
        // 发送 8路传感器原始状态，方便你调节电位器屏蔽地板缝隙
        char binary_str[10];
        uint8_t sensor = (uint8_t)(GPIOF->IDR & 0x00FF);
        
        for(int j=0; j<8; j++)
        {
            // 如果第 (7-j) 位是 1，就填 '1'，否则填 '0'
            if (sensor & (1 << (7-j))) 
                binary_str[j] = '1';
            else 
                binary_str[j] = '0';
        }
        binary_str[8] = '\n'; 
        binary_str[9] = '\0';
        
        HAL_UART_Transmit(&huart3, (uint8_t*)binary_str, 9, 100);
        osDelay(500); // 半秒发一次，别刷屏太快
    }

    osDelay(10); // 10ms 调度周期
  }
}
/* USER CODE END StartLogicTask */

/* USER CODE BEGIN Header_StartComTask */
void StartComTask(void const * argument)
{
  uint8_t rx_byte;
  osEvent evt;
  
  static char cmd_buffer[20]; 
  static int cmd_index = 0;
  char echo_msg[64]; 

  for(;;)
  {
    evt = osMessageGet(BluetoothQueueHandle, osWaitForever);
    if (evt.status == osEventMessage)
    {
        rx_byte = (uint8_t)evt.value.v;

        // 1. 单字符指令
        switch (rx_byte)
        {
            case 'A': car_mode = 1; break; // 避障
            case 'M': car_mode = 0; speed_left = 0; speed_right = 0; break; // 遥控
            case 'T': car_mode = 2; break; // 循迹
            case 'D': car_mode = 3; speed_left = 0; speed_right = 0; break; // 调试
            case '?': // 查询参数
                sprintf(echo_msg, "P=%.1f, D=%.1f, V=%d\r\n", Kp_Track, Kd_Track, base_speed);
                HAL_UART_Transmit(&huart3, (uint8_t*)echo_msg, strlen(echo_msg), 100);
                break;

            case 'S': if(!car_mode) { speed_left = 0; speed_right = 0; } break;
            case 'G': if(!car_mode) { 
                    speed_left = 50; speed_right = 50; 
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
                } break;
            case 'B': if(!car_mode) { 
                    speed_left = 50; speed_right = 50; 
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
                } break;
            case 'L': if(!car_mode) { speed_left = 0; speed_right = 60; } break;
            case 'R': if(!car_mode) { speed_left = 60; speed_right = 0; } break;
            case 'U': Servo_Turn(160); break;
            case 'I': Servo_Turn(90); break;
            case 'O': Servo_Turn(20); break;
        }

        // 2. 调参指令解析 (以 # 结尾)
        if (rx_byte == '#' || rx_byte == '\n')
        {
            cmd_buffer[cmd_index] = '\0'; 
            if (cmd_buffer[0] == 'P' || cmd_buffer[0] == 'p') {
                Kp_Track = atof(&cmd_buffer[1]);
                sprintf(echo_msg, "Set P = %.2f OK\r\n", Kp_Track);
            }
            else if (cmd_buffer[0] == 'D' || cmd_buffer[0] == 'd') {
                Kd_Track = atof(&cmd_buffer[1]);
                sprintf(echo_msg, "Set D = %.2f OK\r\n", Kd_Track);
            }
            else if (cmd_buffer[0] == 'V' || cmd_buffer[0] == 'v') {
                base_speed = atoi(&cmd_buffer[1]);
                sprintf(echo_msg, "Set Speed = %d OK\r\n", base_speed);
            }
            HAL_UART_Transmit(&huart3, (uint8_t*)echo_msg, strlen(echo_msg), 100);
            cmd_index = 0; memset(cmd_buffer, 0, 20);
        }
        else
        {
            if (rx_byte != 'A' && rx_byte != 'M' && rx_byte != 'T' && 
                rx_byte != 'S' && rx_byte != 'G' && rx_byte != '?' && rx_byte != '\r')
            {
                if (cmd_index < 19) cmd_buffer[cmd_index++] = rx_byte;
            }
        }
    }
  }
}
/* USER CODE END StartComTask */

/* USER CODE BEGIN 4 */

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
    uint32_t now_tick;
    float target_abs = (target_angle > 0) ? target_angle : -target_angle;

    // 参数：提前35度刹车，慢速40，快速60
    float stop_offset = 35.0f; 
    int speed_fast = 60;
    int speed_slow = 40;
    int current_speed = speed_fast;

    if(target_angle > 0) { 
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
    } else { 
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
    }

    while(accumulated_angle < (target_abs - stop_offset)) 
    {
        int16_t gyro = MPU_Get_Gyro_Z();
        float speed = gyro - global_gyro_offset; 
        if(speed > -50 && speed < 50) speed = 0;
        
        now_tick = HAL_GetTick();
        dt = (now_tick - last_tick) / 1000.0f;
        last_tick = now_tick;
        if(dt > 0.1f) dt = 0.01f;

        float angle_step = (speed / 131.0f) * dt; 
        if(angle_step < 0) angle_step = -angle_step;
        accumulated_angle += angle_step;

        if ((target_abs - accumulated_angle) < 40.0f) current_speed = speed_slow; 
        speed_left = current_speed; speed_right = current_speed;
        osDelay(5); 
    }
    speed_left = 0; speed_right = 0;
    HAL_Delay(800); 
}

// 8路循迹误差计算
int Get_Tracking_Error(void)
{
    // 假设黑线输出 1 (高电平)
    uint8_t sensor = (uint8_t)(GPIOF->IDR & 0x00FF);
    static int last_known_error = 0;
    int error = 0;
    int sensor_count = 0;
    
    // PF0(最左) 权重-4, PF7(最右) 权重+4
    if (sensor & 0x01) { error -= 4; sensor_count++; } // PF0
    if (sensor & 0x02) { error -= 3; sensor_count++; } // PF1
    if (sensor & 0x04) { error -= 2; sensor_count++; } // PF2
    if (sensor & 0x08) { error -= 1; sensor_count++; } // PF3
    if (sensor & 0x10) { error += 1; sensor_count++; } // PF4
    if (sensor & 0x20) { error += 2; sensor_count++; } // PF5
    if (sensor & 0x40) { error += 3; sensor_count++; } // PF6
    if (sensor & 0x80) { error += 4; sensor_count++; } // PF7
    
    if (sensor_count == 0) return last_known_error; 

    last_known_error = error;
    return error;
}
/* USER CODE END 4 */
