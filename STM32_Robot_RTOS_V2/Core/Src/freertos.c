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
volatile float Kp_Track = 1.0f;  // P: 转向力度
volatile float Kd_Track = 0.0f;  // D: 抑制震荡
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
	osThreadSuspend(NULL);
  for(;;)
  {
    osDelay(1000);
  }
}

/* USER CODE BEGIN Header_StartLogicTask */
/* USER CODE BEGIN StartLogicTask */

void StartLogicTask(void const * argument)
{
  // ==========================================
  // 1. 变量定义区
  // ==========================================
  
  // MPU6050 相关
  int16_t gyro_z_raw = 0;
  float angle_z = 0;
  char msg[64];
  int i; 

  // 避障模式变量
  volatile float dis_front = 0;
  volatile float dis_left = 0;
  volatile float dis_right = 0;
  float safe_distance = 15.0f;    
  float target_heading = 0; 
  float Kp_Obstacle = 2.0f; // 避障走直线用的纠偏力度

  // 循迹模式变量
  int tracking_error = 0;
  int last_tracking_error = 0;
  // 注意：Kp_Track, Kd_Track, base_speed 是全局变量，在文件顶部定义
  // 这样你可以通过蓝牙命令 (Pxx#, Vxx#) 实时修改它们

  // 时间积分变量
  uint32_t last_tick = 0;
  uint32_t now_tick = 0;
  float dt = 0;
  float scale_factor = 1.0f; 

  // ==========================================
  // 2. 初始化流程
  // ==========================================
  MPU_Init();
  osDelay(500); 

  sprintf(msg, "System Ready! Calibrating...\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

  // 陀螺仪零偏校准 (静止 0.5s)
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

  // ==========================================
  // 3. 任务主循环
  // ==========================================
  for(;;)
  {
    // ---------------------------
    // A. 陀螺仪后台持续积分
    // ---------------------------
    gyro_z_raw = MPU_Get_Gyro_Z();
    float gyro_z_corrected = gyro_z_raw - global_gyro_offset;
    // 简单死区，消除静止漂移
    if(gyro_z_corrected > -50 && gyro_z_corrected < 50) gyro_z_corrected = 0;

    now_tick = HAL_GetTick();
    dt = (now_tick - last_tick) / 1000.0f;
    last_tick = now_tick;
    // 防止 dt 异常过大
    if(dt > 0.1f) dt = 0.01f;

    angle_z += (gyro_z_corrected / 131.0f) * dt * scale_factor; 

    // ---------------------------
    // B. 模式状态机
    // ---------------------------
    if (car_mode == 0)
    {
        // === 模式 0: 遥控待机 ===
        // 什么都不做，听 StartComTask 指挥
    }
    else if (car_mode == 1)
    {
        // ===========================
        //      模式 1: 自动避障
        // ===========================
        dis_front = Get_Distance();

        if (dis_front > 25.0f)
        {
            // --- 直行 + 陀螺仪纠偏 ---
            float error = angle_z - target_heading;
            int turn_adjust = (int)(error * Kp_Obstacle);

            int left_motor = 50 - turn_adjust;
            int right_motor = 50 + turn_adjust;

            // 限幅
            if(left_motor > 90) left_motor = 90; if(left_motor < 0) left_motor = 0;
            if(right_motor > 90) right_motor = 90; if(right_motor < 0) right_motor = 0;

            speed_left = left_motor; speed_right = right_motor;
            
			// 🔥🔥 方向引脚交换 🔥🔥
            // 逻辑左轮(L) -> 控制 BIN (G9/D6)
            // 逻辑右轮(R) -> 控制 AIN (D0/C11)
            
            // 左轮前进 (控制 BIN)
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);    
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
            
            // 右轮前进 (控制 AIN)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);    
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); 
        }
        else
        {
            // --- 遇到障碍 ---
            speed_left = 0; speed_right = 0;
            osDelay(500);

            // 摇头侦察
            Servo_Turn(160); osDelay(600); dis_left = Get_Distance();
            Servo_Turn(20);  osDelay(600); dis_right = Get_Distance();
            Servo_Turn(90);  osDelay(300);

           if (dis_left < safe_distance && dis_right < safe_distance)
            {
                // 死胡同：倒车 (方向全反)
                // 左轮后退(BIN)
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET); 
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
                // 右轮后退(AIN)
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); 
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
                
                speed_left = 50; speed_right = 50; HAL_Delay(500);
                speed_left = 0; speed_right = 0; HAL_Delay(200);

                Auto_Turn(-90.0f); 
                angle_z -= 90.0f; target_heading -= 90.0f; last_tick = HAL_GetTick();
            }
            else if (dis_left > dis_right)
            {
                Auto_Turn(90.0f); 
                angle_z += 90.0f; target_heading += 90.0f; last_tick = HAL_GetTick();
            }
            else
            {
                Auto_Turn(-90.0f); 
                angle_z -= 90.0f; target_heading -= 90.0f; last_tick = HAL_GetTick();
            }
        }
    }
    else if (car_mode == 2)
    {
        // ===========================
        //      模式 2: PID 循迹 (暴力死区补偿版)
        // ===========================
        
        // 1. 获取误差
        tracking_error = Get_Tracking_Error();
        
        // 2. PID 计算
        int pid_out = (int)(Kp_Track * tracking_error + Kd_Track * (tracking_error - last_tracking_error));
        last_tracking_error = tracking_error;

        // 3. 计算初步速度
        int motor_l = base_speed - pid_out; 
        int motor_r = base_speed + pid_out;
        
        // 🔥🔥🔥【死区穿越逻辑】🔥🔥🔥
        // 你的电机死区大概是 25。如果算出来的速度在 -25 到 25 之间，
        // 电机不仅不动，还浪费了转向机会。
        // 我们强制把它改成 -35，让它猛烈反转！
        
        int dead_zone = 25;     // 死区阈值 (根据你的电机情况调整)
        int violent_kick = -35; // 强制反转速度 (越负越暴力)

        // 左轮检查
        if (motor_l < dead_zone && motor_l > -dead_zone) 
        {
            motor_l = violent_kick; 
        }

        // 右轮检查
        if (motor_r < dead_zone && motor_r > -dead_zone) 
        {
            motor_r = violent_kick; 
        }

        // 4. 安全限幅
        if(motor_l > 90) motor_l = 90; if(motor_l < -90) motor_l = -90;
        if(motor_r > 90) motor_r = 90; if(motor_r < -90) motor_r = -90;
        
        // 5. 写入电机 (支持正反转)
        
       // 🔥🔥 方向引脚交换 🔥🔥
        // --- 左轮 (控制 BIN) ---
        if(motor_l >= 0) {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);   // 正转
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
            speed_left = motor_l;
        } else {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET); // 反转
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
            speed_left = -motor_l;
        }

        // --- 右轮 (控制 AIN) ---
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
    else if (car_mode == 3)
    {
        // ===========================
        //      模式 3: 传感器调试
        // ===========================
        // 把 8路数据发给手机看 (二进制字符串)
        char binary_str[10];
        uint8_t sensor = (uint8_t)(GPIOF->IDR & 0x00FF);
        
        for(int j=0; j<8; j++)
        {
            if (sensor & (1 << (7-j))) binary_str[j] = '1';
            else binary_str[j] = '0';
        }
        binary_str[8] = '\n'; 
        binary_str[9] = '\0';
        
        HAL_UART_Transmit(&huart3, (uint8_t*)binary_str, 9, 100);
        osDelay(500); 
    }
	else if (car_mode == 4)
    {
        // ===========================
        //      模式 4: 砰砰控制 (无 PID 对照组)
        // ===========================
        // 原理：检测到那边有黑线，就死命往那边转，没有微调
        
        tracking_error = Get_Tracking_Error();
        
        // 笨蛋模式的速度通常要慢一点，否则直接飞出去
        // 你可以通过对比：PID 跑 40 不飞，这个跑 25 就飞了 -> 证明 PID 强
        int dumb_speed = 25; 
        
        // 1. 设置方向 (沿用你的软件换线逻辑：左控BIN，右控AIN)
        // 两个轮子都默认正转
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

        // 2. 简单粗暴的逻辑
        if (tracking_error > 5) 
        {
            // --- 偏右太厉害，向左死转 ---
            // 左轮停 (甚至倒转)，右轮转
            speed_left = 0;   // 或者 -20
            speed_right = dumb_speed + 10; 
        }
        else if (tracking_error < -5) 
        {
            // --- 偏左太厉害，向右死转 ---
            // 左轮转，右轮停
            speed_left = dumb_speed + 10;
            speed_right = 0;  // 或者 -20
        }
        else
        {
            // --- 差不多在中间，直走 ---
            speed_left = dumb_speed;
            speed_right = dumb_speed;
        }
        
        // 3. 安全限幅 (防止 PWM 溢出)
        if(speed_left > 90) speed_left = 90; if(speed_left < -90) speed_left = -90;
        if(speed_right > 90) speed_right = 90; if(speed_right < -90) speed_right = -90;
    }

    osDelay(10); // 调度间隔
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
			case 'N': car_mode = 4; break;
            case '?': // 查询参数
                sprintf(echo_msg, "P=%.1f, D=%.1f, V=%d\r\n", Kp_Track, Kd_Track, base_speed);
                HAL_UART_Transmit(&huart3, (uint8_t*)echo_msg, strlen(echo_msg), 100);
                break;

            case 'S': if(!car_mode) { speed_left = 0; speed_right = 0; } break;
            case 'G': if(!car_mode) { 
                    speed_left = 50; speed_right = 50; 
                    // 左轮(BIN)正
                    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
                    // 右轮(AIN)正
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
                } break;
            case 'B': if(!car_mode) { 
                    speed_left = 50; speed_right = 50; 
                    // 左轮(BIN)反
                    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
                    // 右轮(AIN)反
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
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
    // ============================================
    // 1. 变量初始化
    // ============================================
    float accumulated_angle = 0;
    float dt = 0;
    uint32_t last_tick = HAL_GetTick();
    uint32_t now_tick;
    float target_abs = (target_angle > 0) ? target_angle : -target_angle;

    // 🔥 安全保险丝：记录开始时间
    // 如果转弯超过 2秒 (2000ms)，说明出问题了，强制退出
    uint32_t start_time = HAL_GetTick(); 

    // ============================================
    // 2. 刹车策略配置
    // ============================================
    // 参数：提前 35度 刹车利用惯性 (根据地面摩擦力调整，地滑就改大，地涩就改小)
    float stop_offset = 35.0f; 
    
    // 🔥 小角度补丁：如果只需要转 20度，就不要提前刹车了，否则一步都不走
    if (target_abs < stop_offset) stop_offset = 0;

    int speed_fast = 60;
    int speed_slow = 40;
    int current_speed = speed_fast;

    // ============================================
    // 3. 设置电机方向 (原地坦克掉头)
    // ============================================
   if(target_angle > 0) { 
        // 左转：左轮(BIN)退、右轮(AIN)进
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    } else { 
        // 右转：左轮(BIN)进、右轮(AIN)退
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    }

    // ============================================
    // 4. 循环积分控制
    // ============================================
    while(accumulated_angle < (target_abs - stop_offset)) 
    {
        // 🔥🔥【关键修改】超时保护 🔥🔥
        // 如果线松了或者卡住了，超过 2000ms 还没转完，强制跳出！
        if ((HAL_GetTick() - start_time) > 2000) break;

        // --- 陀螺仪积分逻辑 ---
        int16_t gyro = MPU_Get_Gyro_Z();
        float speed = gyro - global_gyro_offset; 
        
        // 动态死区过滤
        if(speed > -50 && speed < 50) speed = 0;
        
        now_tick = HAL_GetTick();
        dt = (now_tick - last_tick) / 1000.0f;
        last_tick = now_tick;
        
        // 防止 dt 异常
        if(dt > 0.1f) dt = 0.01f;

        // 计算这一瞬间转过的角度 (取绝对值累加)
        float angle_step = (speed / 131.0f) * dt; 
        if(angle_step < 0) angle_step = -angle_step;
        accumulated_angle += angle_step;

        // --- 速度规划 ---
        // 离目标还有 40度 时，减速慢行，防止冲过头
        if ((target_abs - accumulated_angle) < 40.0f) current_speed = speed_slow; 
        
        speed_left = current_speed; 
        speed_right = current_speed;
        
        // --- RTOS 调度 ---
        osDelay(5); // 让出 CPU 给蓝牙任务
    }

    // ============================================
    // 5. 刹车与缓冲
    // ============================================
    speed_left = 0; speed_right = 0;
    
    // 使用 osDelay 而不是 HAL_Delay，保证刹车期间蓝牙不掉线
    osDelay(800); 
}

// 8路循迹误差计算
int Get_Tracking_Error(void)
{
    uint8_t sensor = (uint8_t)(GPIOF->IDR & 0x00FF);
    static int last_known_error = 0;
    
    int error_sum = 0;   // 累加和
    int sensor_count = 0; // 触发个数
    
    // 1. 权重扩大 10 倍，保留小数点后一位的精度
    // PF0(-40), PF1(-30) ... PF7(+40)
    if (sensor & 0x01) { error_sum -= 40; sensor_count++; } 
    if (sensor & 0x02) { error_sum -= 30; sensor_count++; } 
    if (sensor & 0x04) { error_sum -= 20; sensor_count++; } 
    if (sensor & 0x08) { error_sum -= 10; sensor_count++; } 
    if (sensor & 0x10) { error_sum += 10; sensor_count++; } 
    if (sensor & 0x20) { error_sum += 20; sensor_count++; } 
    if (sensor & 0x40) { error_sum += 30; sensor_count++; } 
    if (sensor & 0x80) { error_sum += 40; sensor_count++; } 
    
    // 2. 丢失目标处理
    if (sensor_count == 0) return last_known_error; 

    // 3. 【关键修改】求平均值 (重心法)
    int final_error = error_sum / sensor_count;

    // 4. 这里的 error 范围变成了 -40 到 +40
    // 所以你的 PID 参数 Kp 需要相应地除以 10 (或者调小一点)，否则车会晃得很厉害
    last_known_error = final_error;
    return final_error;
}
/* USER CODE END 4 */
