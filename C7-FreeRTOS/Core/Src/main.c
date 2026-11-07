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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../BSP/can_motor/can_motor.h"
#include "../BSP/pid/pid.h"
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
/* 用 g_ 前缀放全局变量：CubeMonitor 里直接搜名字就能抓到 */
PID_t  g_speed_pid;                 /* 速度环 PID 实例 */
float  g_target_rpm = 100.0f;       /* 目标转速，改这个就能改速度 */
float  g_actual_rpm = 0.0f;         /* 实际转速（来自反馈帧） */
float  g_out        = 0.0f;         /* PID 输出（控制量） */
float  g_loop_hz    = 0.0f;         /* 实测控制循环频率，正常应该是 1000 左右 */
uint8_t g_all_online = 0;           /* 所有电机是否在线（供其他任务读） */

/* 启动开关，定义在 BSP/tasks.c：收到 "motor+NNN" 后置 1。
   控制循环要先看它，所以这里要有声明。 */
extern volatile int ifStartMain;

/* 两颗 LED 都挂在 TIM5 上做 PWM（ARR = 100，所以占空比就是 0~100）：
 *   绿灯 PH11 → TIM5_CH2   红灯 PH12 → TIM5_CH3
 * 用 PWM 的好处：亮度可调、能做呼吸，而且这两个脚本来就是复用模式，
 * 用 HAL_GPIO_WritePin 写 ODR 是无效的。 */
#define LED_G_PWM_TIM       htim5
#define LED_G_PWM_CHANNEL   TIM_CHANNEL_2     /* PH11 */
#define LED_R_PWM_TIM       htim5
#define LED_R_PWM_CHANNEL   TIM_CHANNEL_3     /* PH12 */

#define LED_G_ON_DUTY       100               /* 绿灯亮度 0~100（TIM5 的 ARR = 100），刺眼就调小 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ------------------------------------------------------------------
 * APP_MODE_OPEN_LOOP = 1  开环：不看反馈，给固定控制量（用来单独验证电机链路）
 * APP_MODE_OPEN_LOOP = 0  闭环：速度环 PID，稳在 g_target_rpm
 * 调参前建议先用开环跑一次，确认"给 9400 能转到 100rpm 左右"再切回来。
 * ------------------------------------------------------------------ */
#define APP_MODE_OPEN_LOOP   0
#define OPEN_LOOP_OUT        9400     /* 开环控制量，空载约 7.5V / 100rpm */

static void App_OfflineIndicate(uint8_t all_online)
{
  /* 绿灯（PH11 / TIM5_CH2）：在线常亮，离线熄灭。
     红灯的呼吸由 PWMTask 负责，这里不碰。 */
  __HAL_TIM_SET_COMPARE(&LED_G_PWM_TIM, LED_G_PWM_CHANNEL,
                        all_online ? LED_G_ON_DUTY : 0);
}

/* 1kHz 控制循环：采样 → 算 PID → 发指令 → 刷新在线状态
 * 注意：节拍由调用者控制（defaultTask 用 osDelayUntil 每 1ms 调一次），
 *       这里不再自己判 HAL_GetTick，否则两层计时会互相打架。 */
void App_ControlLoop(void)
{
  static uint32_t cnt = 0, win_start = 0;
  uint32_t now = HAL_GetTick();
  const Motor_Data_t *d;

  /* ---- 诊断：统计真实的控制频率（CubeMonitor 里看 g_loop_hz） ---- */
  cnt++;
  if ((uint32_t)(now - win_start) >= 1000u) {
    g_loop_hz  = (float)cnt * 1000.0f / (float)(uint32_t)(now - win_start);
    cnt        = 0;
    win_start  = now;
  }

  /* ① 在线判定 + 指示灯：每拍都做，不受 ifStartMain 影响。
        Motor_Update() 只是拿 last_tick 和 now 比一下，不发帧、不碰电机，
        放到下面那层 if 之后的话，没收到命令时就永远判离线，红灯一直亮。 */
  g_all_online = Motor_Update();      /* 软件看门狗：刷新在线状态 */
  App_OfflineIndicate(g_all_online);

  d = Motor_Get(1);
  if (d == 0) {
    return;
  }
  g_actual_rpm = (float)d->speed;

  /* ② 还没收到 "motor+NNN"：只监视、不输出，电机保持静止 */
  if (!ifStartMain) {
    return;
  }

  /* ③ 收到命令后才开始闭环输出 */
#if APP_MODE_OPEN_LOOP
  g_out = (float)OPEN_LOOP_OUT;
#else
  if (!Motor_IsOnline(1)) {
    PID_Reset(&g_speed_pid);      /* 掉线：清账本，避免恢复瞬间冲击 */
    g_out = 0.0f;
  } else {
    /* 一行搞定：喂目标转速和实际转速，返回本拍控制量 */
    g_out = PID_Calc(&g_speed_pid, g_target_rpm, g_actual_rpm);
  }
#endif

  Motor_SetOutput(1, (int16_t)g_out);
  Motor_Flush();
}

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
  MX_CAN1_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ① 初始化 CAN1：配过滤器 + 启动外设 + 打开 RX0 接收中断 */
  Motor_Init(&hcan1);

  /* ② 注册电机：这里的 ID 必须和电机实际 ID 一致（6020 用拨码/上位机设置，默认 1） */
  if (!Motor_Attach(1, MOTOR_TYPE_GM6020))
  {
    Error_Handler();          /* ID 非法或槽位冲突，停在这里比"电机不动"好查 */
  }

  /* ③ 初始化速度环 PID：先给一组能转的起点，具体参数自己调 */
  PID_Init(&g_speed_pid, 40.0f, 500.0f, 0.0f, speed);

  /* 如果电机接在 CAN2 上，把上面两句换成：
       Motor_Init(&hcan2);
       Motor_Attach(1, MOTOR_TYPE_GM6020);
     并确认 CubeMX 里 CAN2 的 RX0 中断也打开了 */


  /* ④ 启动 PWM 通道：不启动的话 SET_COMPARE 只是改了个数，引脚上没有任何波形 */
  __HAL_TIM_SET_COMPARE(&LED_G_PWM_TIM, LED_G_PWM_CHANNEL, 0);   /* 绿灯先灭 */
  __HAL_TIM_SET_COMPARE(&LED_R_PWM_TIM, LED_R_PWM_CHANNEL, 0);   /* 红灯先灭 */
  HAL_TIM_PWM_Start(&LED_G_PWM_TIM, LED_G_PWM_CHANNEL);          /* 绿灯，PH11 / TIM5_CH2 */
  HAL_TIM_PWM_Start(&LED_R_PWM_TIM, LED_R_PWM_CHANNEL);          /* 红灯，PH12 / TIM5_CH3 */

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);  /* 舵机上电回 0°（0.5ms） */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       /* 舵机，PE9 / TIM1_CH1 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
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
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
