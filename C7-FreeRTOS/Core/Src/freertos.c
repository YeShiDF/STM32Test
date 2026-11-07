/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* DMA 一次最多收这么多字节。队列项是 1 个字节（CubeMX 里配的 uint8_t），
   所以中断里逐字节塞进队列，任务里逐字节取出拼成一行 —— 这样队列配置
   和 .ioc 保持一致，CubeMX 重新生成也不会把大小改坏 */
#define UART_MSG_MAX 64
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO_TIM htim1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart6;

uint8_t readBuf[UART_MSG_MAX];      /* DMA 收数据的落地缓冲区 */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CmdTask */
osThreadId_t CmdTaskHandle;
const osThreadAttr_t CmdTask_attributes = {
  .name = "CmdTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PWMTask */
osThreadId_t PWMTaskHandle;
const osThreadAttr_t PWMTask_attributes = {
  .name = "PWMTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BTNdetectTask */
osThreadId_t BTNdetectTaskHandle;
const osThreadAttr_t BTNdetectTask_attributes = {
  .name = "BTNdetectTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CmdQueue */
osMessageQueueId_t CmdQueueHandle;
const osMessageQueueAttr_t CmdQueue_attributes = {
  .name = "CmdQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCmdTask(void *argument);
void StartUartTask(void *argument);
void StartTaskPWM(void *argument);
void StartTaskBTN(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}

void CmdAnalysis(void);
void CmdSend(void);

/* 多个任务和中断都会读写它，必须加 volatile，否则编译器可能把它缓存进寄存器 */
volatile int ifStartMain = 0;
extern float g_actual_rpm;
extern float g_target_rpm;
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CmdQueue */
  CmdQueueHandle = osMessageQueueNew (32, sizeof(uint8_t), &CmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CmdTask */
  CmdTaskHandle = osThreadNew(StartCmdTask, NULL, &CmdTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(StartUartTask, NULL, &uartTask_attributes);

  /* creation of PWMTask */
  PWMTaskHandle = osThreadNew(StartTaskPWM, NULL, &PWMTask_attributes);

  /* creation of BTNdetectTask */
  BTNdetectTaskHandle = osThreadNew(StartTaskBTN, NULL, &BTNdetectTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  (void)argument;

  /* 1kHz 控制循环：干完活就 osDelayUntil 睡到下一个 tick。
     不要写成 while 空转——那会把 CPU 吃满，后面加的任务全部饿死。 */
  for(;;)
  {
    if (ifStartMain) App_ControlLoop();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCmdTask */
/**
* @brief Function implementing the CmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdTask */
void StartCmdTask(void *argument)
{
  /* USER CODE BEGIN StartCmdTask */
  (void)argument;

  /* 必须在调度器跑起来之后才开接收：回调里要往队列里塞数据，
     调度器没启动就调用 RTOS API 会直接进 HardFault */
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, readBuf, UART_MSG_MAX);

  /* Infinite loop */
  for(;;)
  {
    CmdAnalysis();      /* 没数据时阻塞在队列上，不占 CPU */
  }
  /* USER CODE END StartCmdTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  static const char boot_msg[] = "boot ok\r\n";
  static const char wait_msg[] = "wait cmd: motor+100\r\n";

  (void)argument;

  /* 上电先发一句：看到它说明 COM 口、线、波特率都是对的 */
  HAL_UART_Transmit(&huart6, (uint8_t *)boot_msg, sizeof(boot_msg) - 1u, 50);

  /* Infinite loop */
  for(;;)
  {
    if (ifStartMain) {
      CmdSend();                 /* 已启动：每 2 秒报一次转速 */
    } else {
      /* 还没收到命令：每 2 秒发一次提示。
         如果这里一直有输出，说明 TX 是通的，问题只在 RX；
         如果这里一个字都没有，说明串口线/COM 口根本不对 */
      HAL_UART_Transmit(&huart6, (uint8_t *)wait_msg, sizeof(wait_msg) - 1u, 50);
    }
    osDelay(2000);
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartTaskPWM */
/**
* @brief Function implementing the PWMTask thread.
* @param argument: Not used
* @retval None
*/
extern uint8_t g_all_online;
/* USER CODE END Header_StartTaskPWM */
void StartTaskPWM(void *argument)
{
  /* USER CODE BEGIN StartTaskPWM */
  int pulse = 0;
  int duty;

  (void)argument;

  for(;;)
  {
    if (g_all_online) {
      /* 在线：红灯（PH12 / TIM5_CH3）灭，三角波从头开始，下次掉线才是完整的 2 秒 */
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
      pulse = 0;
      osDelay(50);
    }
    else {
      /* 离线：100 步走完一个三角波，每步 20ms，正好 2 秒一个周期。
         占空比 0 → 100 → 0，是满量程呼吸（TIM5 的 ARR = 100） */
      duty = (pulse < 50) ? (pulse * 2) : ((100 - pulse) * 2);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, duty);
      pulse++;
      if (pulse >= 100) pulse = 0;
      osDelay(20);
    }
  }
  /* USER CODE END StartTaskPWM */
}

/* USER CODE BEGIN Header_StartTaskBTN */
/**
* @brief Function implementing the BTNdetectTask thread.
* @param argument: Not used
* @retval None
*/
typedef enum {
  pressed,
  released,
  triggered,
} btn_state_t;

static volatile btn_state_t btn = released;
/* USER CODE END Header_StartTaskBTN */
void StartTaskBTN(void *argument)
{
  /* USER CODE BEGIN StartTaskBTN */
  int angle = 0;
  int stable;

  (void)argument;

  for(;;)
  {
    if (btn == triggered)
    {
      btn = released;        /* 先复位，避免同一次按下被处理很多遍 */

      /* 连续 5ms 都是低电平才算真的按下，中途弹起就认为是抖动 */
      stable = 1;
      for (int i = 0; i < 5; i++) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_RESET) {
          stable = 0;
          break;
        }
        osDelay(1);
      }

      if (stable) {
        angle += 20;
        if (angle > 180) angle = 0;
        /* 500~2500 对应 0.5~2.5ms；必须写成 angle*2000/180，
           写成 angle/180*2000 的话整数除法先算，永远是 0 */
        __HAL_TIM_SET_COMPARE(&SERVO_TIM, SERVO_TIM_CHANNEL, 500 + angle * 2000 / 180);
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartTaskBTN */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
      btn = triggered;
    }
  }
}

/* DMA 收完一帧（空闲）就会进这里。
   注意：这是中断上下文，队列操作的超时必须是 0，
   写 osWaitForever 的话一旦队列满就会在中断里死等 → 整个系统卡死 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t i;

  if (huart != &huart6) {
    return;
  }
  if (Size > UART_MSG_MAX) {
    Size = UART_MSG_MAX;
  }

  /* 逐字节入队：队列项就是 1 字节，和 CubeMX 里的配置一致 */
  for (i = 0; i < Size; i++) {
    osMessageQueuePut(CmdQueueHandle, &readBuf[i], 0, 0);
  }

  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);   /* 只关心空闲事件，不关心半传输 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, readBuf, UART_MSG_MAX);
}

void CmdAnalysis(void)
{
  static char    line[16];
  static uint8_t n = 0;

  uint8_t ch;
  int v;

  /* 一次取一个字节：没数据时阻塞，不占 CPU */
  if (osMessageQueueGet(CmdQueueHandle, &ch, 0, osWaitForever) != osOK) {
    return;
  }

  /* 换行符 = 一行结束，清空缓冲（回车/换行都不算内容） */
  if (ch == '\r' || ch == '\n') {
    n = 0;
    return;
  }

  if (n < sizeof(line) - 1u) {
    line[n++] = (char)ch;
  } else {
    n = 0;                       /* 太长，丢掉重新收 */
    return;
  }

  /* "motor+100" 固定 9 个字符，收满就解析 */
  if (n != 9u) {
    return;
  }
  n = 0;

  if (memcmp(line, "motor", 5) != 0) return;
  if (line[5] != '+' && line[5] != '-') return;

  v = (line[6] - '0') * 100 + (line[7] - '0') * 10 + (line[8] - '0');
  if (line[5] == '-') {
    v = -v;
  }

  g_target_rpm = (float)v;
  ifStartMain  = 1;
}

void CmdSend(void)
{
  char buf[16];
  int  n = snprintf(buf, sizeof(buf), "rpm=%d\r\n", (int)g_actual_rpm);

  if (n > 0) {
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
    HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)n, 50);
  }
}

/* USER CODE END Application */

