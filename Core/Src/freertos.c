/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Variables */
osThreadId AlgorithmTaskHandle;
osThreadId USART1_RecTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId CmdTaskHandle;
osThreadId DMmotorTaskHandle;
osThreadId RefereeTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AlgorithmTask_Entry(void const * argument);
void USART1_RecEntry(void const * argument);
void ChassisTask_Entry(void const * argument);
void CmdTask_Entry(void const * argument);
void DMmotor_Entry(void const * argument);
void Referee_Entry(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of AlgorithmTask */
  osThreadDef(AlgorithmTask, AlgorithmTask_Entry, osPriorityHigh, 0, 256);
  AlgorithmTaskHandle = osThreadCreate(osThread(AlgorithmTask), NULL);

  /* definition and creation of USART1_RecTask */
  osThreadDef(USART1_RecTask, USART1_RecEntry, osPriorityHigh, 0, 512);
  USART1_RecTaskHandle = osThreadCreate(osThread(USART1_RecTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, ChassisTask_Entry, osPriorityHigh, 0, 256);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of CmdTask */
  osThreadDef(CmdTask, CmdTask_Entry, osPriorityHigh, 0, 512);
  CmdTaskHandle = osThreadCreate(osThread(CmdTask), NULL);

  /* definition and creation of DMmotorTask */
  osThreadDef(DMmotorTask, DMmotor_Entry, osPriorityHigh, 0, 512);
  DMmotorTaskHandle = osThreadCreate(osThread(DMmotorTask), NULL);

  /* definition and creation of RefereeTask */
  osThreadDef(RefereeTask, Referee_Entry, osPriorityHigh, 0, 128);
  RefereeTaskHandle = osThreadCreate(osThread(RefereeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_AlgorithmTask_Entry */
/**
  * @brief  Function implementing the AlgorithmTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AlgorithmTask_Entry */
__weak void AlgorithmTask_Entry(void const * argument)
{
  /* USER CODE BEGIN AlgorithmTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AlgorithmTask_Entry */
}

/* USER CODE BEGIN Header_USART1_RecEntry */
/**
* @brief Function implementing the USART1_RecTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USART1_RecEntry */
__weak void USART1_RecEntry(void const * argument)
{
  /* USER CODE BEGIN USART1_RecEntry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END USART1_RecEntry */
}

/* USER CODE BEGIN Header_ChassisTask_Entry */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask_Entry */
__weak void ChassisTask_Entry(void const * argument)
{
  /* USER CODE BEGIN ChassisTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisTask_Entry */
}

/* USER CODE BEGIN Header_CmdTask_Entry */
/**
* @brief Function implementing the CmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CmdTask_Entry */
__weak void CmdTask_Entry(void const * argument)
{
  /* USER CODE BEGIN CmdTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CmdTask_Entry */
}

/* USER CODE BEGIN Header_DMmotor_Entry */
/**
* @brief Function implementing the DMmotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DMmotor_Entry */
__weak void DMmotor_Entry(void const * argument)
{
  /* USER CODE BEGIN DMmotor_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DMmotor_Entry */
}

/* USER CODE BEGIN Header_Referee_Entry */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Entry */
__weak void Referee_Entry(void const * argument)
{
  /* USER CODE BEGIN Referee_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
