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
osThreadId ChassisTaskHandle;
osThreadId CmdTaskHandle;
osThreadId DMmotorTaskHandle;
osThreadId RefereeTaskHandle;
osThreadId TranmissionTaskHandle;
osThreadId UsartTaskHandle;
osThreadId InsTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AlgorithmTask_Entry(void const * argument);
void ChassisTask_Entry(void const * argument);
void CmdTask_Entry(void const * argument);
void DMmotorTask_Entry(void const * argument);
void RefereeTask_Entry(void const * argument);
void TransmissionTask_Entry(void const * argument);
void UsartTask_Entry(void const * argument);
void InsTask_Entry(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  osThreadDef(AlgorithmTask, AlgorithmTask_Entry, osPriorityHigh, 0, 2048);
  AlgorithmTaskHandle = osThreadCreate(osThread(AlgorithmTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, ChassisTask_Entry, osPriorityHigh, 0, 2048);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of CmdTask */
  osThreadDef(CmdTask, CmdTask_Entry, osPriorityHigh, 0, 2048);
  CmdTaskHandle = osThreadCreate(osThread(CmdTask), NULL);

  /* definition and creation of DMmotorTask */
  osThreadDef(DMmotorTask, DMmotorTask_Entry, osPriorityHigh, 0, 2048);
  DMmotorTaskHandle = osThreadCreate(osThread(DMmotorTask), NULL);

  /* definition and creation of RefereeTask */
  osThreadDef(RefereeTask, RefereeTask_Entry, osPriorityHigh, 0, 2048);
  RefereeTaskHandle = osThreadCreate(osThread(RefereeTask), NULL);

  /* definition and creation of TranmissionTask */
  osThreadDef(TranmissionTask, TransmissionTask_Entry, osPriorityHigh, 0, 2048);
  TranmissionTaskHandle = osThreadCreate(osThread(TranmissionTask), NULL);

  /* definition and creation of UsartTask */
  osThreadDef(UsartTask, UsartTask_Entry, osPriorityHigh, 0, 2048);
  UsartTaskHandle = osThreadCreate(osThread(UsartTask), NULL);

  /* definition and creation of InsTask */
  osThreadDef(InsTask, InsTask_Entry, osPriorityRealtime, 0, 2048);
  InsTaskHandle = osThreadCreate(osThread(InsTask), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN AlgorithmTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AlgorithmTask_Entry */
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

/* USER CODE BEGIN Header_DMmotorTask_Entry */
/**
* @brief Function implementing the DMmotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DMmotorTask_Entry */
__weak void DMmotorTask_Entry(void const * argument)
{
  /* USER CODE BEGIN DMmotorTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DMmotorTask_Entry */
}

/* USER CODE BEGIN Header_RefereeTask_Entry */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RefereeTask_Entry */
__weak void RefereeTask_Entry(void const * argument)
{
  /* USER CODE BEGIN RefereeTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RefereeTask_Entry */
}

/* USER CODE BEGIN Header_TransmissionTask_Entry */
/**
* @brief Function implementing the TranmissionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransmissionTask_Entry */
__weak void TransmissionTask_Entry(void const * argument)
{
  /* USER CODE BEGIN TransmissionTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TransmissionTask_Entry */
}

/* USER CODE BEGIN Header_UsartTask_Entry */
/**
* @brief Function implementing the UsartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UsartTask_Entry */
__weak void UsartTask_Entry(void const * argument)
{
  /* USER CODE BEGIN UsartTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UsartTask_Entry */
}

/* USER CODE BEGIN Header_InsTask_Entry */
/**
* @brief Function implementing the InsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InsTask_Entry */
__weak void InsTask_Entry(void const * argument)
{
  /* USER CODE BEGIN InsTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END InsTask_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
