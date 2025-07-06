/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.3.1
 * Portion Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Portion Copyright (C) 2019 StMicroelectronics, Inc.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
#define configUSE_TASK_FPU_SUPPORT 1       // 启用单精度浮点数支持
#define configUSE_QUEUE_SETS    1   // 启用队列集功能
// #define configUSE_TIME_SLICING  1   // 启用时间片轮转调度，默认不启用，使用vTaskDelay(1)强制每一个tick切换任务，
// 即使启用也只能设置1个tick的时间片，并不能如RT-Thread那样设置任意时间片长度，比如10个TICK
/**
当FreeRTOSConfig.h中未定义configUSE_TIME_SLICING 时，时间片轮转调度机制不会启用，此时同优先级任务的调度行为与时间片无关。以下是具体分析：
1. configUSE_TIME_SLICING的作用
        启用时：同优先级任务按时间片（1 个 tick）强制轮转，无论任务是否执行完毕，时间片用完即切换。
未启用时：同优先级任务不会被强制切换，仅在以下情况才会让出 CPU：
任务主动调用vTaskDelay()、xQueueReceive()等阻塞 API；
任务执行完毕（退出）；
任务通过taskYIELD()主动让步。
2. 任务运行时间的决定因素
        当configUSE_TIME_SLICING = 0时：

任务运行时间不固定为 1ms，而是由任务自身逻辑决定：
若任务执行快（如 0.5ms 完成），则立即释放 CPU；
若任务无阻塞操作且不主动让步，则会一直占用 CPU（导致同优先级其他任务饥饿）。
*/
/**10个tick的时间片，执行一次代码0.5个tick，然后延时挂起1tick时间，那么剩余时间片会变为9.5tick吗？还是说依旧是10个tick？此时挂起时间结束了，任务又变为就绪状态，时间片会被重置为10tick吗？
在 RT-Thread 中，当任务调用rt_thread_delay()主动挂起时，时间片机制的行为如下：
一、核心结论
时间片不会保留剩余值，挂起结束后会重置为初始值。具体规则：

调用延时函数时：
任务立即让出 CPU，当前时间片剩余值被丢弃（无论剩余多少）。
延时结束后：
任务重新进入就绪队列时，时间片重置为初始值（如 10 tick）。
二、详细执行过程示例
假设任务配置：

时间片 = 10 tick
单次循环执行时间 = 0.5 tick
延时函数 = rt_thread_delay(1)
1. 第一轮调度
plaintext
时间点   事件                              时间片状态
0-0.5   任务执行0.5 tick操作              剩余9.5 tick
0.5     调用rt_thread_delay(1)            时间片剩余值被丢弃
0.5-1.5 任务处于挂起状态，CPU让给其他任务  时间片机制暂停
1.5     延时结束，任务进入就绪队列          时间片重置为10 tick

2. 第二轮调度
plaintext
时间点   事件                              时间片状态
1.5-2.0 任务被调度，执行0.5 tick操作       剩余9.5 tick
2.0     再次调用rt_thread_delay(1)         时间片剩余值被丢弃
2.0-3.0 任务挂起                          时间片机制暂停
3.0     延时结束，任务就绪                 时间片重置为10 tick
 */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif
#define configENABLE_FPU                         1
#define configENABLE_MPU                         0

#define configUSE_PREEMPTION                     1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     ( 7 )
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                    ((size_t)200000)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_TRACE_FACILITY                 1
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
/* USER CODE BEGIN MESSAGE_BUFFER_LENGTH_TYPE */
/* Defaults to size_t for backward compatibility, but can be changed
   if lengths will always be less than the number of bytes in a size_t. */
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
/* USER CODE END MESSAGE_BUFFER_LENGTH_TYPE */

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 2 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             256

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet             1
#define INCLUDE_uxTaskPriorityGet            1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskCleanUpResources        0
#define INCLUDE_vTaskSuspend                 1
#define INCLUDE_vTaskDelayUntil              1
#define INCLUDE_vTaskDelay                   1
#define INCLUDE_xTaskGetSchedulerState       1
#define INCLUDE_xTimerPendFunctionCall       1
#define INCLUDE_xQueueGetMutexHolder         1
#define INCLUDE_uxTaskGetStackHighWaterMark  1
#define INCLUDE_xTaskGetCurrentTaskHandle    1
#define INCLUDE_eTaskGetState                1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

/* 可用于"设置优先级"函数的最低中断优先级（数值越大优先级越低） */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* 任何调用FreeRTOS中断安全API函数的中断服务程序可使用的最高优先级。
  注意：不要在优先级高于此值的中断中调用FreeRTOS API函数！
  （Cortex-M中数值越小表示优先级越高） */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* 内核端口层自身使用的中断优先级。这些对所有Cortex-M端口都是通用的，
  不依赖于任何特定的库函数。 */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY 绝不能设为0 !!!!
  详见：http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */

#define xPortSysTickHandler SysTick_Handler

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
