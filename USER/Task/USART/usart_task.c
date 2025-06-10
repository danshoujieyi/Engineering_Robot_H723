/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */
#include "usart_task.h"
#include "FreeRTOS.h"
#include "rc_sbus.h"
#include "referee_system.h"
#include "usart.h"
#include <string.h>
#include "drv_dwt.h"

#include <stdio.h>
#include <stdarg.h>

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
//static struct chassis_cmd_msg chassis_cmd;
//static struct chassis_fdb_msg chassis_fdb;
//static struct trans_fdb_msg trans_fdb;
//static struct ins_msg ins_data;
//
//static publisher_t *pub_chassis;
//static subscriber_t *sub_cmd,*sub_ins,*sub_trans;
//
//static void chassis_pub_init(void);
//static void chassis_sub_init(void);
//static void chassis_pub_push(void);
//static void chassis_sub_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t usart_task_dwt = 0;   // 毫秒监测
static float usart_task_dt = 0;       // 线程实际运行时间dt
static float usart_task_delta = 0;    // 监测线程运行时间
static float usart_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

// 自定义控制器串口
static volatile uint8_t usart1_rx_buffer_index = 0;  // 当前使用的接收缓冲区
static volatile uint16_t usart1_rx_size = 0;
static uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];
static SemaphoreHandle_t xSemaphoreUART1_RX = NULL;           // 通知任务处理信号量

// 福斯遥控器
static volatile uint8_t usart5_rx_buffer_index = 0;  // 当前使用的接收缓冲区
static volatile uint16_t usart5_rx_size = 0;
static uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];
static SemaphoreHandle_t xSemaphoreUART5_RX = NULL;

// 裁判系统串口 10
static volatile uint8_t referee_rx_buffer_index = 0;  // 当前使用的接收缓冲区
static volatile uint16_t referee_rx_size = 0;
static uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
static SemaphoreHandle_t xSemaphoreUART10_RX = NULL;

static QueueSetHandle_t xUartQueueSet = NULL; // 定义接收队列集句柄,统一管理串口中断信号量

// usart7双缓冲发送
#define USART7_TX_DEBUG_BUFFER_SIZE 512 // printf发送缓冲区大小
static char usart7_tx_debug_buffer[2][USART7_TX_DEBUG_BUFFER_SIZE];
static volatile uint8_t usart7_tx_buffer_index = 0;  // 当前使用的缓冲区
static SemaphoreHandle_t xSemaphoreUART7_TX = NULL; // 串口1发送信号量

extern struct referee_fdb_msg referee_fdb;

void usart_rx_semaphore_init(void)
{
    // FreeRTOS 初始化
    xSemaphoreUART10_RX = xSemaphoreCreateBinary();  // <-- 在此处创建信号量
    xSemaphoreUART1_RX = xSemaphoreCreateBinary();  // <-- 在此处创建信号量
    xSemaphoreUART5_RX = xSemaphoreCreateBinary();  // <-- 在此处创建信号量
    // 定义队列集（最多监听 3 个信号量）
    xUartQueueSet = xQueueCreateSet(3);
    // 将各串口信号量加入队列集
    xQueueAddToSet(xSemaphoreUART1_RX, xUartQueueSet);
    xQueueAddToSet(xSemaphoreUART5_RX, xUartQueueSet);
    xQueueAddToSet(xSemaphoreUART10_RX, xUartQueueSet);
}

void usart_tx_semaphore_init(void)
{
    // 创建二值信号量（用于DMA传输同步）
    xSemaphoreUART7_TX = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphoreUART7_TX);
}

void USART1_RX_DMA_Init(void) {
    memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void USART5_RX_DMA_Init(void) {
    memset(usart5_rx_buffer, 0, SBUS_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收完毕后重启
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void USART10_RX_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

void process_usart1_rx_data(void) {
    uint8_t finishedBuffer;

    // 非阻塞获取信号量（因为队列集已确保信号量有效）
    if (xSemaphoreTake(xSemaphoreUART1_RX, 0) == pdTRUE) {
        finishedBuffer = usart1_rx_buffer_index ^ 1;
        /* 数据解析 */
        referee_data_unpack(usart1_rx_buffer[finishedBuffer], usart1_rx_size);

        memset(usart1_rx_buffer[finishedBuffer], 0, CUSTOMER_CONTROLLER_BUF_SIZE);

    }
}

void process_uart5_rx_data(void) {
    uint8_t finishedBuffer;

    if (xSemaphoreTake(xSemaphoreUART5_RX, 0) == pdTRUE) {
        finishedBuffer = usart5_rx_buffer_index ^ 1;
        /* SBUS协议解析 */
        sbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);

        memset(usart5_rx_buffer[finishedBuffer], 0, SBUS_RX_BUF_SIZE);
    }
}

void process_uart10_rx_data(void) {
    uint8_t finishedBuffer;

    if (xSemaphoreTake(xSemaphoreUART10_RX, 0) == pdTRUE) {
        finishedBuffer = referee_rx_buffer_index ^ 1;
        /* 裁判系统数据解析 */
        referee_data_unpack(referee_rx_buffer[finishedBuffer], referee_rx_size);

        memset(referee_rx_buffer[finishedBuffer], 0, REFEREE_RX_BUF_SIZE);
    }
}


/* -------------------------------- 线程入口 ------------------------------- */
void UsartTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    QueueSetMemberHandle_t xActivatedMember;

    USART1_RX_DMA_Init();
    USART5_RX_DMA_Init();
    USART10_RX_DMA_Init();
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    usart_task_dt = dwt_get_delta(&usart_task_dwt);
    usart_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        usart_task_delta = dwt_get_time_ms() - usart_task_start_dt;
        usart_task_start_dt = dwt_get_time_ms();
        usart_task_dt = dwt_get_delta(&usart_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        xActivatedMember = xQueueSelectFromSet(xUartQueueSet, portMAX_DELAY);

        // 判断触发源并处理数据，根据串口数据的重要程度调整处理顺序
        if (xActivatedMember == xSemaphoreUART1_RX) {
            process_usart1_rx_data();  // 自定义控制器
        } else if (xActivatedMember == xSemaphoreUART5_RX) {
            process_uart5_rx_data();  // 福斯遥控器
        } else if (xActivatedMember == xSemaphoreUART10_RX) {
            process_uart10_rx_data(); // 裁判系统（电管）
        }
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
///**
// * @brief chassis 线程中所有发布者初始化
// */
//static void chassis_pub_init(void)
//{
//    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
//}
//
///**
// * @brief chassis 线程中所有订阅者初始化
// */
//static void chassis_sub_init(void)
//{
//    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
//    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
//    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
//}
//
///**
// * @brief chassis 线程中所有发布者推送更新话题
// */
//static void chassis_pub_push(void)
//{
//    pub_push_msg(pub_chassis,&chassis_fdb);
//}
///**
// * @brief chassis 线程中所有订阅者获取更新话题
// */
//static void chassis_sub_pull(void)
//{
//    sub_get_msg(sub_cmd, &chassis_cmd);
//    sub_get_msg(sub_trans, &trans_fdb);
//    sub_get_msg(sub_ins, &ins_data);
//}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == UART5)
    {
        if (Size > SBUS_RX_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        usart5_rx_size = Size;
        usart5_rx_buffer_index = usart5_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART5_RX, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART10)
    {
        // 判断接收的数据大小是否限制，如果超过，则不处理
        if (Size > REFEREE_RX_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        referee_rx_size = Size;
        referee_rx_buffer_index = referee_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART10_RX,NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART1)  // 修改判断条件
    {
        // 判断接收的数据大小是否限制，如果超过，则不处理
        if (Size > CUSTOMER_CONTROLLER_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        usart1_rx_size = Size;
        usart1_rx_buffer_index = usart1_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1_rx_buffer[usart1_rx_buffer_index],CUSTOMER_CONTROLLER_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART1_RX, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// DMA发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // 检查是否是USART1的DMA发送完成
    if (huart == &huart7) {
        // 释放信号量，表示DMA可以处理下一个缓冲区
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//        // 在中断中切换缓冲区
        usart7_tx_buffer_index = usart7_tx_buffer_index ^ 1;

        xSemaphoreGiveFromISR(xSemaphoreUART7_TX, &xHigherPriorityTaskWoken);
        // 如果需要唤醒更高优先级任务，则触发上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收发生错误后重启
        memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));							   // 清除接收缓存
    }

    if(huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE); // 接收发生错误后重启
        memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));// 清除双缓存
    }

    if(huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE); // 接收发生错误后重启
        memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));// 清除双缓存
    }
}

// 自定义DebugPrintf函数（类似printf）
//TODO:实际双缓冲区切换无效，不能做到一个buff[0]发送，另一个buff[1]接收，目前是buff[0]接收完立刻发送，同时切换到buff[1]接收完立刻发送
void USART7_DebugPrintf(const char *format, ...) {
    va_list args;
    uint16_t length;
    uint8_t finishedBuffer;

    // 等待DMA传输完成（获取信号量）
    if (xSemaphoreTake(xSemaphoreUART7_TX, portMAX_DELAY) == pdTRUE) {
        finishedBuffer = usart7_tx_buffer_index ^ 1; // 切换到下一个缓冲区

        // 使用可变参数格式化字符串到当前缓冲区
        va_start(args, format);
        length = vsnprintf(usart7_tx_debug_buffer[finishedBuffer], USART7_TX_DEBUG_BUFFER_SIZE, format, args);
        va_end(args);

        // 确保字符串长度有效
        if (length > 0) {
            // 使用DMA异步发送当前缓冲区数据
            HAL_UART_Transmit_DMA(&huart7, (uint8_t*)usart7_tx_debug_buffer[finishedBuffer], length);
        } else {
            // 长度无效时直接释放信号量
            xSemaphoreGive(xSemaphoreUART7_TX);
        }
    }
}