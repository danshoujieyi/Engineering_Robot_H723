//
// Created by 刘嘉俊 on 25-4-9.
//

#include "usart_recive_task.h"
#include "FreeRTOS.h"
#include "rc_sbus.h"
#include "referee_system.h"
#include "usart.h"
#include <string.h>

// 自定义控制器串口
static volatile uint8_t usart1_rx_buffer_index;  // 当前使用的接收缓冲区
static volatile uint16_t usart1_rx_size;
static uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART1;

// 福斯遥控器
static volatile uint8_t usart5_rx_buffer_index;  // 当前使用的接收缓冲区
static volatile uint16_t usart5_rx_size;
static uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART5;

// 裁判系统串口 10
static volatile uint8_t referee_rx_buffer_index;  // 当前使用的接收缓冲区
static volatile uint16_t referee_rx_size;
static uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART10;

// 外部引用
extern QueueSetHandle_t xUartQueueSet; // 定义队列集句柄,统一管理串口中断信号量
extern struct referee_fdb_msg referee_fdb;


void USART1_DMA_Init(void) {
    memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void USART5_DMA_Init(void) {
    memset(usart5_rx_buffer, 0, SBUS_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收完毕后重启
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void USART10_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

void process_usart1_data(void) {
    uint8_t finishedBuffer;

    // 非阻塞获取信号量（因为队列集已确保信号量有效）
    if (xSemaphoreTake(xSemaphoreUART1, 0) == pdTRUE) {
        finishedBuffer = usart1_rx_buffer_index ^ 1;
        /* 数据解析 */
        referee_data_unpack(usart1_rx_buffer[finishedBuffer], usart1_rx_size);

        memset(usart1_rx_buffer[finishedBuffer], 0, CUSTOMER_CONTROLLER_BUF_SIZE);

    }
}

void process_uart5_data(void) {
    uint8_t finishedBuffer;

    if (xSemaphoreTake(xSemaphoreUART5, 0) == pdTRUE) {
        finishedBuffer = usart5_rx_buffer_index ^ 1;
        /* SBUS协议解析 */
        sbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);

        memset(usart5_rx_buffer[finishedBuffer], 0, SBUS_RX_BUF_SIZE);
    }
}

void process_uart10_data(void) {
    uint8_t finishedBuffer;

    if (xSemaphoreTake(xSemaphoreUART10, 0) == pdTRUE) {
        finishedBuffer = referee_rx_buffer_index ^ 1;
        /* 裁判系统数据解析 */
        referee_data_unpack(referee_rx_buffer[finishedBuffer], referee_rx_size);

        memset(referee_rx_buffer[finishedBuffer], 0, REFEREE_RX_BUF_SIZE);
    }
}

void USARTRecTask_Entry(void const * argument)
{
    QueueSetMemberHandle_t xActivatedMember;

    USART1_DMA_Init();
    USART5_DMA_Init();
    USART10_DMA_Init();
    /* USER CODE BEGIN USARTRecTask_Entry */
    /* Infinite loop */
    for(;;)
    {
        // 阻塞等待任一信号量触发
        xActivatedMember = xQueueSelectFromSet(xUartQueueSet, portMAX_DELAY);

        // 判断触发源并处理数据，根据串口数据的重要程度调整处理顺序
        if (xActivatedMember == xSemaphoreUART1) {
            process_usart1_data();  // 自定义控制器
        } else if (xActivatedMember == xSemaphoreUART5) {
            process_uart5_data();  // 福斯遥控器
        } else if (xActivatedMember == xSemaphoreUART10) {
            process_uart10_data(); // 裁判系统（电管）
        }

        vTaskDelay(1);
    }
    /* USER CODE END USARTRecTask_Entry */
}

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

        xSemaphoreGiveFromISR(xSemaphoreUART5, NULL);
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

        xSemaphoreGiveFromISR(xSemaphoreUART10,NULL);
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

        xSemaphoreGiveFromISR(xSemaphoreUART1, NULL);
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