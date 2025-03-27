//
// Created by 刘嘉俊 on 25-3-4.
//
#include "FreeRTOS.h"
#include "rc_sbus.h"
#include "referee_system.h"
#include "usart.h"
#include <string.h>

extern volatile uint8_t usart5_rx_buffer_index;  // 当前使用的接收缓冲区
extern volatile uint16_t usart5_rx_size;
extern uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART5;

extern volatile uint8_t referee_rx_buffer_index;  // 当前使用的接收缓冲区
extern volatile uint16_t referee_rx_size;
extern uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART10;

extern volatile uint8_t usart1_rx_buffer_index;  // 当前使用的接收缓冲区
extern volatile uint16_t usart1_rx_size;
extern uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART1;

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
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
        xSemaphoreGiveFromISR(xSemaphoreUART5, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

//    if (huart->Instance == USART10)
//    {
//        // 判断接收的数据大小是否限制，如果超过，则不处理
//        if (Size > REFEREE_RX_BUF_SIZE)
//        {
//            return;
//        }
//        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//        referee_rx_size = Size;
//        referee_rx_buffer_index = referee_rx_buffer_index ^ 1;
//
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
//        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
//        xSemaphoreGiveFromISR(xSemaphoreUART10,NULL);
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }

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
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        xSemaphoreGiveFromISR(xSemaphoreUART1, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收发生错误后重启
        // 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
        memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));							   // 清除接收缓存
    }
//
//    if(huart->Instance == USART10)
//    {
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE); // 接收发生错误后重启
//        // 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
//        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
//        memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));// 清除双缓存
//    }

    if(huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE); // 接收发生错误后重启
        // 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));// 清除双缓存
    }
}
