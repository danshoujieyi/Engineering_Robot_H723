//
// Created by 刘嘉俊 on 25-3-4.
//
#include "FreeRTOS.h"
#include "rc_sbus.h"
#include "usart10rec_task.h"
#include "referee_system.h"
#include "usart.h"
#include <string.h>


extern uint8_t dma_rx_buffer[2][FRAME_SIZE]; // 双缓冲区
extern uint8_t current_rx_buffer;        // 当前缓冲区索引
extern volatile uint8_t data_ready;      // 标志位，表示数据接收完成

extern uint8_t referee_rx_buffer_index;  // 当前使用的接收缓冲区
extern uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART10;
uint16_t refree_rec_size = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

    if(huart->Instance == UART5)
    {

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // 接收完毕后重启
    }

//    if (huart->Instance == USART10)
//    {
//        data_ready = 1; // 设置数据接收完成标志位
//        // 切换到另一个接收缓冲区
//        current_rx_buffer = (current_rx_buffer == 0) ? 1 : 0;
//        // 重新启动DMA接收
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE);
//    }

    if (huart->Instance == USART10)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        refree_rec_size = Size;
        uint8_t next_buf_index = referee_rx_buffer_index ^ 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[next_buf_index], REFEREE_RX_BUF_SIZE);
        referee_rx_buffer_index = next_buf_index;  // 最后更新索引
        xSemaphoreGiveFromISR(xSemaphoreUART10, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

//    if (huart->Instance == USART1)
//    {
//        referee_data_ready = 1; // 设置数据接收完成标志位
//        // 切换到另一个接收缓冲区
//        referee_rx_buffer_index = (referee_rx_buffer_index == 0) ? 1 : 0;
//        // 重新启动DMA接收
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
//    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // 接收发生错误后重启
        memset(sbus_rx_buf, 0, SBUS_RX_BUF_NUM);							   // 清除接收缓存
    }

    if(huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE); // 接收发生错误后重启
        memset(dma_rx_buffer, 0, sizeof(dma_rx_buffer));// 清除双缓存
    }

//    if(huart->Instance == USART1)
//    {
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE); // 接收发生错误后重启
//        memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));// 清除双缓存
//    }
}
