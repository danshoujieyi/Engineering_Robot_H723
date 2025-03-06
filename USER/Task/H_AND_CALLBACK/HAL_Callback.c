//
// Created by ���ο� on 25-3-4.
//
#include "FreeRTOS.h"
#include "rc_sbus.h"
#include "referee_system.h"
#include "usart.h"
#include <string.h>

extern uint8_t usart5_rx_buffer_index;  // ��ǰʹ�õĽ��ջ�����
extern uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART5;
uint16_t usart5_rec_size = 0;

extern uint8_t referee_rx_buffer_index;  // ��ǰʹ�õĽ��ջ�����
extern uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART10;
uint16_t refree_rec_size = 0;

extern uint8_t usart1_rx_buffer_index;  // ��ǰʹ�õĽ��ջ�����
extern uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART1;
uint16_t usart1_rec_size = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == UART5)
    {
        if (Size > SBUS_RX_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        usart5_rec_size = Size;
        uint8_t next_buf_index = usart5_rx_buffer_index ^ 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[next_buf_index], SBUS_RX_BUF_SIZE);
        usart5_rx_buffer_index = next_buf_index;  // ����������
        xSemaphoreGiveFromISR(xSemaphoreUART5, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART10)
    {
        // �жϽ��յ����ݴ�С�Ƿ����ƣ�����������򲻴���
        if (Size > REFEREE_RX_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        refree_rec_size = Size;
        uint8_t next_buf_index = referee_rx_buffer_index ^ 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[next_buf_index], REFEREE_RX_BUF_SIZE);
        referee_rx_buffer_index = next_buf_index;  // ����������
        xSemaphoreGiveFromISR(xSemaphoreUART10, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART1)  // �޸��ж�����
    {
        // �жϽ��յ����ݴ�С�Ƿ����ƣ�����������򲻴���
        if (Size > CUSTOMER_CONTROLLER_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        usart1_rec_size = Size;
        uint8_t next_buf_index = usart1_rx_buffer_index ^ 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1_rx_buffer[next_buf_index],CUSTOMER_CONTROLLER_BUF_SIZE);
        usart1_rx_buffer_index = next_buf_index;  // ��������
        xSemaphoreGiveFromISR(xSemaphoreUART1, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // ���շ������������
        memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));							   // ������ջ���
    }

    if(huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE); // ���շ������������
        memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));// ���˫����
    }

    if(huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE); // ���շ������������
        memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));// ���˫����
    }
}
