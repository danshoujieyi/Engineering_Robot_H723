//
// Created by ���ο� on 25-3-4.
//
#include "FreeRTOS.h"
#include "rc_sbus.h"
#include "usart10rec_task.h"
#include "referee_system.h"
#include "usart.h"
#include <string.h>


extern uint8_t dma_rx_buffer[2][FRAME_SIZE]; // ˫������
extern uint8_t current_rx_buffer;        // ��ǰ����������
extern volatile uint8_t data_ready;      // ��־λ����ʾ���ݽ������

extern uint8_t referee_rx_buffer_index;  // ��ǰʹ�õĽ��ջ�����
extern uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART10;
uint16_t refree_rec_size = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

    if(huart->Instance == UART5)
    {

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // ������Ϻ�����
    }

//    if (huart->Instance == USART10)
//    {
//        data_ready = 1; // �������ݽ�����ɱ�־λ
//        // �л�����һ�����ջ�����
//        current_rx_buffer = (current_rx_buffer == 0) ? 1 : 0;
//        // ��������DMA����
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE);
//    }

    if (huart->Instance == USART10)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        refree_rec_size = Size;
        uint8_t next_buf_index = referee_rx_buffer_index ^ 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[next_buf_index], REFEREE_RX_BUF_SIZE);
        referee_rx_buffer_index = next_buf_index;  // ����������
        xSemaphoreGiveFromISR(xSemaphoreUART10, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

//    if (huart->Instance == USART1)
//    {
//        referee_data_ready = 1; // �������ݽ�����ɱ�־λ
//        // �л�����һ�����ջ�����
//        referee_rx_buffer_index = (referee_rx_buffer_index == 0) ? 1 : 0;
//        // ��������DMA����
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
//    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // ���շ������������
        memset(sbus_rx_buf, 0, SBUS_RX_BUF_NUM);							   // ������ջ���
    }

    if(huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE); // ���շ������������
        memset(dma_rx_buffer, 0, sizeof(dma_rx_buffer));// ���˫����
    }

//    if(huart->Instance == USART1)
//    {
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE); // ���շ������������
//        memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));// ���˫����
//    }
}
