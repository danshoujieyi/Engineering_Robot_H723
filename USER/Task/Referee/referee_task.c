//
// Created by ���ο� on 25-3-4.
//

#include "referee_task.h"
#include "FreeRTOS.h"
#include "usart.h"
#include "referee_system.h"
#include "semphr.h"
#include <string.h>

/* USART10������ϵͳ���� */
volatile uint8_t referee_rx_buffer_index = 0;       // ��ǰʹ�õĽ��ջ���������
volatile uint16_t referee_rx_size = 0;              // ���ν��յ������ݳ���
uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];    // ˫������
extern SemaphoreHandle_t xSemaphoreUART10;          // ֪ͨ�������ź���

void USART10_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //ʹ��DMA���ڽ���
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // �ر�DMA�Ĵ�������жϣ�����������ж�
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

/*����ϵͳ�߳����*/
void Referee_Entry(void const * argument)
{
    /*����ϵͳ��ʼ��*/
    referee_system_init();
    USART10_DMA_Init();
    uint8_t finishedBuffer;

    for (;;) {
        if (xSemaphoreTake(xSemaphoreUART10, portMAX_DELAY) == pdTRUE) {
            finishedBuffer = referee_rx_buffer_index ^ 1;
            referee_data_unpack(referee_rx_buffer[finishedBuffer], referee_rx_size);
            memset(referee_rx_buffer[finishedBuffer], 0, REFEREE_RX_BUF_SIZE);
        }
        vTaskDelay(1);
    }
}
