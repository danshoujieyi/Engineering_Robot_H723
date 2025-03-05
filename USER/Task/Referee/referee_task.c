//
// Created by ���ο� on 25-3-4.
//

#include "referee_task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "referee_system.h"
#include "string.h"

// ��ͷ�ļ��������о��
QueueHandle_t xUARTQueue;

uint8_t referee_rx_buffer_index = 0;  // ��ǰʹ�õĽ��ջ�����
uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
volatile uint8_t referee_data_ready = 0;      // ��־λ����ʾ���ݽ������

void USART1_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //ʹ��DMA���ڽ���
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // �ر�DMA�Ĵ�������жϣ�����������ж�
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/*����ϵͳ�߳����*/
void Referee_Entry(void const * argument)
{
    xUARTQueue = xQueueCreate(1024, sizeof(uint8_t));
    if (xUARTQueue == NULL) {
        Error_Handler(); // ���д���ʧ��
    }

    /*����ϵͳ��ʼ��*/
    referee_system_init();
    USART1_DMA_Init();

    /*����ϵͳ���ݽ��*/
    while(1)
    {
        if (referee_data_ready) {
            referee_data_ready = 0; // �����־λ
            // д�����
            if (xQueueSend(xUARTQueue, referee_rx_buffer[referee_rx_buffer_index], 0) != pdPASS) {
                memset(referee_rx_buffer[referee_rx_buffer_index], 0, sizeof(referee_rx_buffer[referee_rx_buffer_index]));
            }
            // �������յ�������
            referee_data_unpack();
        }

        vTaskDelay(1);
    }
}
