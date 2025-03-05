//
// Created by 刘嘉俊 on 25-3-4.
//

#include "referee_task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "referee_system.h"
#include "string.h"

// 在头文件声明队列句柄
QueueHandle_t xUARTQueue;

uint8_t referee_rx_buffer_index = 0;  // 当前使用的接收缓冲区
uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
volatile uint8_t referee_data_ready = 0;      // 标志位，表示数据接收完成

void USART1_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/*裁判系统线程入口*/
void Referee_Entry(void const * argument)
{
    xUARTQueue = xQueueCreate(1024, sizeof(uint8_t));
    if (xUARTQueue == NULL) {
        Error_Handler(); // 队列创建失败
    }

    /*裁判系统初始化*/
    referee_system_init();
    USART1_DMA_Init();

    /*裁判系统数据解包*/
    while(1)
    {
        if (referee_data_ready) {
            referee_data_ready = 0; // 清除标志位
            // 写入队列
            if (xQueueSend(xUARTQueue, referee_rx_buffer[referee_rx_buffer_index], 0) != pdPASS) {
                memset(referee_rx_buffer[referee_rx_buffer_index], 0, sizeof(referee_rx_buffer[referee_rx_buffer_index]));
            }
            // 解析接收到的数据
            referee_data_unpack();
        }

        vTaskDelay(1);
    }
}
