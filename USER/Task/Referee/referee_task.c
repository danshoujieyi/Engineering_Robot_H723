//
// Created by 刘嘉俊 on 25-3-4.
//

#include "referee_task.h"
#include "FreeRTOS.h"
#include "usart.h"
#include "referee_system.h"
#include "semphr.h"
#include <string.h>

uint8_t referee_rx_buffer_index = 0;  // 当前使用的接收缓冲区
uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART10;
extern uint16_t refree_rec_size;

void USART10_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

/*裁判系统线程入口*/
void Referee_Entry(void const * argument)
{
    /*裁判系统初始化*/
    referee_system_init();
    USART10_DMA_Init();
    uint8_t* active_buff = NULL;

    for (;;) {
        if (xSemaphoreTake(xSemaphoreUART10, portMAX_DELAY) == pdTRUE) {
            active_buff = referee_rx_buffer[referee_rx_buffer_index ^ 1];  // 交替缓冲区
            referee_data_unpack(active_buff, refree_rec_size);
        }
        vTaskDelay(1);
    }
}
