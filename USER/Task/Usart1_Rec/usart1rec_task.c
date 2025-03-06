// Created by 刘嘉俊 on 25-1-13.

#include <string.h>
#include "usart1rec_task.h"
#include "usart.h"
#include "referee_system.h"

uint8_t usart1_rx_buffer_index = 0;  // 当前使用的接收缓冲区
uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART1;
extern uint16_t usart1_rec_size;

void USART1_DMA_Init(void) {
    memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void USART1_RecEntry(void const *argument) {

    referee_system_init();
    USART1_DMA_Init();
    uint8_t* active_buff = NULL;

    for (;;) {
            if (xSemaphoreTake(xSemaphoreUART1, portMAX_DELAY) == pdTRUE) {
                active_buff = usart1_rx_buffer[usart1_rx_buffer_index ^ 1];
                referee_data_unpack(active_buff, usart1_rec_size);
            }

        vTaskDelay(1);
    }
}
