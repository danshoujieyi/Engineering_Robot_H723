// Created by 刘嘉俊 on 25-1-13.

#include <string.h>
#include "usart1rec_task.h"
#include "usart.h"
#include "referee_system.h"

volatile uint8_t usart1_rx_buffer_index = 0;        // 当前使用的接收缓冲区索引
volatile uint16_t usart1_rx_size = 0;                 // 本次接收到的数据长度
uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];  // 双缓冲区
extern SemaphoreHandle_t xSemaphoreUART1;

extern struct referee_fdb_msg referee_fdb;
float float_values[7] = {0}; // 存储转换后的6个float


void USART1_DMA_Init(void) {
    memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void USART1_RecEntry(void const *argument) {

    referee_system_init();
    USART1_DMA_Init();

    uint8_t finishedBuffer;

    for (;;)
    {
        if (xSemaphoreTake(xSemaphoreUART1, portMAX_DELAY) == pdTRUE) {
            finishedBuffer = usart1_rx_buffer_index ^ 1;
            /* 此处依旧调用裁判系统数据解析函数，若有不同请自行替换 */
            referee_data_unpack(usart1_rx_buffer[finishedBuffer], usart1_rx_size);
            memset(usart1_rx_buffer[finishedBuffer], 0, CUSTOMER_CONTROLLER_BUF_SIZE);
            for (int i = 0; i < 7; i++) {
                uint8_t* byte_ptr = &referee_fdb.custom_robot_data.data[i * 4];
                memcpy(&float_values[i], byte_ptr, sizeof(float));
            }
        }

        vTaskDelay(1);
    }
}
