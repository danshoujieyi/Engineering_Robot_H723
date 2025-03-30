//
// Created by 刘嘉俊 on 25-3-4.
//

#include "referee_task.h"
#include "FreeRTOS.h"
#include "usart.h"
#include "referee_system.h"
#include "semphr.h"
#include <string.h>
#include "cmd_task.h"
#include "rc_sbus.h"
#include "arm_math.h"

///* USART10：裁判系统接收 */
//volatile uint8_t referee_rx_buffer_index = 0;       // 当前使用的接收缓冲区索引
//volatile uint16_t referee_rx_size = 0;              // 本次接收到的数据长度
//uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];    // 双缓冲区
//extern SemaphoreHandle_t xSemaphoreUART10;          // 通知任务处理信号量
//
//void USART10_DMA_Init(void) {
//    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
//    //使能DMA串口接收
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
//    // 关闭DMA的传输过半中断，仅保留完成中断
//    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
//}

volatile uint8_t usart5_rx_buffer_index = 0;      // 当前使用的接收缓冲区索引
volatile uint16_t usart5_rx_size = 0;               // 本次接收到的数据长度
uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];      // 双缓冲区
extern SemaphoreHandle_t xSemaphoreUART5;           // 通知任务处理信号量

extern sbus_data_t sbus_data_fdb;


void USART5_DMA_Init(void) {
    memset(usart5_rx_buffer, 0, SBUS_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收完毕后重启
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

/*裁判系统线程入口*/
void RefereeTask_Entry(void const * argument)
{
    /*裁判系统初始化*/
//    referee_system_init();
//    USART10_DMA_Init();
//    uint8_t finishedBuffer;

    sbus_data_init();
    USART5_DMA_Init();
    uint8_t finishedBuffer;
    float cnt = arm_sin_f32(30 * PI/180.0); // 用于DSP测试
    float a = 0;
    for (;;) {
//        if (xSemaphoreTake(xSemaphoreUART10, portMAX_DELAY) == pdTRUE) {
//            finishedBuffer = referee_rx_buffer_index ^ 1;
//            referee_data_unpack(referee_rx_buffer[finishedBuffer], referee_rx_size);
//            memset(referee_rx_buffer[finishedBuffer], 0, REFEREE_RX_BUF_SIZE);
//        }
        a = cnt;
        if (xSemaphoreTake(xSemaphoreUART5, pdMS_TO_TICKS(portMAX_DELAY)) == pdTRUE) {
            /* 使用刚完成接收数据的缓冲区 */
            finishedBuffer = usart5_rx_buffer_index ^ 1;
            sbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);
            memset(usart5_rx_buffer[finishedBuffer], 0, SBUS_RX_BUF_SIZE);
            remote_to_cmd_sbus();
           // cmd_sbus_keyboard();

        }

        vTaskDelay(1);
    }
}
