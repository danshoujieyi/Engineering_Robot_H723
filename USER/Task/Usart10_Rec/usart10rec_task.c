// Created by 刘嘉俊 on 25-1-13.

#include <string.h>
#include <stdio.h>
#include "usart10rec_task.h"
#include "usart.h"
#include <stdio.h>
// 数据帧长度 (帧头 + 数据区(6 * 4字节) + 校验 + 帧尾)


extern DMA_HandleTypeDef hdma_usart10_rx;

QueueHandle_t xQueueMotor; // FreeRTOS队列句柄，用于存储解码后的角度值
static uint8_t last_frame_seq = 0;    // 上一帧的帧序号
uint8_t dma_rx_buffer[2][FRAME_SIZE]; // 双缓冲区
uint8_t current_rx_buffer = 0;        // 当前缓冲区索引
volatile uint8_t data_ready = 0;      // 标志位，表示数据接收完成
float angles[6] = {0.0f};             // 存储解码后的角度值

// CRC校验函数
uint8_t CalculateChecksum(uint8_t *buffer, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum += buffer[i];
    }
    return checksum;
}

void USART10_DMA_Init(void) {
    memset(dma_rx_buffer, 0, sizeof(dma_rx_buffer));

    // 配置DMA接收到第一个缓冲区
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

// 数据帧解析函数
uint8_t ParseFrame(uint8_t *buffer, float *angles) {
    // 校验帧头和帧尾
    if (buffer[0] != 0xAA || buffer[27] != 0x55) {
      //  printf("Frame error: invalid header or footer\r\n");
        return 0; // 帧头或帧尾错误
    }

    // 校验和验证
    uint8_t checksum = CalculateChecksum(buffer + 1, 25);
    if (checksum != buffer[26]) {
      //  printf("Frame error: checksum mismatch\r\n");
        return 0; // 校验和错误
    }

    // 检测帧序号
    uint8_t frame_seq = buffer[1];
    if (frame_seq != (last_frame_seq + 1) % 256) {
       // printf("Frame sequence mismatch: expected %d, got %d\r\n", (last_frame_seq + 1) % 256, frame_seq);
    }
    last_frame_seq = frame_seq;

    // 解析6个float编码器值
    for (int i = 0; i < 6; i++) {
        memcpy(&angles[i], &buffer[2 + i * 4], sizeof(float));
    }

    return 1; // 解析成功
}

// 串口接收完成回调函数
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//    if (huart->Instance == USART10) {
//        data_ready = 1; // 设置数据接收完成标志位
//
//        // 解析接收到的数据
//        if (ParseFrame(dma_rx_buffer[current_rx_buffer], angles)) {
//            // 将解码后的角度值写入队列
//            if (xQueueSendFromISR(xQueueMotor, angles, NULL) != pdPASS) {
//                printf("Queue is full. Data discarded.\r\n"); // 队列已满，错误处理
//            }
//        }
//
//        // 切换到另一个接收缓冲区
//        current_rx_buffer = (current_rx_buffer == 0) ? 1 : 0;
//
//        // 重新启动DMA接收
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE);
//
//        // 关闭DMA的传输过半中断，仅保留完成中断
//        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
//    }
//}

void USART10_RecEntry(void const *argument) {
    /* USER CODE BEGIN USART10_RecEntry */

    // 创建队列，队列长度为10，每个队列项大小为6个float
    xQueueMotor = xQueueCreate(10, 6 * sizeof(float));
    if (xQueueMotor == NULL) {
        Error_Handler(); // 队列创建失败
    }
    // 初始化USART10 DMA接收
    USART10_DMA_Init();

    /* Infinite loop */
    for (;;) {
       // printf("USART10_RecEntry\r\n");

        if (data_ready) {
            data_ready = 0; // 清除标志位


            // 处理已解析的角度值
            // 此处可以添加额外逻辑，例如打印或传递数据给其他任务
        }

        vTaskDelay(1);
    }
    /* USER CODE END USART10_RecEntry */
}
