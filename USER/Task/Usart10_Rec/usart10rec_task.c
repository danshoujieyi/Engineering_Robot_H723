// Created by 刘嘉俊 on 25-1-13.

#include <string.h>
#include <stdio.h>
#include "usart10rec_task.h"
#include "usart.h"
#include "crc8_crc16.h"
#include <stdio.h>
// 数据帧长度 (帧头 + 数据区(6 * 4字节) + 校验 + 帧尾)


extern DMA_HandleTypeDef hdma_usart10_rx;

QueueHandle_t xQueueMotor; // FreeRTOS队列句柄，用于存储解码后的角度值
static uint8_t last_frame_seq = 0;    // 上一帧的帧序号
uint8_t dma_rx_buffer[2][FRAME_SIZE]; // 双缓冲区
uint8_t current_rx_buffer = 0;        // 当前缓冲区索引
volatile uint8_t data_ready = 0;      // 标志位，表示数据接收完成
float angles[6] = {0.0f};             // 存储解码后的角度值

void USART10_DMA_Init(void) {
    memset(dma_rx_buffer, 0, sizeof(dma_rx_buffer));
    // 配置DMA接收到第一个缓冲区
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

// 数据帧解析函数
uint8_t ParseFrame(uint8_t *buffer, float *angles) {
    RobotArmController_t rx_data;
    // 将接收到的缓冲区数据复制到结构体中
    memcpy(&rx_data, buffer, FRAME_SIZE);

    // 校验帧头起始字节
    if (rx_data.frame_header.sof != HEADER_SOF) {
        // 帧起始错误
        return 0;
    }

    // 校验帧头CRC8
    uint8_t crc8_received = rx_data.frame_header.crc8;
    rx_data.frame_header.crc8 = 0;  // 清零后计算
    append_CRC8_check_sum((uint8_t *)(&rx_data.frame_header), FRAME_HEADER_LENGTH);
    if (rx_data.frame_header.crc8 != crc8_received) {
        // CRC8校验失败
        return 0;
    }

    // 校验全帧CRC16
    uint16_t crc16_received = rx_data.frame_tail;
    rx_data.frame_tail = 0;  // 清零后计算
    append_CRC16_check_sum((uint8_t *)&rx_data, FRAME_SIZE);
    if (rx_data.frame_tail != crc16_received) {
        // CRC16校验失败
        return 0;
    }

    if (rx_data.cmd_id != ARM_CONTROLLER_CMD_ID) {
        // 命令码不匹配
        return 0;
    }

    // 检查帧序号是否连续（此处仅更新上一次帧序号，可根据需求处理异常）
    uint8_t expected_seq = (last_frame_seq + 1) % 256;
    if (rx_data.frame_header.seq != expected_seq) {
        // 帧序号不连续，可以在此处记录或做特殊处理
    }
    last_frame_seq = rx_data.frame_header.seq;

    // 解析数据区中的6个float数值
    memcpy(angles, rx_data.data, 6 * sizeof(float));

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
    xQueueMotor = xQueueCreate(40, 6 * sizeof(float));
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
            // 解析接收到的数据
            if (ParseFrame(dma_rx_buffer[current_rx_buffer], angles)) {
                // 将解码后的角度值写入队列
                if (xQueueSend(xQueueMotor, angles, 0) != pdPASS) {
                    // printf("Queue is full. Data discarded.\r\n"); // 队列已满，错误处理
                }
            }
        }
        vTaskDelay(1);
    }
    /* USER CODE END USART10_RecEntry */
}
