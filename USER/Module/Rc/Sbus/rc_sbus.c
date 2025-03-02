// Tip: 遥控器接收模块
#include "rc_sbus.h"
#include "rm_config.h"
#include <stm32h723xx.h>
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "usart10rec_task.h"

#define DBG_TAG           "rc.sbus"
#define DBG_LVL DBG_INFO

#define NOW 0
#define LAST 1
#define abs(x) ((x > 0) ? x : -x)

/* C板预留的遥控器接口（具备取反电路）为 uart3 */
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

// 自定义串口遥控器
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;

//接收原始数据，为25个字节，给了36个字节长度，防止DMA传输越界
uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
rc_obj_t rc_obj[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST
// TODO: 目前遥控器发送端关闭并不会检测为丢失，只有接收端异常才会判断为离线，
//       后续需要修改判断条件，预期效果是发送端关闭后判断为离线

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00

/**
 * @brief 遥控器sbus数据解析
 *
 * @param rc_obj 指向sbus_rc实例的指针
 */
void sbus_rc_decode(uint8_t *buff){

    if ((buff[0] != SBUS_HEAD) || (buff[24] != SBUS_END))
        return;

    if (buff[23] == 0x0C)
        rc_obj->online = 0;
    else
        rc_obj->online = 1;

        /* 下面是正常遥控器数据的处理 */
        rc_obj[NOW].ch1 = (buff[1] | buff[2] << 8) & 0x07FF;
        rc_obj[NOW].ch1 -= 1024;
        rc_obj[NOW].ch2 = (buff[2] >> 3 | buff[3] << 5) & 0x07FF;
        rc_obj[NOW].ch2 -= 1024;
        rc_obj[NOW].ch3 = (buff[3] >> 6 | buff[4] << 2 | buff[5] << 10) & 0x07FF;
        rc_obj[NOW].ch3 -= 1024;
        rc_obj[NOW].ch4 = (buff[5] >> 1 | buff[6] << 7) & 0x07FF;
        rc_obj[NOW].ch4 -= 1024;
    /* 旋钮值获取 */
       rc_obj[NOW].ch5 =((buff[6] >> 4 | buff[7] << 4) & 0x07FF);
       rc_obj[NOW].ch5 -= 1024;
       rc_obj[NOW].ch6 =((buff[7] >> 7 | buff[8] << 1 | buff[9] << 9) & 0x07FF);
       rc_obj[NOW].ch6 -= 1024;
        /* 防止遥控器零点有偏差 */
        if(rc_obj[NOW].ch1 <= 10 && rc_obj[NOW].ch1 >= -10)
            rc_obj[NOW].ch1 = 0;
        if(rc_obj[NOW].ch2 <= 10 && rc_obj[NOW].ch2 >= -10)
            rc_obj[NOW].ch2 = 0;
        if(rc_obj[NOW].ch3 <= 10 && rc_obj[NOW].ch3 >= -10)
            rc_obj[NOW].ch3 = 0;
        if(rc_obj[NOW].ch4 <= 10 && rc_obj[NOW].ch4 >= -10)
            rc_obj[NOW].ch4 = 0;
        if(rc_obj[NOW].ch5 <= 10 && rc_obj[NOW].ch5 >= -10)
            rc_obj[NOW].ch5 = 0;
        if(rc_obj[NOW].ch6 <= 10 && rc_obj[NOW].ch6 >= -10)
            rc_obj[NOW].ch6 = 0;
        /* 拨杆值获取 */
        rc_obj[NOW].sw1 = ((buff[9] >> 2 | buff[10] << 6) & 0x07FF);
        rc_obj[NOW].sw2 = ((buff[10] >> 5 | buff[11] << 3) & 0x07FF);
        rc_obj[NOW].sw3=((buff[12] | buff[13] << 8) & 0x07FF);
        rc_obj[NOW].sw4 =((buff[13] >> 3 | buff[14] << 5) & 0x07FF);
//        /* 遥控器异常值处理，函数直接返回 */
//        if ((abs(rc_obj[NOW].ch1) > RC_MAX_VALUE) || \
//        (abs(rc_obj[NOW].ch2) > RC_MAX_VALUE) || \
//        (abs(rc_obj[NOW].ch3) > RC_MAX_VALUE) || \
//        (abs(rc_obj[NOW].ch4) > RC_MAX_VALUE) || \
//        (abs(rc_obj[NOW].ch5) > RC_MAX_VALUE) || \
//        (abs(rc_obj[NOW].ch6) > RC_MAX_VALUE))
//        {
//            memset(&rc_obj[NOW], 0, sizeof(rc_obj_t));
//            return ;
//        }

        rc_obj[LAST] = rc_obj[NOW];

}



extern uint8_t dma_rx_buffer[2][FRAME_SIZE]; // 双缓冲区
extern uint8_t current_rx_buffer;        // 当前缓冲区索引
extern volatile uint8_t data_ready;      // 标志位，表示数据接收完成

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

    if(huart->Instance == UART5)
    {
        if (Size <= SBUS_RX_BUF_NUM)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // 接收完毕后重启
        }
        else  // 接收数据长度大于BUFF_SIZE，错误处理
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // 接收完毕后重启
            memset(sbus_rx_buf, 0, SBUS_RX_BUF_NUM);
        }
    }

    if (huart->Instance == USART10)
    {
        data_ready = 1; // 设置数据接收完成标志位
        // 切换到另一个接收缓冲区
        current_rx_buffer = (current_rx_buffer == 0) ? 1 : 0;
        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE);
    }

//    if (huart->Instance == USART10)
//    {
//        data_ready = 1; // 设置数据接收完成标志位
//        // 切换到另一个接收缓冲区
//        current_rx_buffer = (current_rx_buffer == 0) ? 1 : 0;
//        // 重新启动DMA接收
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE);
//    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // 接收发生错误后重启
        memset(sbus_rx_buf, 0, SBUS_RX_BUF_NUM);							   // 清除接收缓存
    }

    if(huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE); // 接收发生错误后重启
        memset(dma_rx_buffer[(current_rx_buffer == 0) ? 1 : 0], 0, FRAME_SIZE);							   // 清除接收缓存
    }
}


