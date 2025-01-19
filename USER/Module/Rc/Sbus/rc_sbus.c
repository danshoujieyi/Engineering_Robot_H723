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


extern QueueHandle_t xQueueMotor; // FreeRTOS队列句柄，用于存储解码后的角度值
extern uint8_t last_frame_seq;    // 上一帧的帧序号
extern uint8_t dma_rx_buffer[2][FRAME_SIZE]; // 双缓冲区
extern uint8_t current_rx_buffer;        // 当前缓冲区索引
extern volatile uint8_t data_ready;      // 标志位，表示数据接收完成
extern float angles[6];             // 存储解码后的角度值

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

    if(huart->Instance == UART5)
    {
        if (Size <= SBUS_RX_BUF_NUM)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // 接收完毕后重启
            sbus_rc_decode( sbus_rx_buf);
		//	memset(rx_buff, 0, BUFF_SIZE);
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
        // 解析接收到的数据
        if (ParseFrame(dma_rx_buffer[current_rx_buffer], angles)) {
            // 将解码后的角度值写入队列
            if (xQueueSendFromISR(xQueueMotor, angles, NULL) != pdPASS) {
               // printf("Queue is full. Data discarded.\r\n"); // 队列已满，错误处理
            }
        }

        // 切换到另一个接收缓冲区
        current_rx_buffer = (current_rx_buffer == 0) ? 1 : 0;

        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buffer[current_rx_buffer], FRAME_SIZE);

        // 关闭DMA的传输过半中断，仅保留完成中断
        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // 接收发生错误后重启
        memset(sbus_rx_buf, 0, SBUS_RX_BUF_NUM);							   // 清除接收缓存
    }
}



///**
// * @brief 遥控器定时器超时回调函数
// */
//static void rc_lost_callback(void *paramete)
//{
//    printf("Sbus RC lost!");
//    memset(&rc_obj[NOW], 0, sizeof(rc_obj[NOW]));
//    rc_obj[NOW].sw1 = RC_UP;
//    rc_obj[NOW].sw2 = RC_UP;
//    rc_obj[NOW].sw3 = RC_UP;
//    rc_obj[NOW].sw4 = RC_UP;
//}
//
///**
// * @brief 串口 DMA 双缓冲初始化
// * @param rx1_buf 缓冲区1
// * @param rx2_buf 缓冲区2
// * @param dma_buf_num DMA缓冲区大小
// */
//static void rc_doub_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
//{
//    // 使能DMA串口接收
//    SET_BIT(huart5.Instance->CR1, USART_CR1_RE); // 使能UART5接收
//
//    // 使能空闲中断
//    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
//
//    // 失效DMA
//    __HAL_DMA_DISABLE(&hdma_uart5_rx);
//    while(hdma_uart5_rx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_uart5_rx);
//    }
//
//    hdma_uart5_rx.Instance-> = (uint32_t) & (UART5->DR);
//    // 内存缓冲区1
//    hdma_uart5_rx.Instance->M0AR = (uint32_t)(rx1_buf);
//    // 内存缓冲区2
//    hdma_uart5_rx.Instance->M1AR = (uint32_t)(rx2_buf);
//    // 数据长度
//    hdma_uart5_rx.Instance->NDTR = dma_buf_num;
//    // 使能双缓冲区
//    SET_BIT(hdma_uart5_rx.Instance->CR, DMA_SxCR_DBM);
//
//    // 使能DMA
//    __HAL_DMA_ENABLE(&hdma_uart5_rx);
//}
//
//
//
//void UART5_IRQHandler(void)
//{
//    if(huart5.Instance->SR & UART_FLAG_RXNE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart5);
//    }
//    else if(UART5->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;
//
//        __HAL_UART_CLEAR_PEFLAG(&huart5);
//
//        if ((hdma_uart5_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_uart5_rx);
//
//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_uart5_rx.Instance->NDTR;
//
//            //重新设定数据长度
//            hdma_uart5_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
//
//            //设定缓冲区1
//            hdma_uart5_rx.Instance->CR |= DMA_SxCR_CT;
//
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_uart5_rx);
//
//            if(this_time_rx_len == SBUS_FRAME_SIZE)
//            {
//                //处理遥控器数据
//                sbus_rc_decode(sbus_rx_buf[0]);
//               // rt_timer_start(rc_timer);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_uart5_rx);
//
//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_uart5_rx.Instance->NDTR;
//
//            //重新设定数据长度
//            hdma_uart5_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
//
//            //设定缓冲区0
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
//
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_uart5_rx);
//
//            if(this_time_rx_len == SBUS_FRAME_SIZE)
//            {
//                //处理遥控器数据
//                sbus_rc_decode(sbus_rx_buf[1]);
//              //  rt_timer_start(rc_timer);
//            }
//        }
//    }
//}
//

///**
// * @brief 初始化sbus_rc
// *
// * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
// */
//rc_obj_t *sbus_rc_init(void)
//{
//    /* DMA controller clock enable */
//    __HAL_RCC_DMA1_CLK_ENABLE();
//    /* DMA1_Stream1_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
//
//    huart5.Instance = UART5;  // 修改为UART5
//    huart5.Init.BaudRate = 100000;
//    huart5.Init.WordLength = UART_WORDLENGTH_9B;
//    huart5.Init.StopBits = UART_STOPBITS_2;
//    huart5.Init.Parity = UART_PARITY_EVEN;
//    huart5.Init.Mode = UART_MODE_TX_RX;
//    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
//    HAL_UART_Init(&huart5);  // 修改为huart5
//
//    rc_doub_dma_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
//
////    // 遥控器离线检测定时器相关
////    rc_timer = rt_timer_create("rc_sbus",
////                             rc_lost_callback,
////                             RT_NULL, 20,
////                             RT_TIMER_FLAG_PERIODIC);
////    rt_timer_start(rc_timer);
//
//    return rc_obj;
//}

