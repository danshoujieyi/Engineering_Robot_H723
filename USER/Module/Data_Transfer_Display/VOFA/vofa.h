#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

extern UART_HandleTypeDef huart1;
#define VOFA_UART         huart1

// 需要一个重定义的printf函数或者自编写printf函数
// 用于RAWDATA协议和JUSTFLOAT协议的数据传输
// 使用JUSTFLOAT协议和UART传输
#define USE_PROTOCOL_JUSTFLOAT    1
#define USE_PROTOCOL_RAWDATA      0
#define USE_PROTOCOL_FIREWATER    0

#define USE_TRANSPORT_UART        1
#define USE_TRANSPORT_USB         0

// 根据配置决定是否定义相应宏
#if USE_PROTOCOL_RAWDATA || USE_PROTOCOL_FIREWATER
#define vofa_printf DebugPrintf
#endif

#if USE_TRANSPORT_UART
#define vofa_transmit(buf, len)   HAL_UART_Transmit((&VOFA_UART), (uint8_t *)(buf), (uint16_t)(len),1000)  //  HAL_UART_Transmit_DMA((&VOFA_UART), (uint8_t *)(buf), (uint16_t)(len))
#elif USE_TRANSPORT_USB
#define vofa_transmit(buf, len)    CDC_Transmit_HS((uint8_t *)(buf), len)// CDC_Transmit_FS((uint8_t *)(buf), len);
#endif

// 如果使用JUSTFLOAT协议，需要定义vofa_float_send
#if USE_PROTOCOL_JUSTFLOAT
uint8_t Vofa_Send_Float(const float* data, uint8_t channel);  //TODO:使用此函数不能再使用DebugPrintf函数，需要修改
#endif

/*---------------------------VOFA数据解析--------------------------------*/
#include <stdint.h>
#include <stdbool.h>
// 数据包相关常量(十六进制-ASCII)
#define VOFA_PACKET_HEADER_1 0x23 // '#'     // 帧头第一个字符
#define VOFA_PACKET_HEADER_2 0x50 // 'P'     // 帧头第二个字符
#define VOFA_PACKET_EQUALS 0x3D // '='         // 等号字符
#define VOFA_PACKET_TAIL 0x21 // '!'         // 帧尾字符
#define VOFA_DATA_BUFFER_SIZE 16     // 数据缓冲区大小
#define VOFA_DATA_MAX_SIZE (2+1+1+16+1+42)        // 完整数据包大小
// 命令类型定义
#define VOFA_CMD_P1  '1'             // 命令类型P1

// 解析状态机状态定义
// 解析状态机状态定义
typedef enum __attribute__((__packed__))
{
    STEP_HEADER_1 = 0,    // 等待帧头#
    STEP_HEADER_2,        // 等待帧头P
    STEP_CMD_TYPE,        // 记录命令类型
    STEP_EQUALS,          // 等待等号=
    STEP_DATA_VOFA,            // 接收数据
    STEP_TAIL             // 验证帧尾
} vofa_unpack_step_e;

// 解析器对象
typedef struct __attribute__((__packed__))
{
    vofa_unpack_step_e unpack_step;  // 当前解析状态
    uint8_t cmd_type;               // 命令类型字符
    uint8_t data_buffer[VOFA_DATA_BUFFER_SIZE]; // 数据缓冲区
    uint8_t data_index;             // 数据索引
    bool data_ready;                // 数据就绪标志
} vofa_unpack_obj_t;

// 解析后的数据结构
typedef struct {
    float p1_value;                 // P1命令值
    float p2_value;                 // P2命令值
    float p3_value;                 // P3命令值
    float p4_value;                 // P1命令值
    float p5_value;                 // P2命令值
    float p6_value;                 // P3命令值
    float p7_value;                 // P1命令值
    float p8_value;                 // P2命令值
    float p9_value;                 // P3命令值
    float p10_value;                 // P1命令值
    float p11_value;                 // P2命令值
    float p12_value;                 // P3命令值
    float p13_value;                 // P1命令值
    float p14_value;                 // P2命令值
    float p15_value;                 // P3命令值
} VofaData_t;

// 初始化解析器
void Vofa_Parser_Init(void);

// 处理接收到的数据
void VofaParser_ProcessData(uint8_t *data, uint16_t len);
void Vofa_Packet_Data_Save(void);

VofaData_t *Vofa_Get_Data(void);
/*---------------------------VOFA数据解析--------------------------------*/

#endif /* __VOFA_H__ */














