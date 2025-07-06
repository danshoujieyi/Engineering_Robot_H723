/**
    在微软的MS-DOS和Windows中，使用“回车CR('r')”和“换行LF('n')”两个字符作为换行符;
    Windows系统里面，每行结尾是 回车+换行(CR+LF)，即“rn”；
    Unix系统里，每行结尾只有 换行CR，即“n”；
    Mac系统里，每行结尾是 回车CR 即'r'；
    所以我们平时编写文件的回车符应该确切来说叫做回车换行符；
*/
/**
 * RAWDATA协议：发送什么就显示什么，不做任何处理，直接printf输出。
 * FIREWATER协议：也是使用printf输出，但是在数据通道之间用逗号分隔，以换行符\n作为发送标志（或者\r\n）。
 * 虽然都使用printf，但是RAWDATA是发什么就显示什么，比如“123.456，刘嘉俊”直接输出一模一样，可以发数据，也可以发字符
 * 但是FIREWATER协议中专门指《数据通道》，而不是数据，以英文逗号区分每一个数据通道以换行符当作发送标志，专门发送数据，不包括字符。
 * FIREWATER协议：printf("%d,%d,%d,%d\r\n",data1,data2,data3);//将data1，data2，data3换成相应想要看到的数据即可
 * 发送四个数据通道，分别是四个整数通道
 * FIREWATER协议的原理就是先把数据通过printf中的sprintf函数转换成字符串，然后再通过UART发送（调用prinf原理），所以占用资源多。
 */
/**
 * JUSTFLOAT协议：只发送浮点数数据，不能使用printf输出字符，也不能输出字符，不可以使用任何涉及sprintf的函数（本质就是不能传字符，为了追求效率和速度）。
 * 发送小端格式的浮点数数据（小端指低位在前，高位在后），直接将浮点数转换成字节数组（十六进制）发送。一个float类型数据占用4个字节。
    for (int i = 0; i < 6; i++) {
        uint8_t *src = (uint8_t *)&values[i];
        uint8_t *dst = &tx_data->data[i * 4];
        memcpy(dst, src, sizeof(float)); // 自动处理4个字节
    }
    含义：将 float 变量的地址强制转换为uint8_t*类型，使我们可以逐个字节地访问 float 的内存表示，即访问四个字节的每一个地址。
    借助memcpy函数，把 4 个字节的数据从src复制到tx_data->data数组对应的位置，每次都会塞满四个字节数据（传地址实现）。
 * 解析小端格式的浮点数数据：
    for (int i = 0; i < 7; i++) {
        uint8_t *byte_ptr = &buffer[i * 4];
        memcpy(&float_values[i], byte_ptr, sizeof(float));
    }
    含义：
    这段代码原本想实现的功能是：从buffer这个字节数组里提取 7 个小端格式的浮点数，再把它们存到float_values数组中。具体步骤如下：
    进行 7 次循环，每次循环处理一个浮点数。
    每次循环时，找到当前浮点数在buffer里的起始位置(传地址）。
    借助memcpy函数，把 4 个字节的数据复制到float_values数组对应的位置。
    小端格式指的是数据的低位字节存放在内存的低地址处，高位字节存放在内存的高地址处。
    举个例子，有一个 32 位浮点数0x12345678，在小端格式下，它在内存中的存储顺序是0x78 0x56 0x34 0x12。
    当使用memcpy函数复制这 4 个字节的数据时，系统会按照小端格式来解释这些字节，从而得到正确的浮点数数值。
    至于为什么memcpy能够把四个字节的数据转换成一个浮点数。把上面的所有内容复制发送给AI求解释。
    memcpy用于复制指定数量的字节数据到目标地址。不关心类型，只看地址。
 * 需要定义发送数据包数组buffer，然后将数据转换成浮点数格式存入buffer中，最后添加
 */
#include "vofa.h"
#include "math.h"
#include "usart_task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * 发送浮点数数据到VOFA+上位机（使用JUSTFLOAT协议）
 * @param data 待发送的浮点数数组
 * @param count 待发送的数据数量（最大支持20个，受缓冲区大小限制）
 */
// JUSTFLOAT协议：只发送浮点数数据。专用函数
uint8_t Vofa_Send_Float(const float* data, uint8_t channel)
{
    // 检查参数有效性
    if (data == NULL || channel == 0 || channel > 20) {
        return -1; // 无效参数
    }

    // 计算缓冲区大小（每个float占4字节，再加4字节结束符）
    const uint8_t bufferSize = channel * 4 + 4;
    uint8_t tempData[84];   // 传输缓冲区，最大16*4+4=68字节

    // 复制浮点数数据到发送缓冲区
    memcpy(tempData, (uint8_t *)data, (sizeof(float)*channel));

    // 写入JUSTFLOAT协议的结尾标识
    tempData[bufferSize - 4] = 0x00;
    tempData[bufferSize - 3] = 0x00;
    tempData[bufferSize - 2] = 0x80;
    tempData[bufferSize - 1] = 0x7f;

    // 使用DMA发送数据必须严格等待DMA发送完成，否则会导致数据丢失或重叠。
//    if (isTransmitting == 0) {
//        isTransmitting = 1;
//        vofa_transmit(tempData, bufferSize);
//    }

    vofa_transmit(tempData, bufferSize);

    return 0; // 发送成功
}

/* 发送完成回调 */
// DMA发送完成回调函数
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//    // 检查是否是USART1的DMA发送完成
//    if (huart->Instance == USART1) {
//        // 释放信号量，表示DMA可以处理下一个缓冲区
//        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//        isTransmitting = false; // 设置发送状态为非传输中
//
//        xSemaphoreGiveFromISR(xSemaphoreUART1_TX, &xHigherPriorityTaskWoken);
//        // 如果需要唤醒更高优先级任务，则触发上下文切换
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }
//}


/*---------------------------VOFA数据解析--------------------------------*/
static vofa_unpack_obj_t vofa_unpack_obj = {0};
static VofaData_t vofa_data = {0};

// 初始化解析器
void Vofa_Parser_Init(void) {
    memset(&vofa_unpack_obj, 0, sizeof(vofa_unpack_obj_t));
    vofa_unpack_obj.unpack_step = STEP_HEADER_1;
    memset(&vofa_data, 0, sizeof(VofaData_t));
}

// 处理接收到的数据
// 处理接收到的数据
void VofaParser_ProcessData(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (vofa_unpack_obj.unpack_step) {
            case STEP_HEADER_1: // 等待帧头#
                if (byte == VOFA_PACKET_HEADER_1) {
                    vofa_unpack_obj.data_index = 0;
                    vofa_unpack_obj.data_ready = false; // 重置数据就绪标志
                    vofa_unpack_obj.cmd_type = 0; // 重置命令类型
                    vofa_unpack_obj.unpack_step = STEP_HEADER_2;
                }
                break;

            case STEP_HEADER_2: // 等待帧头P
                if (byte == VOFA_PACKET_HEADER_2) {
                    vofa_unpack_obj.unpack_step = STEP_CMD_TYPE;
                } else {
                    vofa_unpack_obj.unpack_step = STEP_HEADER_1; // 重置状态机
                }
                break;

            case STEP_CMD_TYPE: // 记录命令类型
                vofa_unpack_obj.cmd_type = byte;
                vofa_unpack_obj.unpack_step = STEP_EQUALS;
                break;

            case STEP_EQUALS: // 等待等号=
                if (byte == VOFA_PACKET_EQUALS) {
                    vofa_unpack_obj.data_index = 0;
                    vofa_unpack_obj.unpack_step = STEP_DATA_VOFA;
                } else {
                    vofa_unpack_obj.unpack_step = STEP_HEADER_1; // 格式错误，重置
                }
                break;

            case STEP_DATA_VOFA: // 接收数据
                if (vofa_unpack_obj.data_index < VOFA_DATA_BUFFER_SIZE - 1) {
                    if (byte == VOFA_PACKET_TAIL) {
                        // 遇到帧尾，结束数据接收
                        vofa_unpack_obj.data_buffer[vofa_unpack_obj.data_index] = '\0';  // 字符数据末尾自动补'\0'，满足atof函数使用要求
                        vofa_unpack_obj.data_ready = true;
                        vofa_unpack_obj.unpack_step = STEP_TAIL;
                    } else {
                        // 存储数据字节
                        vofa_unpack_obj.data_buffer[vofa_unpack_obj.data_index++] = byte;
                    }
                } else {
                    vofa_unpack_obj.unpack_step = STEP_HEADER_1; // 缓冲区溢出，重置
                }
                break;

            case STEP_TAIL: // 验证帧尾

                // 验证成功，处理数据
                Vofa_Packet_Data_Save();

                vofa_unpack_obj.unpack_step = STEP_HEADER_1;
                vofa_unpack_obj.data_ready = false;
                break;

            default:
                vofa_unpack_obj.unpack_step = STEP_HEADER_1; // 未知状态，重置
                break;
        }
    }
}


// 处理完整数据包
void Vofa_Packet_Data_Save(void) {
    if (!vofa_unpack_obj.data_ready) return;

    // 将ASCII字符串转换为浮点数
    // atof直接处理带有'\0'结尾的字符串，可以自动处理正负号、小数点
    float value = (float)atof((char*)vofa_unpack_obj.data_buffer);

    // 根据命令类型保存数据
    switch (vofa_unpack_obj.cmd_type) {
        case '1':
            vofa_data.p1_value = value;
            break;
        case '2':
            vofa_data.p2_value = value;
            break;
        case '3':
            vofa_data.p3_value = value;
            break;
        case '4':
            vofa_data.p4_value = value;
            break;
        case '5':
            vofa_data.p5_value = value;
            break;
        case '6':
            vofa_data.p6_value = value;
            break;
        case '7':
            vofa_data.p7_value = value;
            break;
        case '8':
            vofa_data.p8_value = value;
            break;
        case '9':
            vofa_data.p9_value = value;
            break;
        case 'A':
            vofa_data.p10_value = value;
            break;
        case 'B':
            vofa_data.p11_value = value;
            break;
        case 'C':
            vofa_data.p12_value = value;
            break;
        case 'D':
            vofa_data.p13_value = value;
            break;
        case 'E':
            vofa_data.p14_value = value;
            break;
        case 'F':
            vofa_data.p15_value = value;
        default:
            // 未知命令类型，不处理
            break;
    }
}

VofaData_t* Vofa_Get_Data(void) {
    return &vofa_data; // 返回指向VOFA数据的指针
}
/**
*  // 获取并打印数据
    JY61P_data_t *data = JY61P_GetData();
    printf("加速度: X=%.2f Y=%.2f Z=%.2f\n", data->acc[0], data->acc[1], data->acc[2]);
    printf("角速度: X=%.2f Y=%.2f Z=%.2f\n", data->gyro[0], data->gyro[1], data->gyro[2]);
    printf("角度: 滚转=%.2f 俯仰=%.2f 偏航=%.2f\n", data->angle[0], data->angle[1], data->angle[2]);
*/
/*---------------------------VOFA数据解析--------------------------------*/
