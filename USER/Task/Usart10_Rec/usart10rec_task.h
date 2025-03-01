//
// Created by 刘嘉俊 on 25-1-13.
//

#ifndef CTRBOARD_H7_ALL_USART10REC_TASK_H
#define CTRBOARD_H7_ALL_USART10REC_TASK_H
#include "cmsis_os.h"
//#define FRAME_SIZE 28
uint8_t ParseFrame(uint8_t *buffer, float *angles);


// 定义数据帧各部分长度
#define HEADER_SOF 0xA5
#define FRAME_HEADER_LENGTH 5 // 帧头长度
#define CMD_ID_LENGTH 2       // 命令码长度
#define DATA_LENGTH 30        // 数据段长度
#define FRAME_TAIL_LENGTH 2   // 帧尾长度
// 数据帧长度 (帧头 + 命令码 + 数据区 + 帧尾)
// 5字节帧头 + 2字节命令码 + 30字节数据 + 2字节帧尾 #define FRAME_SIZE 39
#define FRAME_SIZE (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个数据帧长度

#define ARM_CONTROLLER_CMD_ID 0x0302 // 六轴机械臂控制器命令码

// 六轴机械臂控制器数据包结构体
// 使用 `__attribute__((packed))` 禁用结构体成员的对齐，确保结构体紧凑地排列在内存中。
// 这样可以避免因为结构体对齐而导致额外的填充字节（例如帧头和数据段之间的多余字节）。
// 这个属性确保结构体中的成员紧密排列，减少内存浪费，尤其在需要精确控制数据格式时非常重要。

typedef struct __attribute__((packed)) { // 禁用结构体对齐
    struct __attribute__((packed)){  // 禁用帧头部分结构体的对齐
        uint8_t sof;              // 起始字节，固定值为0xA5，表示数据帧的开始
        uint16_t data_length;     // 数据段长度（2字节），表示数据区的字节数
        uint8_t seq;              // 包序号（1字节），表示当前数据包的序列号
        uint8_t crc8;             // 帧头 CRC8 校验（1字节），用于数据校验
    } frame_header;               // 帧头部分

    uint16_t cmd_id;              // 命令码（2字节），表示当前帧的命令类型
    uint8_t data[30];             // 数据段（30字节），用于存放实际的数据
    uint16_t frame_tail;         // 帧尾 CRC16 校验（2字节），用于确保数据传输完整性
} RobotArmController_t;


#endif //CTRBOARD_H7_ALL_USART10REC_TASK_H
