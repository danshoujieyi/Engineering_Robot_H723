//
// Created by 刘嘉俊 on 25-6-19.
//

#ifndef F407VGT6_CARARM_JY61P_H
#define F407VGT6_CARARM_JY61P_H

#include "stm32f4xx_hal.h"

// 数据包相关常量
#define JY61P_PACKET_HEADER 0x55  // 数据包头
#define JY61P_PACKET_SIZE   (66+33+4)    // 完整数据包大小

// 数据包类型定义
#define JY61P_ACC_PACKET    0x51  // 加速度数据包
#define JY61P_GYRO_PACKET   0x52  // 角速度数据包
#define JY61P_ANGLE_PACKET  0x53  // 角度数据包
#define JY61P_MAG_PACKET    0x54  // 磁场数据包
#define JY61P_PORT_PACKET   0x55  // 端口状态数据包

// 物理量转换系数
#define ACC_SCALE_FACTOR    (16.0f * 9.8f / 32768.0f)  // 加速度量程 (±16g)
#define GYRO_SCALE_FACTOR   (2000.0f / 32768.0f)       // 角速度量程 (±2000dps)
#define ANGLE_SCALE_FACTOR  (180.0f / 32768.0f)        // 角度量程 (±180°)
#define TEMP_SCALE_FACTOR   (1.0f / 100.0f)            // 温度量程

// 解析状态机状态定义
typedef enum {
    STEP_HEADER_SOF = 0,    // 等待包头0x55
    STEP_DATA_TYPE,          // 等待数据类型
    STEP_DATA,               // 接收数据字节
    STEP_CHECKSUM            // 等待校验和
} jy61p_unpack_step_t;

// JY61P数据包格式
typedef struct __attribute__((__packed__))
{
    jy61p_unpack_step_t unpack_step;  // 当前解析状态
    uint8_t packet_type;              // 当前包类型
    uint8_t packet_buffer[8];         // 原始数据缓冲区(8字节)
    uint8_t byte_count;               // 已接收字节计数
    uint8_t checksum;                 // 校验和累计值
} jy61p_unpack_obj_t;

typedef struct {
    float acc[3];     // 加速度 (m/s²)   X, Y, Z轴
    float gyro[3];    // 角速度 (deg/s)   X, Y, Z轴
    float angle[3];   // 角度 (deg)  ROLL, PITCH, YAW
    float temperature;// 温度 (°C)
} JY61P_data_t;

// 函数声明
void JY61P_Init(void);
void JY61P_Data_Unpack(uint8_t *data, uint16_t len);
void JY61P_Data_Save(uint8_t packet_type, int16_t *raw_data);
JY61P_data_t *JY61P_GetData(void);

//unsigned char KOY[5] = {0xFF,0xAA,0x69,0x88,0x55};				//解锁指令
//unsigned char Save[5] = {0xFF,0xAA,0x00,0x00,0x00};				//保存指令
//unsigned char Angle_CMD[5] = {0xFF,0xAA,0x01,0x08,0x00}; 		//角度参考（以当前实际位置，让xy轴角度归零）	延时3s
//unsigned char ACC_CMD[5] = {0xFF,0xAA,0x01,0x01,0x00};			//加速度校准  延时5s

#endif //F407VGT6_CARARM_JY61P_H
