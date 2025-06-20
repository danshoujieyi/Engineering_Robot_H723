//
// Created by 刘嘉俊 on 25-6-19.
//

#include "JY61P.h"
#include <math.h>
#include <string.h>

jy61p_unpack_obj_t jy61p_unpack_obj = {0};
JY61P_data_t JY61P_data = {0};

// 初始化解析状态机
void JY61P_Init(void) {
    memset(&jy61p_unpack_obj, 0, sizeof(jy61p_unpack_obj_t));
    jy61p_unpack_obj.unpack_step = STEP_HEADER_SOF;
    memset(&JY61P_data, 0, sizeof(JY61P_data_t));
}

void JY61P_Data_Unpack(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (jy61p_unpack_obj.unpack_step) {
            case STEP_HEADER_SOF: // 等待包头0x55
                if (byte == JY61P_PACKET_HEADER) {
                    jy61p_unpack_obj.checksum = byte; // 开始计算校验和
                    jy61p_unpack_obj.byte_count = 0;   // 重置字节计数
                    jy61p_unpack_obj.unpack_step = STEP_DATA_TYPE;
                }
                break;

            case STEP_DATA_TYPE: // 等待数据类型
                if (byte == JY61P_ACC_PACKET ||
                    byte == JY61P_GYRO_PACKET ||
                    byte == JY61P_ANGLE_PACKET) {

                    jy61p_unpack_obj.packet_type = byte;
                    jy61p_unpack_obj.checksum += byte; // 累加校验和
                    jy61p_unpack_obj.unpack_step = STEP_DATA;
                } else {
                    // 无效数据类型，重置状态机
                    jy61p_unpack_obj.unpack_step = STEP_HEADER_SOF;
                }
                break;

            case STEP_DATA: // 接收数据字节
                if (jy61p_unpack_obj.byte_count < 8) {
                    // 直接存储字节到缓冲区
                    jy61p_unpack_obj.packet_buffer[jy61p_unpack_obj.byte_count] = byte;
                    jy61p_unpack_obj.checksum += byte; // 累加校验和
                    jy61p_unpack_obj.byte_count++;

                    // 检查是否接收完所有数据字节
                    if (jy61p_unpack_obj.byte_count >= 8) {
                        jy61p_unpack_obj.unpack_step = STEP_CHECKSUM;
                    }
                } else {
                    // 缓冲区溢出，重置状态机
                    jy61p_unpack_obj.unpack_step = STEP_HEADER_SOF;
                }
                break;

            case STEP_CHECKSUM: // 等待校验和
                // 验证校验和
                if (jy61p_unpack_obj.checksum == byte) {
                    // 校验成功，组合16位数据
                    int16_t raw_data[4];

                    // 组合16位数据 (小端模式)
                    raw_data[0] = (int16_t)(jy61p_unpack_obj.packet_buffer[0] |
                                            (jy61p_unpack_obj.packet_buffer[1] << 8));
                    raw_data[1] = (int16_t)(jy61p_unpack_obj.packet_buffer[2] |
                                            (jy61p_unpack_obj.packet_buffer[3] << 8));
                    raw_data[2] = (int16_t)(jy61p_unpack_obj.packet_buffer[4] |
                                            (jy61p_unpack_obj.packet_buffer[5] << 8));
                    raw_data[3] = (int16_t)(jy61p_unpack_obj.packet_buffer[6] |
                                            (jy61p_unpack_obj.packet_buffer[7] << 8));

                    // 保存数据到全局结构体
                    JY61P_Data_Save(jy61p_unpack_obj.packet_type, raw_data);
                }

                // 无论校验是否成功，都重置状态机
                jy61p_unpack_obj.unpack_step = STEP_HEADER_SOF;
                break;

            default:
                // 未知状态，重置状态机
                jy61p_unpack_obj.unpack_step = STEP_HEADER_SOF;
                break;
        }
    }
}

// 保存解析后的数据
void JY61P_Data_Save(uint8_t packet_type, int16_t *raw_data) {
    switch (packet_type) {
        case JY61P_ACC_PACKET: // 加速度数据
            JY61P_data.acc[0] = raw_data[0] * ACC_SCALE_FACTOR;
            JY61P_data.acc[1] = raw_data[1] * ACC_SCALE_FACTOR;
            JY61P_data.acc[2] = raw_data[2] * ACC_SCALE_FACTOR;
            JY61P_data.temperature = raw_data[3] * TEMP_SCALE_FACTOR; // 温度数据
            break;

        case JY61P_GYRO_PACKET: // 角速度数据
            JY61P_data.gyro[0] = raw_data[0] * GYRO_SCALE_FACTOR;
            JY61P_data.gyro[1] = raw_data[1] * GYRO_SCALE_FACTOR;
            JY61P_data.gyro[2] = raw_data[2] * GYRO_SCALE_FACTOR;
            break;

        case JY61P_ANGLE_PACKET: // 角度数据
            JY61P_data.angle[0] = raw_data[0] * ANGLE_SCALE_FACTOR;
            JY61P_data.angle[1] = raw_data[1] * ANGLE_SCALE_FACTOR;
            JY61P_data.angle[2] = raw_data[2] * ANGLE_SCALE_FACTOR;
            break;

        default:
            // 未知数据类型，不处理
            break;
    }
}