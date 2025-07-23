//
// Created by 刘嘉俊 on 25-7-17.
//

#include "vt13_vt03.h"
#include <string.h>
#include "cmsis_os.h"
#include "stdlib.h"
#include "crc8_crc16.h"

/* 全局变量定义 */
static vt13_remote_unpack_obj_t vt13_remote_unpack_obj;  // 状态机控制对象
static vt13_remote_parsed_data_t vt13_remote_parsed_data;  // 解析后数据
vt13_remote_parsed_data_t vt13_remote_parsed_data_fdb;  // 备份数据
static SemaphoreHandle_t vt13_remote_mutex;  // 互斥锁


/* 初始化函数：重置状态机和数据 */
void vt13_remote_data_init(void) {
    // 初始化状态机
    memset(&vt13_remote_unpack_obj, 0, sizeof(vt13_remote_unpack_obj_t));
    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF1;  // 初始状态：等待帧头1
    vt13_remote_unpack_obj.data_len = 0;

    // 初始化解析数据
    memset(&vt13_remote_parsed_data, 0, sizeof(vt13_remote_parsed_data_t));
    memset(&vt13_remote_parsed_data_fdb, 0, sizeof(vt13_remote_parsed_data_t));

    // 初始化互斥锁
    vt13_remote_mutex = xSemaphoreCreateMutex();
}


/* 状态机核心处理函数：逐字节解析（合并原unpack功能） */
void vt13_remote_data_process(uint8_t *data, uint16_t len) {
    // 逐字节处理输入的字节流（合并原unpack的循环逻辑）
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];  // 当前处理的字节

        switch (vt13_remote_unpack_obj.step) {
            case VT13_REMOTE_STEP_SOF1:
                // 等待帧头1（sof_1）
                if (byte == REMOTE_SOF1) {
                    memset(vt13_remote_unpack_obj.data_buf, 0, REMOTE_FRAME_LEN);
                    vt13_remote_unpack_obj.data_buf[0] = byte;
                    vt13_remote_unpack_obj.data_len = 1;
                    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF2;
                }
                break;

            case VT13_REMOTE_STEP_SOF2:
                // 等待帧头2（sof_2）
                if (byte == REMOTE_SOF2) {
                    vt13_remote_unpack_obj.data_buf[1] = byte;
                    vt13_remote_unpack_obj.data_len = 2;
                    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_DATA;
                } else {
                    // 帧头2不匹配，重置
                    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF1;
                    vt13_remote_unpack_obj.data_len = 0;
                }
                break;

            case VT13_REMOTE_STEP_DATA:
                // 接收帧头后的剩余数据
                if (vt13_remote_unpack_obj.data_len < REMOTE_FRAME_LEN - 1) {
                    vt13_remote_unpack_obj.data_buf[vt13_remote_unpack_obj.data_len++] = byte;
                } else {
                    // 接收最后1字节，进入校验
                    vt13_remote_unpack_obj.data_buf[vt13_remote_unpack_obj.data_len++] = byte;
                    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_VERIFY;
                }
                break;

            case VT13_REMOTE_STEP_VERIFY:
                // 完整帧接收完成，校验CRC
                if (vt13_remote_unpack_obj.data_len == REMOTE_FRAME_LEN) {
                    if (verify_CRC16_check_sum(vt13_remote_unpack_obj.data_buf, vt13_remote_unpack_obj.data_len)) {
                        // CRC校验通过，调用解析函数处理数据
                        vt13_remote_parse_data(vt13_remote_unpack_obj.data_buf);

                    } else {
                        vt13_remote_parsed_data.online = 0;  // CRC校验失败
                    }
                }

                // 重置状态机，等待下一帧
                vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF1;
                vt13_remote_unpack_obj.data_len = 0;
                break;

            default:
                // 异常状态重置
                vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF1;
                vt13_remote_unpack_obj.data_len = 0;
                break;
        }
    }
}

/* 数据解析核心函数：处理校验通过后的完整帧数据 */
void vt13_remote_parse_data(uint8_t *data_buf) {
    // 初始化在线标志为有效
    vt13_remote_parsed_data.online = 1;

    // 1. 解析64位位域数据（data[2]~data[9]）
    uint64_t bitfield;
    memcpy(&bitfield, &data_buf[2], 8);

    // 2. 解析通道数据（11位，转换为相对值）
    vt13_remote_parsed_data.ch[0] = ((bitfield >> 0)  & 0x7FF) - RC_CENTER_VAL;
    vt13_remote_parsed_data.ch[1] = ((bitfield >> 11) & 0x7FF) - RC_CENTER_VAL;
    vt13_remote_parsed_data.ch[2] = ((bitfield >> 22) & 0x7FF) - RC_CENTER_VAL;
    vt13_remote_parsed_data.ch[3] = ((bitfield >> 33) & 0x7FF) - RC_CENTER_VAL;

    // 3. 解析控制位
    vt13_remote_parsed_data.mode_sw = (bitfield >> 44) & 0x3;  // 2位挡位开关
    vt13_remote_parsed_data.pause   = (bitfield >> 46) & 0x1;  // 暂停键
    vt13_remote_parsed_data.fn_1    = (bitfield >> 47) & 0x1;  // 自定义按键1
    vt13_remote_parsed_data.fn_2    = (bitfield >> 48) & 0x1;  // 自定义按键2
    vt13_remote_parsed_data.wheel   = ((bitfield >> 49) & 0x7FF) - RC_CENTER_VAL;  // 拨轮
    vt13_remote_parsed_data.trigger = (bitfield >> 60) & 0x1;  // 扳机键

    // 4. 解析鼠标坐标（int16_t类型）
    memcpy(&vt13_remote_parsed_data.mouse_x, &data_buf[10], 2);
    memcpy(&vt13_remote_parsed_data.mouse_y, &data_buf[12], 2);
    memcpy(&vt13_remote_parsed_data.mouse_z, &data_buf[14], 2);

    // 5. 解析鼠标按键（各2位，取最低位）
    uint8_t mouse_btn = data_buf[16];
    vt13_remote_parsed_data.mouse_left   = (mouse_btn >> 0) & 0x1;
    vt13_remote_parsed_data.mouse_right  = (mouse_btn >> 2) & 0x1;
    vt13_remote_parsed_data.mouse_middle = (mouse_btn >> 4) & 0x1;

    // 6. 解析键盘按键（16位）
    memcpy(&vt13_remote_parsed_data.key, &data_buf[17], 2);

    // 7. 数据范围有效性校验
    uint8_t data_valid = 1;
    for (uint8_t i = 0; i < 4; i++) {
        if (abs(vt13_remote_parsed_data.ch[i]) > VALID_RANGE) {
            data_valid = 0;
            break;
        }
    }
    if (abs(vt13_remote_parsed_data.wheel) > VALID_RANGE) {
        data_valid = 0;
    }

    // 8. 校验通过则更新备份数据（加互斥锁保护）
    if (data_valid) {
        xSemaphoreTake(vt13_remote_mutex, portMAX_DELAY);
        memcpy(&vt13_remote_parsed_data_fdb, &vt13_remote_parsed_data, sizeof(vt13_remote_parsed_data_t));
        xSemaphoreGive(vt13_remote_mutex);
    } else {
        vt13_remote_parsed_data.online = 0;  // 标记数据无效
    }
}