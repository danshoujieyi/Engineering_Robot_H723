//
// Created by 刘嘉俊 on 25-7-17.
//

#ifndef CTRBOARD_H7_ALL_VT13_VT03_H
#define CTRBOARD_H7_ALL_VT13_VT03_H

#include "stm32h7xx_hal.h"

typedef struct __attribute__((__packed__))
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
} vt13_remote_data_t;

/* 协议常量定义 */
#define REMOTE_SOF1       0xA9       // 帧头1
#define REMOTE_SOF2       0x53       // 帧头2
#define REMOTE_FRAME_LEN  21         // 一帧数据长度
#define RC_CENTER_VAL     1024       // 通道中间值
#define VALID_RANGE       660        // 通道有效范围（1024±660 → 364~1684）

/* 解析后数据结构 */
typedef struct {
    uint8_t  online;        // 在线标志（1:在线, 0:离线）
    int16_t  ch[4];         // 通道0~3原始值（已减中间值，范围-660~660）
    uint8_t  mode_sw;       // 挡位开关（0:C, 1:N, 2:S）
    uint8_t  pause;         // 暂停按键（0:未按, 1:按下）
    uint8_t  fn_1;          // 自定义按键左（0:未按, 1:按下）
    uint8_t  fn_2;          // 自定义按键右（0:未按, 1:按下）
    int16_t  wheel;         // 拨轮值（已减中间值，范围-660~660）
    uint8_t  trigger;       // 扳机键（0:未按, 1:按下）
    int16_t  mouse_x;       // 鼠标X轴速度
    int16_t  mouse_y;       // 鼠标Y轴速度
    int16_t  mouse_z;       // 鼠标Z轴速度（滚轮）
    uint8_t  mouse_left;    // 鼠标左键（0:未按, 1:按下）
    uint8_t  mouse_right;   // 鼠标右键（0:未按, 1:按下）
    uint8_t  mouse_middle;  // 鼠标中键（0:未按, 1:按下）
    uint16_t key;           // 键盘按键位映射（bit0~15对应按键）
} vt13_remote_parsed_data_t;

/* 状态机状态定义 */
typedef enum {
    VT13_REMOTE_STEP_SOF1,       // 等待帧头1
    VT13_REMOTE_STEP_SOF2,       // 等待帧头2
    VT13_REMOTE_STEP_DATA,       // 接收数据内容
    VT13_REMOTE_STEP_VERIFY,     // 校验数据
    VT13_REMOTE_STEP_RESET       // 重置状态
} vt13_remote_unpack_step_t;

/* 状态机控制结构体 */
typedef struct {
    vt13_remote_unpack_step_t step;       // 当前解析步骤
    uint8_t data_buf[REMOTE_FRAME_LEN];   // 数据缓冲区（存储一帧完整数据）
    uint16_t data_len;                    // 已接收数据长度
} vt13_remote_unpack_obj_t;

void vt13_remote_data_init(void);
void vt13_remote_data_process(uint8_t *data, uint16_t len);
void vt13_remote_parse_data(uint8_t *data_buf);

#endif //CTRBOARD_H7_ALL_VT13_VT03_H
