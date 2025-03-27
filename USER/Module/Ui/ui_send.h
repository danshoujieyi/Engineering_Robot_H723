//
// Created by 刘嘉俊 on 25-3-15.
//

#ifndef CTRBOARD_H7_ALL_UI_SEND_H
#define CTRBOARD_H7_ALL_UI_SEND_H

#include "stm32h7xx_hal.h"
#include "referee_system.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "crc8_crc16.h"

//1920*1080
#define CLIENT_MID_POSITION_X 960
#define CLIENT_MID_POSITION_Y 540

/********************其他end********************/

void client_info_update(void);

//画直线
graphic_data_struct_t draw_line(char *name,  //图形名
                                uint8_t operate_tpye,  //图形操作
                                uint8_t layer,  //图层数，0~9
                                uint8_t color,  //颜色
                                uint16_t width,  //线条宽度
                                uint16_t start_x,  //起点 x 坐标
                                uint16_t start_y,  //起点 y 坐标
                                uint16_t end_x,  //终点 x 坐标
                                uint16_t end_y);  //终点 y 坐标

//画矩形
graphic_data_struct_t draw_rectangle(char *name,  //图形名
                                     uint8_t operate_tpye,  //图形操作
                                     uint8_t layer,  //图层数，0~9
                                     uint8_t color,  //颜色
                                     uint16_t width,  //线条宽度
                                     uint16_t start_x,  //起点 x 坐标
                                     uint16_t start_y,  //起点 y 坐标
                                     uint16_t end_x,  //对角顶点 x 坐标
                                     uint16_t end_y);  //对角顶点 y 坐标

//画整圆
graphic_data_struct_t draw_circle(char *name,  //图形名
                                  uint8_t operate_tpye,  //图形操作
                                  uint8_t layer,  //图层数，0~9
                                  uint8_t color,  //颜色
                                  uint16_t width,  //线条宽度
                                  uint16_t start_x,  //圆心 x 坐标
                                  uint16_t start_y,  //圆心 y 坐标
                                  uint16_t radius);  //半径

//画椭圆
graphic_data_struct_t draw_ellipse(char *name,  //图形名
                                   uint8_t operate_tpye,  //图形操作
                                   uint8_t layer,  //图层数，0~9
                                   uint8_t color,  //颜色
                                   uint16_t width,  //线条宽度
                                   uint16_t start_x,  //圆心 x 坐标
                                   uint16_t start_y,  //圆心 y 坐标
                                   uint16_t end_x,  //x 半轴长度
                                   uint16_t end_y);  //y 半轴长度

//画圆弧
graphic_data_struct_t draw_arc(char *name,  //图形名
                               uint8_t operate_tpye,  //图形操作
                               uint8_t layer,  //图层数，0~9
                               uint8_t color,  //颜色
                               uint16_t start_angle,  //起始角度
                               uint16_t end_angle,  //终止角度
                               uint16_t width,  //线条宽度
                               uint16_t start_x,  //圆心 x 坐标
                               uint16_t start_y,  //圆心 y 坐标
                               uint16_t end_x,  //x 半轴长度
                               uint16_t end_y);  //y 半轴长度

//画浮点数
graphic_data_struct_t draw_float(char *name,  //图形名
                                 uint8_t operate_tpye,  //图形操作
                                 uint8_t layer,  //图层数，0~9
                                 uint8_t color,  //颜色
                                 uint16_t size,  //字体大小
                                 uint16_t decimal,  //小数位有效个数
                                 uint16_t width,  //线条宽度
                                 uint16_t start_x,  //起点 x 坐标
                                 uint16_t start_y,  //起点 y 坐标
                                 int32_t num);  //乘以 1000 后，以 32 位整型数，int32_t

//画整型数
graphic_data_struct_t draw_int(char *name,  //图形名
                               uint8_t operate_tpye,  //图形操作
                               uint8_t layer,  //图层数，0~9
                               uint8_t color,  //颜色
                               uint16_t size,  //字体大小
                               uint16_t width,  //线条宽度
                               uint16_t start_x,  //起点 x 坐标
                               uint16_t start_y,  //起点 y 坐标
                               int32_t num);  //32 位整型数，int32_t

//画字符串
graphic_data_struct_t draw_char(char *name,  //图形名
                                uint8_t operate_tpye,  //图形操作
                                uint8_t layer,  //图层数，0~9
                                uint8_t color,  //颜色
                                uint16_t size,  //字体大小
                                uint16_t length,  //字符长度
                                uint16_t width,  //线条宽度
                                uint16_t start_x,  //起点 x 坐标
                                uint16_t start_y);  //起点 y 坐标

uint8_t client_send_single_graphic(ext_client_custom_graphic_single_t data);
uint8_t client_send_double_graphic(ext_client_custom_graphic_double_t data);
uint8_t client_send_five_graphic(ext_client_custom_graphic_five_t data);
uint8_t client_send_seven_graphic(ext_client_custom_graphic_seven_t data);
uint8_t client_send_char(ext_client_custom_character_t data);
uint8_t client_graphic_delete_update(uint8_t delete_layer);

uint8_t uart_send_data(uint8_t *txbuf, uint16_t length);

#endif //CTRBOARD_H7_ALL_UI_SEND_H
