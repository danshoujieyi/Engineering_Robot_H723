//
// Created by ���ο� on 25-3-15.
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

/********************����end********************/

void client_info_update(void);

//��ֱ��
graphic_data_struct_t draw_line(char *name,  //ͼ����
                                uint8_t operate_tpye,  //ͼ�β���
                                uint8_t layer,  //ͼ������0~9
                                uint8_t color,  //��ɫ
                                uint16_t width,  //�������
                                uint16_t start_x,  //��� x ����
                                uint16_t start_y,  //��� y ����
                                uint16_t end_x,  //�յ� x ����
                                uint16_t end_y);  //�յ� y ����

//������
graphic_data_struct_t draw_rectangle(char *name,  //ͼ����
                                     uint8_t operate_tpye,  //ͼ�β���
                                     uint8_t layer,  //ͼ������0~9
                                     uint8_t color,  //��ɫ
                                     uint16_t width,  //�������
                                     uint16_t start_x,  //��� x ����
                                     uint16_t start_y,  //��� y ����
                                     uint16_t end_x,  //�ԽǶ��� x ����
                                     uint16_t end_y);  //�ԽǶ��� y ����

//����Բ
graphic_data_struct_t draw_circle(char *name,  //ͼ����
                                  uint8_t operate_tpye,  //ͼ�β���
                                  uint8_t layer,  //ͼ������0~9
                                  uint8_t color,  //��ɫ
                                  uint16_t width,  //�������
                                  uint16_t start_x,  //Բ�� x ����
                                  uint16_t start_y,  //Բ�� y ����
                                  uint16_t radius);  //�뾶

//����Բ
graphic_data_struct_t draw_ellipse(char *name,  //ͼ����
                                   uint8_t operate_tpye,  //ͼ�β���
                                   uint8_t layer,  //ͼ������0~9
                                   uint8_t color,  //��ɫ
                                   uint16_t width,  //�������
                                   uint16_t start_x,  //Բ�� x ����
                                   uint16_t start_y,  //Բ�� y ����
                                   uint16_t end_x,  //x ���᳤��
                                   uint16_t end_y);  //y ���᳤��

//��Բ��
graphic_data_struct_t draw_arc(char *name,  //ͼ����
                               uint8_t operate_tpye,  //ͼ�β���
                               uint8_t layer,  //ͼ������0~9
                               uint8_t color,  //��ɫ
                               uint16_t start_angle,  //��ʼ�Ƕ�
                               uint16_t end_angle,  //��ֹ�Ƕ�
                               uint16_t width,  //�������
                               uint16_t start_x,  //Բ�� x ����
                               uint16_t start_y,  //Բ�� y ����
                               uint16_t end_x,  //x ���᳤��
                               uint16_t end_y);  //y ���᳤��

//��������
graphic_data_struct_t draw_float(char *name,  //ͼ����
                                 uint8_t operate_tpye,  //ͼ�β���
                                 uint8_t layer,  //ͼ������0~9
                                 uint8_t color,  //��ɫ
                                 uint16_t size,  //�����С
                                 uint16_t decimal,  //С��λ��Ч����
                                 uint16_t width,  //�������
                                 uint16_t start_x,  //��� x ����
                                 uint16_t start_y,  //��� y ����
                                 int32_t num);  //���� 1000 ���� 32 λ��������int32_t

//��������
graphic_data_struct_t draw_int(char *name,  //ͼ����
                               uint8_t operate_tpye,  //ͼ�β���
                               uint8_t layer,  //ͼ������0~9
                               uint8_t color,  //��ɫ
                               uint16_t size,  //�����С
                               uint16_t width,  //�������
                               uint16_t start_x,  //��� x ����
                               uint16_t start_y,  //��� y ����
                               int32_t num);  //32 λ��������int32_t

//���ַ���
graphic_data_struct_t draw_char(char *name,  //ͼ����
                                uint8_t operate_tpye,  //ͼ�β���
                                uint8_t layer,  //ͼ������0~9
                                uint8_t color,  //��ɫ
                                uint16_t size,  //�����С
                                uint16_t length,  //�ַ�����
                                uint16_t width,  //�������
                                uint16_t start_x,  //��� x ����
                                uint16_t start_y);  //��� y ����

uint8_t client_send_single_graphic(ext_client_custom_graphic_single_t data);
uint8_t client_send_double_graphic(ext_client_custom_graphic_double_t data);
uint8_t client_send_five_graphic(ext_client_custom_graphic_five_t data);
uint8_t client_send_seven_graphic(ext_client_custom_graphic_seven_t data);
uint8_t client_send_char(ext_client_custom_character_t data);
uint8_t client_graphic_delete_update(uint8_t delete_layer);

uint8_t uart_send_data(uint8_t *txbuf, uint16_t length);

#endif //CTRBOARD_H7_ALL_UI_SEND_H
