//
// Created by ���ο� on 25-7-17.
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

/* Э�鳣������ */
#define REMOTE_SOF1       0xA9       // ֡ͷ1
#define REMOTE_SOF2       0x53       // ֡ͷ2
#define REMOTE_FRAME_LEN  21         // һ֡���ݳ���
#define RC_CENTER_VAL     1024       // ͨ���м�ֵ
#define VALID_RANGE       660        // ͨ����Ч��Χ��1024��660 �� 364~1684��

/* ���������ݽṹ */
typedef struct {
    uint8_t  online;        // ���߱�־��1:����, 0:���ߣ�
    int16_t  ch[4];         // ͨ��0~3ԭʼֵ���Ѽ��м�ֵ����Χ-660~660��
    uint8_t  mode_sw;       // ��λ���أ�0:C, 1:N, 2:S��
    uint8_t  pause;         // ��ͣ������0:δ��, 1:���£�
    uint8_t  fn_1;          // �Զ��尴����0:δ��, 1:���£�
    uint8_t  fn_2;          // �Զ��尴���ң�0:δ��, 1:���£�
    int16_t  wheel;         // ����ֵ���Ѽ��м�ֵ����Χ-660~660��
    uint8_t  trigger;       // �������0:δ��, 1:���£�
    int16_t  mouse_x;       // ���X���ٶ�
    int16_t  mouse_y;       // ���Y���ٶ�
    int16_t  mouse_z;       // ���Z���ٶȣ����֣�
    uint8_t  mouse_left;    // ��������0:δ��, 1:���£�
    uint8_t  mouse_right;   // ����Ҽ���0:δ��, 1:���£�
    uint8_t  mouse_middle;  // ����м���0:δ��, 1:���£�
    uint16_t key;           // ���̰���λӳ�䣨bit0~15��Ӧ������
} vt13_remote_parsed_data_t;

/* ״̬��״̬���� */
typedef enum {
    VT13_REMOTE_STEP_SOF1,       // �ȴ�֡ͷ1
    VT13_REMOTE_STEP_SOF2,       // �ȴ�֡ͷ2
    VT13_REMOTE_STEP_DATA,       // ������������
    VT13_REMOTE_STEP_VERIFY,     // У������
    VT13_REMOTE_STEP_RESET       // ����״̬
} vt13_remote_unpack_step_t;

/* ״̬�����ƽṹ�� */
typedef struct {
    vt13_remote_unpack_step_t step;       // ��ǰ��������
    uint8_t data_buf[REMOTE_FRAME_LEN];   // ���ݻ��������洢һ֡�������ݣ�
    uint16_t data_len;                    // �ѽ������ݳ���
} vt13_remote_unpack_obj_t;

void vt13_remote_data_init(void);
void vt13_remote_data_process(uint8_t *data, uint16_t len);
void vt13_remote_parse_data(uint8_t *data_buf);

#endif //CTRBOARD_H7_ALL_VT13_VT03_H
