//
// Created by ���ο� on 25-7-17.
//

#include "vt13_vt03.h"
#include <string.h>
#include "cmsis_os.h"
#include "stdlib.h"
#include "crc8_crc16.h"

/* ȫ�ֱ������� */
static vt13_remote_unpack_obj_t vt13_remote_unpack_obj;  // ״̬�����ƶ���
static vt13_remote_parsed_data_t vt13_remote_parsed_data;  // ����������
vt13_remote_parsed_data_t vt13_remote_parsed_data_fdb;  // ��������
static SemaphoreHandle_t vt13_remote_mutex;  // ������


/* ��ʼ������������״̬�������� */
void vt13_remote_data_init(void) {
    // ��ʼ��״̬��
    memset(&vt13_remote_unpack_obj, 0, sizeof(vt13_remote_unpack_obj_t));
    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF1;  // ��ʼ״̬���ȴ�֡ͷ1
    vt13_remote_unpack_obj.data_len = 0;

    // ��ʼ����������
    memset(&vt13_remote_parsed_data, 0, sizeof(vt13_remote_parsed_data_t));
    memset(&vt13_remote_parsed_data_fdb, 0, sizeof(vt13_remote_parsed_data_t));

    // ��ʼ��������
    vt13_remote_mutex = xSemaphoreCreateMutex();
}


/* ״̬�����Ĵ����������ֽڽ������ϲ�ԭunpack���ܣ� */
void vt13_remote_data_process(uint8_t *data, uint16_t len) {
    // ���ֽڴ���������ֽ������ϲ�ԭunpack��ѭ���߼���
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];  // ��ǰ������ֽ�

        switch (vt13_remote_unpack_obj.step) {
            case VT13_REMOTE_STEP_SOF1:
                // �ȴ�֡ͷ1��sof_1��
                if (byte == REMOTE_SOF1) {
                    memset(vt13_remote_unpack_obj.data_buf, 0, REMOTE_FRAME_LEN);
                    vt13_remote_unpack_obj.data_buf[0] = byte;
                    vt13_remote_unpack_obj.data_len = 1;
                    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF2;
                }
                break;

            case VT13_REMOTE_STEP_SOF2:
                // �ȴ�֡ͷ2��sof_2��
                if (byte == REMOTE_SOF2) {
                    vt13_remote_unpack_obj.data_buf[1] = byte;
                    vt13_remote_unpack_obj.data_len = 2;
                    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_DATA;
                } else {
                    // ֡ͷ2��ƥ�䣬����
                    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF1;
                    vt13_remote_unpack_obj.data_len = 0;
                }
                break;

            case VT13_REMOTE_STEP_DATA:
                // ����֡ͷ���ʣ������
                if (vt13_remote_unpack_obj.data_len < REMOTE_FRAME_LEN - 1) {
                    vt13_remote_unpack_obj.data_buf[vt13_remote_unpack_obj.data_len++] = byte;
                } else {
                    // �������1�ֽڣ�����У��
                    vt13_remote_unpack_obj.data_buf[vt13_remote_unpack_obj.data_len++] = byte;
                    vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_VERIFY;
                }
                break;

            case VT13_REMOTE_STEP_VERIFY:
                // ����֡������ɣ�У��CRC
                if (vt13_remote_unpack_obj.data_len == REMOTE_FRAME_LEN) {
                    if (verify_CRC16_check_sum(vt13_remote_unpack_obj.data_buf, vt13_remote_unpack_obj.data_len)) {
                        // CRCУ��ͨ�������ý���������������
                        vt13_remote_parse_data(vt13_remote_unpack_obj.data_buf);

                    } else {
                        vt13_remote_parsed_data.online = 0;  // CRCУ��ʧ��
                    }
                }

                // ����״̬�����ȴ���һ֡
                vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF1;
                vt13_remote_unpack_obj.data_len = 0;
                break;

            default:
                // �쳣״̬����
                vt13_remote_unpack_obj.step = VT13_REMOTE_STEP_SOF1;
                vt13_remote_unpack_obj.data_len = 0;
                break;
        }
    }
}

/* ���ݽ������ĺ���������У��ͨ���������֡���� */
void vt13_remote_parse_data(uint8_t *data_buf) {
    // ��ʼ�����߱�־Ϊ��Ч
    vt13_remote_parsed_data.online = 1;

    // 1. ����64λλ�����ݣ�data[2]~data[9]��
    uint64_t bitfield;
    memcpy(&bitfield, &data_buf[2], 8);

    // 2. ����ͨ�����ݣ�11λ��ת��Ϊ���ֵ��
    vt13_remote_parsed_data.ch[0] = ((bitfield >> 0)  & 0x7FF) - RC_CENTER_VAL;
    vt13_remote_parsed_data.ch[1] = ((bitfield >> 11) & 0x7FF) - RC_CENTER_VAL;
    vt13_remote_parsed_data.ch[2] = ((bitfield >> 22) & 0x7FF) - RC_CENTER_VAL;
    vt13_remote_parsed_data.ch[3] = ((bitfield >> 33) & 0x7FF) - RC_CENTER_VAL;

    // 3. ��������λ
    vt13_remote_parsed_data.mode_sw = (bitfield >> 44) & 0x3;  // 2λ��λ����
    vt13_remote_parsed_data.pause   = (bitfield >> 46) & 0x1;  // ��ͣ��
    vt13_remote_parsed_data.fn_1    = (bitfield >> 47) & 0x1;  // �Զ��尴��1
    vt13_remote_parsed_data.fn_2    = (bitfield >> 48) & 0x1;  // �Զ��尴��2
    vt13_remote_parsed_data.wheel   = ((bitfield >> 49) & 0x7FF) - RC_CENTER_VAL;  // ����
    vt13_remote_parsed_data.trigger = (bitfield >> 60) & 0x1;  // �����

    // 4. ����������꣨int16_t���ͣ�
    memcpy(&vt13_remote_parsed_data.mouse_x, &data_buf[10], 2);
    memcpy(&vt13_remote_parsed_data.mouse_y, &data_buf[12], 2);
    memcpy(&vt13_remote_parsed_data.mouse_z, &data_buf[14], 2);

    // 5. ������갴������2λ��ȡ���λ��
    uint8_t mouse_btn = data_buf[16];
    vt13_remote_parsed_data.mouse_left   = (mouse_btn >> 0) & 0x1;
    vt13_remote_parsed_data.mouse_right  = (mouse_btn >> 2) & 0x1;
    vt13_remote_parsed_data.mouse_middle = (mouse_btn >> 4) & 0x1;

    // 6. �������̰�����16λ��
    memcpy(&vt13_remote_parsed_data.key, &data_buf[17], 2);

    // 7. ���ݷ�Χ��Ч��У��
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

    // 8. У��ͨ������±������ݣ��ӻ�����������
    if (data_valid) {
        xSemaphoreTake(vt13_remote_mutex, portMAX_DELAY);
        memcpy(&vt13_remote_parsed_data_fdb, &vt13_remote_parsed_data, sizeof(vt13_remote_parsed_data_t));
        xSemaphoreGive(vt13_remote_mutex);
    } else {
        vt13_remote_parsed_data.online = 0;  // ���������Ч
    }
}