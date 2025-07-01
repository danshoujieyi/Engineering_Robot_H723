#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

extern UART_HandleTypeDef huart1;
#define VOFA_UART         huart1

// ��Ҫһ���ض����printf���������Ա�дprintf����
// ����RAWDATAЭ���JUSTFLOATЭ������ݴ���
// ʹ��JUSTFLOATЭ���UART����
#define USE_PROTOCOL_JUSTFLOAT    1
#define USE_PROTOCOL_RAWDATA      0
#define USE_PROTOCOL_FIREWATER    0

#define USE_TRANSPORT_UART        1
#define USE_TRANSPORT_USB         0

// �������þ����Ƿ�����Ӧ��
#if USE_PROTOCOL_RAWDATA || USE_PROTOCOL_FIREWATER
#define vofa_printf DebugPrintf
#endif

#if USE_TRANSPORT_UART
#define vofa_transmit(buf, len)   HAL_UART_Transmit((&VOFA_UART), (uint8_t *)(buf), (uint16_t)(len),1000)  //  HAL_UART_Transmit_DMA((&VOFA_UART), (uint8_t *)(buf), (uint16_t)(len))
#elif USE_TRANSPORT_USB
#define vofa_transmit(buf, len)    CDC_Transmit_HS((uint8_t *)(buf), len)// CDC_Transmit_FS((uint8_t *)(buf), len);
#endif

// ���ʹ��JUSTFLOATЭ�飬��Ҫ����vofa_float_send
#if USE_PROTOCOL_JUSTFLOAT
uint8_t Vofa_Send_Float(const float* data, uint8_t channel);  //TODO:ʹ�ô˺���������ʹ��DebugPrintf��������Ҫ�޸�
#endif

/*---------------------------VOFA���ݽ���--------------------------------*/
#include <stdint.h>
#include <stdbool.h>
// ���ݰ���س���(ʮ������-ASCII)
#define VOFA_PACKET_HEADER_1 0x23 // '#'     // ֡ͷ��һ���ַ�
#define VOFA_PACKET_HEADER_2 0x50 // 'P'     // ֡ͷ�ڶ����ַ�
#define VOFA_PACKET_EQUALS 0x3D // '='         // �Ⱥ��ַ�
#define VOFA_PACKET_TAIL 0x21 // '!'         // ֡β�ַ�
#define VOFA_DATA_BUFFER_SIZE 16     // ���ݻ�������С
#define VOFA_DATA_MAX_SIZE (2+1+1+16+1+42)        // �������ݰ���С
// �������Ͷ���
#define VOFA_CMD_P1  '1'             // ��������P1

// ����״̬��״̬����
// ����״̬��״̬����
typedef enum __attribute__((__packed__))
{
    STEP_HEADER_1 = 0,    // �ȴ�֡ͷ#
    STEP_HEADER_2,        // �ȴ�֡ͷP
    STEP_CMD_TYPE,        // ��¼��������
    STEP_EQUALS,          // �ȴ��Ⱥ�=
    STEP_DATA_VOFA,            // ��������
    STEP_TAIL             // ��֤֡β
} vofa_unpack_step_e;

// ����������
typedef struct __attribute__((__packed__))
{
    vofa_unpack_step_e unpack_step;  // ��ǰ����״̬
    uint8_t cmd_type;               // ���������ַ�
    uint8_t data_buffer[VOFA_DATA_BUFFER_SIZE]; // ���ݻ�����
    uint8_t data_index;             // ��������
    bool data_ready;                // ���ݾ�����־
} vofa_unpack_obj_t;

// ����������ݽṹ
typedef struct {
    float p1_value;                 // P1����ֵ
    float p2_value;                 // P2����ֵ
    float p3_value;                 // P3����ֵ
    float p4_value;                 // P1����ֵ
    float p5_value;                 // P2����ֵ
    float p6_value;                 // P3����ֵ
    float p7_value;                 // P1����ֵ
    float p8_value;                 // P2����ֵ
    float p9_value;                 // P3����ֵ
    float p10_value;                 // P1����ֵ
    float p11_value;                 // P2����ֵ
    float p12_value;                 // P3����ֵ
    float p13_value;                 // P1����ֵ
    float p14_value;                 // P2����ֵ
    float p15_value;                 // P3����ֵ
} VofaData_t;

// ��ʼ��������
void Vofa_Parser_Init(void);

// ������յ�������
void VofaParser_ProcessData(uint8_t *data, uint16_t len);
void Vofa_Packet_Data_Save(void);

VofaData_t *Vofa_Get_Data(void);
/*---------------------------VOFA���ݽ���--------------------------------*/

#endif /* __VOFA_H__ */














