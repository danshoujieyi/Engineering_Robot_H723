/**
    ��΢���MS-DOS��Windows�У�ʹ�á��س�CR('r')���͡�����LF('n')�������ַ���Ϊ���з�;
    Windowsϵͳ���棬ÿ�н�β�� �س�+����(CR+LF)������rn����
    Unixϵͳ�ÿ�н�βֻ�� ����CR������n����
    Macϵͳ�ÿ�н�β�� �س�CR ��'r'��
    ��������ƽʱ��д�ļ��Ļس���Ӧ��ȷ����˵�����س����з���
*/
/**
 * RAWDATAЭ�飺����ʲô����ʾʲô�������κδ���ֱ��printf�����
 * FIREWATERЭ�飺Ҳ��ʹ��printf���������������ͨ��֮���ö��ŷָ����Ի��з�\n��Ϊ���ͱ�־������\r\n����
 * ��Ȼ��ʹ��printf������RAWDATA�Ƿ�ʲô����ʾʲô�����硰123.456�����ο���ֱ�����һģһ�������Է����ݣ�Ҳ���Է��ַ�
 * ����FIREWATERЭ����ר��ָ������ͨ���������������ݣ���Ӣ�Ķ�������ÿһ������ͨ���Ի��з��������ͱ�־��ר�ŷ������ݣ��������ַ���
 * FIREWATERЭ�飺printf("%d,%d,%d,%d\r\n",data1,data2,data3);//��data1��data2��data3������Ӧ��Ҫ���������ݼ���
 * �����ĸ�����ͨ�����ֱ����ĸ�����ͨ��
 * FIREWATERЭ���ԭ������Ȱ�����ͨ��printf�е�sprintf����ת�����ַ�����Ȼ����ͨ��UART���ͣ�����prinfԭ��������ռ����Դ�ࡣ
 */
/**
 * JUSTFLOATЭ�飺ֻ���͸��������ݣ�����ʹ��printf����ַ���Ҳ��������ַ���������ʹ���κ��漰sprintf�ĺ��������ʾ��ǲ��ܴ��ַ���Ϊ��׷��Ч�ʺ��ٶȣ���
 * ����С�˸�ʽ�ĸ��������ݣ�С��ָ��λ��ǰ����λ�ں󣩣�ֱ�ӽ�������ת�����ֽ����飨ʮ�����ƣ����͡�һ��float��������ռ��4���ֽڡ�
    for (int i = 0; i < 6; i++) {
        uint8_t *src = (uint8_t *)&values[i];
        uint8_t *dst = &tx_data->data[i * 4];
        memcpy(dst, src, sizeof(float)); // �Զ�����4���ֽ�
    }
    ���壺�� float �����ĵ�ַǿ��ת��Ϊuint8_t*���ͣ�ʹ���ǿ�������ֽڵط��� float ���ڴ��ʾ���������ĸ��ֽڵ�ÿһ����ַ��
    ����memcpy�������� 4 ���ֽڵ����ݴ�src���Ƶ�tx_data->data�����Ӧ��λ�ã�ÿ�ζ��������ĸ��ֽ����ݣ�����ַʵ�֣���
 * ����С�˸�ʽ�ĸ��������ݣ�
    for (int i = 0; i < 7; i++) {
        uint8_t *byte_ptr = &buffer[i * 4];
        memcpy(&float_values[i], byte_ptr, sizeof(float));
    }
    ���壺
    ��δ���ԭ����ʵ�ֵĹ����ǣ���buffer����ֽ���������ȡ 7 ��С�˸�ʽ�ĸ��������ٰ����Ǵ浽float_values�����С����岽�����£�
    ���� 7 ��ѭ����ÿ��ѭ������һ����������
    ÿ��ѭ��ʱ���ҵ���ǰ��������buffer�����ʼλ��(����ַ����
    ����memcpy�������� 4 ���ֽڵ����ݸ��Ƶ�float_values�����Ӧ��λ�á�
    С�˸�ʽָ�������ݵĵ�λ�ֽڴ�����ڴ�ĵ͵�ַ������λ�ֽڴ�����ڴ�ĸߵ�ַ����
    �ٸ����ӣ���һ�� 32 λ������0x12345678����С�˸�ʽ�£������ڴ��еĴ洢˳����0x78 0x56 0x34 0x12��
    ��ʹ��memcpy���������� 4 ���ֽڵ�����ʱ��ϵͳ�ᰴ��С�˸�ʽ��������Щ�ֽڣ��Ӷ��õ���ȷ�ĸ�������ֵ��
    ����Ϊʲômemcpy�ܹ����ĸ��ֽڵ�����ת����һ������������������������ݸ��Ʒ��͸�AI����͡�
    memcpy���ڸ���ָ���������ֽ����ݵ�Ŀ���ַ�����������ͣ�ֻ����ַ��
 * ��Ҫ���巢�����ݰ�����buffer��Ȼ������ת���ɸ�������ʽ����buffer�У�������
 */
#include "vofa.h"
#include "math.h"
#include "usart_task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * ���͸��������ݵ�VOFA+��λ����ʹ��JUSTFLOATЭ�飩
 * @param data �����͵ĸ���������
 * @param count �����͵��������������֧��20�����ܻ�������С���ƣ�
 */
// JUSTFLOATЭ�飺ֻ���͸��������ݡ�ר�ú���
uint8_t Vofa_Send_Float(const float* data, uint8_t channel)
{
    // ��������Ч��
    if (data == NULL || channel == 0 || channel > 20) {
        return -1; // ��Ч����
    }

    // ���㻺������С��ÿ��floatռ4�ֽڣ��ټ�4�ֽڽ�������
    const uint8_t bufferSize = channel * 4 + 4;
    uint8_t tempData[84];   // ���仺���������16*4+4=68�ֽ�

    // ���Ƹ��������ݵ����ͻ�����
    memcpy(tempData, (uint8_t *)data, (sizeof(float)*channel));

    // д��JUSTFLOATЭ��Ľ�β��ʶ
    tempData[bufferSize - 4] = 0x00;
    tempData[bufferSize - 3] = 0x00;
    tempData[bufferSize - 2] = 0x80;
    tempData[bufferSize - 1] = 0x7f;

    // ʹ��DMA�������ݱ����ϸ�ȴ�DMA������ɣ�����ᵼ�����ݶ�ʧ���ص���
//    if (isTransmitting == 0) {
//        isTransmitting = 1;
//        vofa_transmit(tempData, bufferSize);
//    }

    vofa_transmit(tempData, bufferSize);

    return 0; // ���ͳɹ�
}

/* ������ɻص� */
// DMA������ɻص�����
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//    // ����Ƿ���USART1��DMA�������
//    if (huart->Instance == USART1) {
//        // �ͷ��ź�������ʾDMA���Դ�����һ��������
//        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//        isTransmitting = false; // ���÷���״̬Ϊ�Ǵ�����
//
//        xSemaphoreGiveFromISR(xSemaphoreUART1_TX, &xHigherPriorityTaskWoken);
//        // �����Ҫ���Ѹ������ȼ������򴥷��������л�
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }
//}


/*---------------------------VOFA���ݽ���--------------------------------*/
static vofa_unpack_obj_t vofa_unpack_obj = {0};
static VofaData_t vofa_data = {0};

// ��ʼ��������
void Vofa_Parser_Init(void) {
    memset(&vofa_unpack_obj, 0, sizeof(vofa_unpack_obj_t));
    vofa_unpack_obj.unpack_step = STEP_HEADER_1;
    memset(&vofa_data, 0, sizeof(VofaData_t));
}

// ������յ�������
// ������յ�������
void VofaParser_ProcessData(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (vofa_unpack_obj.unpack_step) {
            case STEP_HEADER_1: // �ȴ�֡ͷ#
                if (byte == VOFA_PACKET_HEADER_1) {
                    vofa_unpack_obj.data_index = 0;
                    vofa_unpack_obj.data_ready = false; // �������ݾ�����־
                    vofa_unpack_obj.cmd_type = 0; // ������������
                    vofa_unpack_obj.unpack_step = STEP_HEADER_2;
                }
                break;

            case STEP_HEADER_2: // �ȴ�֡ͷP
                if (byte == VOFA_PACKET_HEADER_2) {
                    vofa_unpack_obj.unpack_step = STEP_CMD_TYPE;
                } else {
                    vofa_unpack_obj.unpack_step = STEP_HEADER_1; // ����״̬��
                }
                break;

            case STEP_CMD_TYPE: // ��¼��������
                vofa_unpack_obj.cmd_type = byte;
                vofa_unpack_obj.unpack_step = STEP_EQUALS;
                break;

            case STEP_EQUALS: // �ȴ��Ⱥ�=
                if (byte == VOFA_PACKET_EQUALS) {
                    vofa_unpack_obj.data_index = 0;
                    vofa_unpack_obj.unpack_step = STEP_DATA_VOFA;
                } else {
                    vofa_unpack_obj.unpack_step = STEP_HEADER_1; // ��ʽ��������
                }
                break;

            case STEP_DATA_VOFA: // ��������
                if (vofa_unpack_obj.data_index < VOFA_DATA_BUFFER_SIZE - 1) {
                    if (byte == VOFA_PACKET_TAIL) {
                        // ����֡β���������ݽ���
                        vofa_unpack_obj.data_buffer[vofa_unpack_obj.data_index] = '\0';  // �ַ�����ĩβ�Զ���'\0'������atof����ʹ��Ҫ��
                        vofa_unpack_obj.data_ready = true;
                        vofa_unpack_obj.unpack_step = STEP_TAIL;
                    } else {
                        // �洢�����ֽ�
                        vofa_unpack_obj.data_buffer[vofa_unpack_obj.data_index++] = byte;
                    }
                } else {
                    vofa_unpack_obj.unpack_step = STEP_HEADER_1; // ���������������
                }
                break;

            case STEP_TAIL: // ��֤֡β

                // ��֤�ɹ�����������
                Vofa_Packet_Data_Save();

                vofa_unpack_obj.unpack_step = STEP_HEADER_1;
                vofa_unpack_obj.data_ready = false;
                break;

            default:
                vofa_unpack_obj.unpack_step = STEP_HEADER_1; // δ֪״̬������
                break;
        }
    }
}


// �����������ݰ�
void Vofa_Packet_Data_Save(void) {
    if (!vofa_unpack_obj.data_ready) return;

    // ��ASCII�ַ���ת��Ϊ������
    // atofֱ�Ӵ������'\0'��β���ַ����������Զ����������š�С����
    float value = (float)atof((char*)vofa_unpack_obj.data_buffer);

    // �����������ͱ�������
    switch (vofa_unpack_obj.cmd_type) {
        case '1':
            vofa_data.p1_value = value;
            break;
        case '2':
            vofa_data.p2_value = value;
            break;
        case '3':
            vofa_data.p3_value = value;
            break;
        case '4':
            vofa_data.p4_value = value;
            break;
        case '5':
            vofa_data.p5_value = value;
            break;
        case '6':
            vofa_data.p6_value = value;
            break;
        case '7':
            vofa_data.p7_value = value;
            break;
        case '8':
            vofa_data.p8_value = value;
            break;
        case '9':
            vofa_data.p9_value = value;
            break;
        case 'A':
            vofa_data.p10_value = value;
            break;
        case 'B':
            vofa_data.p11_value = value;
            break;
        case 'C':
            vofa_data.p12_value = value;
            break;
        case 'D':
            vofa_data.p13_value = value;
            break;
        case 'E':
            vofa_data.p14_value = value;
            break;
        case 'F':
            vofa_data.p15_value = value;
        default:
            // δ֪�������ͣ�������
            break;
    }
}

VofaData_t* Vofa_Get_Data(void) {
    return &vofa_data; // ����ָ��VOFA���ݵ�ָ��
}
/**
*  // ��ȡ����ӡ����
    JY61P_data_t *data = JY61P_GetData();
    printf("���ٶ�: X=%.2f Y=%.2f Z=%.2f\n", data->acc[0], data->acc[1], data->acc[2]);
    printf("���ٶ�: X=%.2f Y=%.2f Z=%.2f\n", data->gyro[0], data->gyro[1], data->gyro[2]);
    printf("�Ƕ�: ��ת=%.2f ����=%.2f ƫ��=%.2f\n", data->angle[0], data->angle[1], data->angle[2]);
*/
/*---------------------------VOFA���ݽ���--------------------------------*/
