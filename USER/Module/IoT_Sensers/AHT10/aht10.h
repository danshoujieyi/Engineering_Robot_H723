//
// Created by ���ο� on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_AHT10_H
#define CTRBOARD_H7_ALL_AHT10_H

#include "stm32h7xx_hal.h"

#define AHT10_Write_ADDRESS  0x70 // 7λ��ַ+д = 0x38<<1 = 0x70
#define AHT10_Read_ADDRESS   0x71

#define AHT10_Init_com       0xE1
#define AHT10_SoftReset_com  0xBA
#define AHT10_TrigeMea_com   0xAC

typedef struct {
    float temperature;  // ��λ����C
    float humidity;     // ��λ��%RH
    uint8_t valid;      // �Ƿ��ȡ�ɹ���1 = �ɹ���0 = ʧ��
} aht10_data_t;

void aht10_init(void);
aht10_data_t aht10_get_data(void);

#endif //CTRBOARD_H7_ALL_AHT10_H
