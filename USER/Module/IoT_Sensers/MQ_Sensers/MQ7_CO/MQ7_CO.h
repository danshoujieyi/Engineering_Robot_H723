//
// Created by ���ο� on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_MQ7_CO_H
#define CTRBOARD_H7_ALL_MQ7_CO_H

#include "stm32h7xx_hal.h"

typedef struct {
    uint16_t adc_raw;      // ԭʼ ADC ֵ
    float voltage;         // ��ѹֵ (V)
    float percentage;      // ռ�� %
    float rs;              // ���������� Rs
    float rs_r0;           // Rs/R0 ��ֵ
    float ppm;             // CO Ũ�ȣ�ppm��
    uint8_t alert;         // ���ָ澯����
} mq7_data_t;
mq7_data_t mq7_get_data(void);

#endif //CTRBOARD_H7_ALL_MQ7_CO_H
