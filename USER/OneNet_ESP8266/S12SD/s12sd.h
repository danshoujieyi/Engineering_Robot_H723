//
// Created by ���ο� on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_S12SD_H
#define CTRBOARD_H7_ALL_S12SD_H

#include "stm32h7xx_hal.h"

typedef struct {
    uint16_t adc_raw;     // ADC ԭʼֵ
    uint8_t uv_index;     // ������ǿ�ȵȼ���0~11��
} s12sd_data_t;

s12sd_data_t s12sd_get_data(void);

#endif //CTRBOARD_H7_ALL_S12SD_H
