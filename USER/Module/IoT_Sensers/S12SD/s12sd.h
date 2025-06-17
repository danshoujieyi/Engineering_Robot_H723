//
// Created by 刘嘉俊 on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_S12SD_H
#define CTRBOARD_H7_ALL_S12SD_H

#include "stm32h7xx_hal.h"

typedef struct {
    uint16_t adc_raw;     // ADC 原始值
    uint8_t uv_index;     // 紫外线强度等级（0~11）
} s12sd_data_t;

s12sd_data_t s12sd_get_data(void);

#endif //CTRBOARD_H7_ALL_S12SD_H
