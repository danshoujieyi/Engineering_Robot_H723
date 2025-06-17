//
// Created by 刘嘉俊 on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_MQ7_CO_H
#define CTRBOARD_H7_ALL_MQ7_CO_H

#include "stm32h7xx_hal.h"

typedef struct {
    uint16_t adc_raw;      // 原始 ADC 值
    float voltage;         // 电压值 (V)
    float percentage;      // 占比 %
    float rs;              // 传感器电阻 Rs
    float rs_r0;           // Rs/R0 比值
    float ppm;             // CO 浓度（ppm）
    uint8_t alert;         // 数字告警引脚
} mq7_data_t;
mq7_data_t mq7_get_data(void);

#endif //CTRBOARD_H7_ALL_MQ7_CO_H
