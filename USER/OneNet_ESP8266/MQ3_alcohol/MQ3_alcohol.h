//
// Created by 刘嘉俊 on 25-6-5.
//

#ifndef F407VGT6_CARARM_MQ3_ALCOHOL_H
#define F407VGT6_CARARM_MQ3_ALCOHOL_H

#include "stm32f4xx_hal.h"

#define MQ3_ADC_HANDLE     hadc2          // 使用 ADC2

#define MQ3_ADC_CHANNEL    ADC_CHANNEL_7
#define MQ3_SAMPLES        20

#define ADC_MAX              4095.0f
#define VCC                  3.3f
#define RL                   10.0f     // 负载电阻 (kΩ)
#define R0                   10.0f     // 清洁空气下 Rs 值（校准用）


#define MQ3_DO_GPIO_PORT   GPIOA
#define MQ3_DO_GPIO_PIN    GPIO_PIN_1

typedef struct {
    uint16_t adc_raw;
    float voltage;
    float rs;
    float rs_r0;
    float ppm;
    uint8_t percentage;
    uint8_t alert;
} mq3_data_t;

mq3_data_t mq3_get_data(void);

#endif //F407VGT6_CARARM_MQ3_ALCOHOL_H
