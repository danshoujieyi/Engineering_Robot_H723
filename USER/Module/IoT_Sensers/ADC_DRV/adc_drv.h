//
// Created by 刘嘉俊 on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_ADC_DRV_H
#define CTRBOARD_H7_ALL_ADC_DRV_H

#include "stm32h7xx_hal.h"
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通用ADC读取函数
 * @param hadc     ADC句柄指针（如 &hadc1）
 * @param channel  ADC通道（如 ADC_CHANNEL_11）
 * @param samples  采样次数（建议 ≥ 1）
 * @return 平均ADC值（uint16_t）
 */
uint16_t adc_read_avg(ADC_HandleTypeDef *hadc, uint32_t channel, uint8_t samples);

#ifdef __cplusplus
}
#endif


#endif //CTRBOARD_H7_ALL_ADC_DRV_H
