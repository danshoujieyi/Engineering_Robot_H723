//
// Created by ���ο� on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_ADC_DRV_H
#define CTRBOARD_H7_ALL_ADC_DRV_H

#include "stm32h7xx_hal.h"
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ͨ��ADC��ȡ����
 * @param hadc     ADC���ָ�루�� &hadc1��
 * @param channel  ADCͨ������ ADC_CHANNEL_11��
 * @param samples  �������������� �� 1��
 * @return ƽ��ADCֵ��uint16_t��
 */
uint16_t adc_read_avg(ADC_HandleTypeDef *hadc, uint32_t channel, uint8_t samples);

#ifdef __cplusplus
}
#endif


#endif //CTRBOARD_H7_ALL_ADC_DRV_H
