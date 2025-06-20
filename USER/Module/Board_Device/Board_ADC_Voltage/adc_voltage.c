//
// Created by 刘嘉俊 on 25-6-21.
//

#include "adc_voltage.h"
#include "adc.h"

//HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
//HAL_ADC_Start_DMA(&hadc1, &adc_val,1);
//
//uint32_t adc_val = 0; // ADC采样值数组
//float vbus;
//
//vbus = (adc_val*3.3f/65535)*11.0f;
//vbus = (adc_val*3.3f/65535)*11.06f;
//vbus = (adc_val*3.3f/65535)*14.0f;