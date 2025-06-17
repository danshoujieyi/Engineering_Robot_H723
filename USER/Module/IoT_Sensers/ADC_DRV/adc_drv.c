//
// Created by 刘嘉俊 on 25-6-5.
//

#include "adc_drv.h"
#include "cmsis_os.h"  // 若未使用RTOS可替换为 HAL_Delay()

uint16_t adc_read_avg(ADC_HandleTypeDef *hadc, uint32_t channel, uint8_t samples)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
        return 0;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        HAL_ADC_Start(hadc);
        if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
            sum += HAL_ADC_GetValue(hadc);
        }
        HAL_ADC_Stop(hadc);
        vTaskDelay(5);  // 若无RTOS，可替换为 HAL_Delay(5)
    }

    return (uint16_t)(sum / samples);
}
