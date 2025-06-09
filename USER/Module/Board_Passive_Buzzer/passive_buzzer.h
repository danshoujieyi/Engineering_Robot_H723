//
// Created by Áõ¼Î¿¡ on 25-6-10.
//
#ifndef CTRBOARD_H7_ALL_PASSIVE_BUZZER_H
#define CTRBOARD_H7_ALL_PASSIVE_BUZZER_H

#include "stm32h7xx_hal.h"

void Buzzer_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_SetFrequency(uint32_t freq_hz);
void Buzzer_SetVolume(uint8_t volume_percent);  // 0~100%

#endif //CTRBOARD_H7_ALL_PASSIVE_BUZZER_H
