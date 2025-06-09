//
// Created by ���ο� on 25-6-10.
//
/**
 * ���ص�������4000Hz����Դ��������
 * TIM12_CH2(PB15)
 * ��Դ�������Ѿ�Ӳ���ⲿ����3��3V���벻Ҫ�ڵ�Ƭ���ڶ�PB15�����������������á����䲻�����������á�
 */

#include "passive_buzzer.h"

static TIM_HandleTypeDef *buzzer_htim = NULL;
static uint32_t buzzer_channel;
static uint32_t timer_clk = 240000000;  // 240 MHz ��ʱ��ʱ��
static uint32_t buzzer_prescaler = 239; // Ĭ��Ԥ��Ƶ����ʱ��Ϊ 1MHz
static uint32_t current_period = 249;

void Buzzer_Init(TIM_HandleTypeDef *htim, uint32_t channel) {
    buzzer_htim = htim;
    buzzer_channel = channel;

    __HAL_TIM_SET_PRESCALER(buzzer_htim, buzzer_prescaler);
    __HAL_TIM_SET_AUTORELOAD(buzzer_htim, current_period);
    __HAL_TIM_SET_COMPARE(buzzer_htim, buzzer_channel, current_period / 2);

    HAL_TIM_PWM_Start(buzzer_htim, buzzer_channel);
}

void Buzzer_On(void) {
    HAL_TIM_PWM_Start(buzzer_htim, buzzer_channel);
}

void Buzzer_Off(void) {
    HAL_TIM_PWM_Stop(buzzer_htim, buzzer_channel);
}

void Buzzer_SetFrequency(uint32_t freq_hz) {
    current_period = (timer_clk / ((buzzer_prescaler + 1) * freq_hz)) - 1;
    __HAL_TIM_SET_AUTORELOAD(buzzer_htim, current_period);

    // Ĭ������Ϊ50%ռ�ձ�
    __HAL_TIM_SET_COMPARE(buzzer_htim, buzzer_channel, current_period / 2);
}

// �����������100%���ڵ���Ƶ��4KHz�£�prescalerΪ240-1��volume_percent���Ϊ250-1
void Buzzer_SetVolume(uint8_t volume_percent) {
    if (volume_percent > 100) volume_percent = 100;
    uint32_t pulse = (current_period + 1) * volume_percent / 100;
    __HAL_TIM_SET_COMPARE(buzzer_htim, buzzer_channel, pulse);
}
