//
// Created by 刘嘉俊 on 25-6-10.
//
/**
 * 板载典型声音4000Hz的无源蜂鸣器，
 * TIM12_CH2(PB15)
 * 无源蜂鸣器已经硬件外部上拉3。3V，请不要在单片机内对PB15进行上拉或下拉配置。尤其不允许下拉配置。
 */

#include "passive_buzzer.h"

static TIM_HandleTypeDef *buzzer_htim = NULL;
static uint32_t buzzer_channel;
static uint32_t timer_clk = 240000000;  // 240 MHz 定时器时钟
static uint32_t buzzer_prescaler = 239; // 默认预分频，让时基为 1MHz
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

    // 默认设置为50%占空比
    __HAL_TIM_SET_COMPARE(buzzer_htim, buzzer_channel, current_period / 2);
}

// 最大音量设置100%，在典型频率4KHz下，prescaler为240-1，volume_percent最大为250-1
void Buzzer_SetVolume(uint8_t volume_percent) {
    if (volume_percent > 100) volume_percent = 100;
    uint32_t pulse = (current_period + 1) * volume_percent / 100;
    __HAL_TIM_SET_COMPARE(buzzer_htim, buzzer_channel, pulse);
}
