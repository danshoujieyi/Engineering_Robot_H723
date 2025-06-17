//
// Created by 刘嘉俊 on 25-6-5.
//

#include "GP2Y1014AU.h"
#include "adc_drv.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

// 配置参数
#define DUST_ADC_INSTANCE    hadc1
#define DUST_ADC_CHANNEL     ADC_CHANNEL_5
#define DUST_ADC_SAMPLES     20

#define DUST_LED_GPIO_PORT   GPIOD
#define DUST_LED_GPIO_PIN    GPIO_PIN_10

#define ADC_REF_VOLTAGE      3.3f
#define ADC_MAX_VALUE        4095.0f

// 分段数据表（来源于你给出的图表）
static const float voltage_table[]   = {0.95f, 1.45f, 2.00f, 2.65f, 3.25f, 3.60f};
static const float density_table[]   = {0.0f,  0.1f,  0.2f,  0.3f,  0.4f,  0.5f};

// 滤波器
static uint16_t dust_filter(uint16_t input)
{
#define FILTER_BUF_SIZE 10
    static uint16_t buffer[FILTER_BUF_SIZE] = {0};
    static uint32_t sum = 0;
    static uint8_t index = 0;
    static uint8_t filled = 0;

    sum -= buffer[index];
    buffer[index] = input;
    sum += input;

    index = (index + 1) % FILTER_BUF_SIZE;
    if (filled < FILTER_BUF_SIZE) filled++;

    return (uint16_t)(sum / filled);
}

// ADC读取
static uint16_t dust_read_adc(void)
{
    return adc_read_avg(&DUST_ADC_INSTANCE, DUST_ADC_CHANNEL, DUST_ADC_SAMPLES);
}

// 拟合算法
static float interpolate_concentration(float voltage)
{
    const int size = sizeof(voltage_table) / sizeof(voltage_table[0]);
    for (int i = 0; i < size - 1; i++) {
        if (voltage >= voltage_table[i] && voltage <= voltage_table[i + 1]) {
            float v0 = voltage_table[i];
            float v1 = voltage_table[i + 1];
            float c0 = density_table[i];
            float c1 = density_table[i + 1];
            return ((voltage - v0) / (v1 - v0)) * (c1 - c0) + c0;
        }
    }

    // 超出范围处理
    if (voltage < voltage_table[0]) return 0.0f;
    if (voltage > voltage_table[size - 1]) return density_table[size - 1];
    return 0.0f; // fallback
}

// 主函数
dust_data_t dust_get_data(void)
{
    dust_data_t data;

    HAL_GPIO_WritePin(DUST_LED_GPIO_PORT, DUST_LED_GPIO_PIN, GPIO_PIN_RESET);
    vTaskDelay(1);

    uint16_t adc = dust_read_adc();
    vTaskDelay(1);

    HAL_GPIO_WritePin(DUST_LED_GPIO_PORT, DUST_LED_GPIO_PIN, GPIO_PIN_SET);
    vTaskDelay(10);

    data.adc_raw = dust_filter(adc);
    data.voltage = (data.adc_raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

    // 插值浓度计算
    data.concentration_mgm3_interp = interpolate_concentration(data.voltage);
    data.concentration_ugm3_interp = data.concentration_mgm3_interp * 1000.0f;

    return data;
}