//
// Created by 刘嘉俊 on 25-6-5.
//
/**
 * MQ-3 气体传感器所使用的气敏材料是在清洁空气中电导率较低的二氧化锡(Sn0)。
 * 当传感器所处环境中存在酒精蒸气时，传感器的电导率随空气中酒精蒸气浓度的增加而增大。
 * 使用简单的电路即可将电导率的变化转换为与该气体浓度相对应的输出信号。
 * 工作电压： 3.3V-5V
 * 工作电流： 150MA
 * 输出方式: DO 接口为数字量输出 AO 接口为模拟量输出
 * 读取方式： ADC
 * 管脚数量： 4 Pin（2.54mm 间距排针）
 */
// TODO：请给MQ3接3.3V电源，因为adc最大参考电压为3.3V。
// TODO：但是最好是使用5V输入，MQ3的精度与温度有关，电压越大温度越高，精度越好。此时需要设计ADC的5V降压转3.3V电路，增加几个电阻即可
#include <math.h>
#include "MQ3_alcohol.h"
#include "adc_drv.h"

static uint16_t mq3_read_adc(void)
{
    return adc_read_avg(&MQ3_ADC_HANDLE, MQ3_ADC_CHANNEL, MQ3_SAMPLES);
}

/**
 * @brief 使用对数拟合估算酒精浓度（ppm）
 */
static float mq3_estimate_ppm(float rs_r0)
{
    if (rs_r0 <= 0.0f) return 0.0f;

    float log_rs_r0 = log10f(rs_r0);
    float log_ppm = -1.497f * log_rs_r0 + 1.52f;
    float ppm = powf(10.0f, log_ppm);

    return (ppm > 1000.0f) ? 1000.0f : ppm;
}

mq3_data_t mq3_get_data(void)
{
    mq3_data_t data;

    data.adc_raw = mq3_read_adc();
    data.voltage = (data.adc_raw / ADC_MAX) * VCC;
    data.percentage = (uint8_t)(data.voltage / VCC * 100.0f + 0.5f);

    if (data.voltage <= 0.01f) {
        data.rs = 0.0f;
        data.rs_r0 = 0.0f;
        data.ppm = 0.0f;
    } else {
        data.rs = RL * (VCC - data.voltage) / data.voltage;
        data.rs_r0 = data.rs / R0;
        data.ppm = mq3_estimate_ppm(data.rs_r0);
    }

    data.alert = HAL_GPIO_ReadPin(MQ3_DO_GPIO_PORT, MQ3_DO_GPIO_PIN);
    return data;
}