////
//// Created by 刘嘉俊 on 25-6-4.
////
///**
// * MQ-7 气体传感器所使用的气敏材料是在清洁空气中电导率较低的二氧化锡(SnO)。采
// * 用高低温循环检测方式低温（1.5V 加热）检测一氧化碳，传感器的电导率随空气中一氧化碳气体浓度增加而增大,
// * 高温（5.0V 加热）清洗低温时吸附的气体。使用简单的电路即可将电导率的变化转换为与该气体浓度相对应的输出信号。
// * 工作电压： 3.3V**-**5V
// * 工作电流： 150MA
// * 输出方式: DO 接口为数字量输出 AO 接口为模拟量输出
// * 读取方式： ADC
// * 管脚数量： 4 Pin（2.54mm 间距排针）
// */
//
// TODO：请给MQ7接3.3V电源，因为adc最大参考电压为3.3V。
// TODO：但是最好是使用5V输入，MQ7的精度与温度有关，电压越大温度越高，精度越好。此时需要设计ADC的5V降压转3.3V电路，增加几个电阻即可
#include "cmsis_os.h"
#include "MQ7_CO.h"
#include "math.h"
#include "adc_drv.h"

#define ADC_MAX              4095.0f
#define VCC                  3.3f
#define RL                   10.0f     // 负载电阻 10kΩ
#define R0                   10.0f     // 清洁空气标定 Rs/R0

#define MQ7_ADC_INSTANCE      hadc1
#define MQ7_ADC_CHANNEL       ADC_CHANNEL_12
#define MQ7_SAMPLES           20

#define MQ7_DO_GPIO_PORT      GPIOA
#define MQ7_DO_PIN            GPIO_PIN_1

/**
 * 使用通用驱动读取 MQ7 ADC 电压值
 */
static uint16_t mq7_read_adc(void)
{
    return adc_read_avg(&MQ7_ADC_INSTANCE, MQ7_ADC_CHANNEL, MQ7_SAMPLES);
}

/**
 * 拟合公式估算 CO 浓度（ppm）
 */
static float mq7_estimate_ppm(float rs_r0)
{
    if (rs_r0 <= 0.0f) return 0.0f;
    float ppm = 500.0f * powf(rs_r0, -1.3f); // MQ7 非线性曲线拟合公式
    return (ppm > 500.0f) ? 500.0f : ppm;
}

/**
 * 获取 MQ7 所有数据
 */
mq7_data_t mq7_get_data(void)
{
    mq7_data_t data;

    data.adc_raw = mq7_read_adc();
    data.voltage = (data.adc_raw / ADC_MAX) * VCC;
    data.percentage = (data.voltage / VCC) * 100.0f;

    if (data.voltage <= 0.01f) {
        data.rs = 0.0f;
        data.rs_r0 = 0.0f;
        data.ppm = 0.0f;
    } else {
        data.rs = RL * (VCC - data.voltage) / data.voltage;
        data.rs_r0 = data.rs / R0;
        data.ppm = mq7_estimate_ppm(data.rs_r0);
    }

    data.alert = HAL_GPIO_ReadPin(MQ7_DO_GPIO_PORT, MQ7_DO_PIN);
    return data;
}