//
// Created by 刘嘉俊 on 25-6-4.
//
/**
 * 此紫外线检测模块采用氮化家基材料的肖特基光电二极管，具有高的响应度和低的暗电流,板载 LM358 放大器对光电二极管输出的微弱信号进行放大，
 * 所有元器件采用 1% 精度元器件制造。应用于紫外线测试仪，紫外线手表，户外运动设备，手机移动电话等。
 * 工作电压： 2.7-5V
 * 工作电流： 1mA
 * 测量角度： 130 度
 * 温飘： 0.08%/℃
 * 检测波长范围： 240nm~370nm
 * 输出方式: ADC
 * 管脚数量：3 Pin
 */

#include "s12sd.h"
#include "adc_drv.h"
#include "cmsis_os.h"

// 指定所用的ADC实例、通道、采样次数
#define S12SD_ADC_INSTANCE    hadc1
#define S12SD_ADC_CHANNEL     ADC_CHANNEL_11
#define S12SD_SAMPLES         20

// 紫外线等级计算
static char Get_S12SD_Ultraviolet_Intensity(uint16_t value)
{
    if (value < 227) return 0;
    if (value < 318) return 1;
    if (value < 408) return 2;
    if (value < 503) return 3;
    if (value < 606) return 4;
    if (value < 696) return 5;
    if (value < 795) return 6;
    if (value < 881) return 7;
    if (value < 976) return 8;
    if (value < 1079) return 9;
    if (value < 1170) return 10;
    return 11;
}

// 主函数：获取UV数据
s12sd_data_t s12sd_get_data(void)
{
    s12sd_data_t data;
    data.adc_raw = adc_read_avg(&S12SD_ADC_INSTANCE, S12SD_ADC_CHANNEL, S12SD_SAMPLES);
    data.uv_index = Get_S12SD_Ultraviolet_Intensity(data.adc_raw);
    return data;
}