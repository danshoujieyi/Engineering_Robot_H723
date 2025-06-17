//
// Created by ���ο� on 25-6-4.
//
/**
 * �������߼��ģ����õ����һ����ϵ�Ф�ػ��������ܣ����иߵ���Ӧ�Ⱥ͵͵İ�����,���� LM358 �Ŵ����Թ������������΢���źŽ��зŴ�
 * ����Ԫ�������� 1% ����Ԫ�������졣Ӧ���������߲����ǣ��������ֱ������˶��豸���ֻ��ƶ��绰�ȡ�
 * ������ѹ�� 2.7-5V
 * ���������� 1mA
 * �����Ƕȣ� 130 ��
 * ��Ʈ�� 0.08%/��
 * ��Ⲩ����Χ�� 240nm~370nm
 * �����ʽ: ADC
 * �ܽ�������3 Pin
 */

#include "s12sd.h"
#include "adc_drv.h"
#include "cmsis_os.h"

// ָ�����õ�ADCʵ����ͨ������������
#define S12SD_ADC_INSTANCE    hadc1
#define S12SD_ADC_CHANNEL     ADC_CHANNEL_11
#define S12SD_SAMPLES         20

// �����ߵȼ�����
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

// ����������ȡUV����
s12sd_data_t s12sd_get_data(void)
{
    s12sd_data_t data;
    data.adc_raw = adc_read_avg(&S12SD_ADC_INSTANCE, S12SD_ADC_CHANNEL, S12SD_SAMPLES);
    data.uv_index = Get_S12SD_Ultraviolet_Intensity(data.adc_raw);
    return data;
}