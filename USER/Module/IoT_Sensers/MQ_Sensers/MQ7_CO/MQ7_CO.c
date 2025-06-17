////
//// Created by ���ο� on 25-6-4.
////
///**
// * MQ-7 ���崫������ʹ�õ��������������������е絼�ʽϵ͵Ķ�������(SnO)����
// * �øߵ���ѭ����ⷽʽ���£�1.5V ���ȣ����һ����̼���������ĵ絼���������һ����̼����Ũ�����Ӷ�����,
// * ���£�5.0V ���ȣ���ϴ����ʱ���������塣ʹ�ü򵥵ĵ�·���ɽ��絼�ʵı仯ת��Ϊ�������Ũ�����Ӧ������źš�
// * ������ѹ�� 3.3V**-**5V
// * ���������� 150MA
// * �����ʽ: DO �ӿ�Ϊ��������� AO �ӿ�Ϊģ�������
// * ��ȡ��ʽ�� ADC
// * �ܽ������� 4 Pin��2.54mm ������룩
// */
//
// TODO�����MQ7��3.3V��Դ����Ϊadc���ο���ѹΪ3.3V��
// TODO�����������ʹ��5V���룬MQ7�ľ������¶��йأ���ѹԽ���¶�Խ�ߣ�����Խ�á���ʱ��Ҫ���ADC��5V��ѹת3.3V��·�����Ӽ������輴��
#include "cmsis_os.h"
#include "MQ7_CO.h"
#include "math.h"
#include "adc_drv.h"

#define ADC_MAX              4095.0f
#define VCC                  3.3f
#define RL                   10.0f     // ���ص��� 10k��
#define R0                   10.0f     // �������궨 Rs/R0

#define MQ7_ADC_INSTANCE      hadc1
#define MQ7_ADC_CHANNEL       ADC_CHANNEL_12
#define MQ7_SAMPLES           20

#define MQ7_DO_GPIO_PORT      GPIOA
#define MQ7_DO_PIN            GPIO_PIN_1

/**
 * ʹ��ͨ��������ȡ MQ7 ADC ��ѹֵ
 */
static uint16_t mq7_read_adc(void)
{
    return adc_read_avg(&MQ7_ADC_INSTANCE, MQ7_ADC_CHANNEL, MQ7_SAMPLES);
}

/**
 * ��Ϲ�ʽ���� CO Ũ�ȣ�ppm��
 */
static float mq7_estimate_ppm(float rs_r0)
{
    if (rs_r0 <= 0.0f) return 0.0f;
    float ppm = 500.0f * powf(rs_r0, -1.3f); // MQ7 ������������Ϲ�ʽ
    return (ppm > 500.0f) ? 500.0f : ppm;
}

/**
 * ��ȡ MQ7 ��������
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