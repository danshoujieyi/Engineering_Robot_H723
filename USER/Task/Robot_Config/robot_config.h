/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   �������㷨�����̣߳��������㷨�������������߳��м����������
  ******************************************************************************
  * @attention
  *
  * ��������ѭGPLv3��ԴЭ�飬����ѧϰ����ʹ��
  * δ����ɲ���������ҵ��;
  *
  ******************************************************************************
  */
#ifndef CTRBOARD_H7_ALL_ROBOT_CONFIG_H
#define CTRBOARD_H7_ALL_ROBOT_CONFIG_H
/******************************************
 * 1. ȫ���ܿ��أ���ģ�鼶��
 *****************************************/
#define ENABLE_C_STD_MODULE                  1   // C��׼�⣨<stdio.h>�ȣ�
#define ENABLE_HAL_DRIVER_MODULE             1   // CubeMX���ɵ�HAL��
#define ENABLE_USER_ALGORITHM_MODULE         1   // �û��㷨ģ�飨USER/Algorithm��
#define ENABLE_USER_MODULE_MODULE            1   // �û�����������USER/Module��
#define ENABLE_USER_ONENET_ESP8266_MODULE    1   // ������ģ�飨USER/OneNet_ESP8266��
#define ENABLE_USER_TASK_MODULE              1   // �û�����ģ��


/******************************************
 * 2. ��ģ�鿪�أ����ڴ�ģ������ʱ��Ч��
 *****************************************/

/* ----------------- �û��㷨ģ�飨USER/Algorithm�� ----------------- */
#if ENABLE_USER_ALGORITHM_MODULE
    #define ENABLE_ALGORITHM_CRC             1   // CRC�㷨
    #define ENABLE_ALGORITHM_DELAY           1   // ��ʱ�㷨
    #define ENABLE_ALGORITHM_DWT             1   // DWT��ʱ
    #define ENABLE_ALGORITHM_FIFO            1   // FIFO������
    #define ENABLE_ALGORITHM_FILTER          1   // �����˲���
    #define ENABLE_ALGORITHM_KALMAN          1   // ��ά�������˲�
    #define ENABLE_ALGORITHM_KALMAN_ONE      1   // һά�������˲�
    #define ENABLE_ALGORITHM_MAHONY_AHRS     1   // Mahony��̬����
    #define ENABLE_ALGORITHM_MYTYPE          1   // �Զ�����������
    #define ENABLE_ALGORITHM_PID             1   // PID����
    #define ENABLE_ALGORITHM_QUATERNION_EKF  1   // ��Ԫ��EKF
    #define ENABLE_ALGORITHM_RAMP            1   // б�º���
    #define ENABLE_ALGORITHM_UI_ALGORITHM    1   // UI�㷨��������߼���
#endif


/* ----------------- �û�����������USER/Module�� ----------------- */
#if ENABLE_USER_MODULE_MODULE
    #define ENABLE_MODULE_BMI088             1   // BMI088������
    #define ENABLE_MODULE_BOARD_BUZZER       1   // ��Դ������
    #define ENABLE_MODULE_BOARD_WS2812       1   // WS2812�ʵ�
    #define ENABLE_MODULE_DJ_MOTOR           1   // DJ�������
    #define ENABLE_MODULE_DM_MOTOR_FDCAN     1   // DM�����FDCAN���ߣ�
    #define ENABLE_MODULE_FDCAN              1   // FDCAN��������
    #define ENABLE_MODULE_KEYBOARD           1   // ��������
    #define ENABLE_MODULE_MSG                1   // ��ϢЭ�飨���Զ���ͨ�ţ�
    #define ENABLE_MODULE_PUMP               1   // �ÿ���
    #define ENABLE_MODULE_RC                 1   // ң��������
    #define ENABLE_MODULE_REFEREE            1   // ����ϵͳ�������Э�飩
    #define ENABLE_MODULE_UI                 1   // UI��ʾ����OLED��LCD��
    #define ENABLE_MODULE_VOFA               1   // Vofa+���Թ���
#endif


/* ----------------- ������ģ�飨OneNet_ESP8266 ��ģ�飩 ----------------- */
#if ENABLE_USER_ONENET_ESP8266_MODULE
    #define ENABLE_ONENET_ADC_DRV            1   // ADC����
    #define ENABLE_ONENET_AHT10              1   // AHT10��ʪ�ȴ�����
    #define ENABLE_ONENET_BASE64             1   // BASE64����
    #define ENABLE_ONENET_BHT1750FUI         1   // BHT1750FUI��ǿ������
    #define ENABLE_ONENET_ESP8266_CORE       1   // ESP8266����ͨ��
    #define ENABLE_ONENET_GP2Y1014AU         1   // GP2Y1014AU�۳�������
    #define ENABLE_ONENET_HMAC_SHA1          1   // HMAC_SHA1����
    #define ENABLE_ONENET_MQ3_ALCOHOL        1   // MQ3�ƾ�������
    #define ENABLE_ONENET_MQ7_CO             1   // MQ7һ����̼������
    #define ENABLE_ONENET_MQTT               1   // MQTTЭ��
    #define ENABLE_ONENET_MQTT_SAMPLE        1   // MQTTʾ��
    #define ENABLE_ONENET_OLED               1   // OLED��ʾ
    #define ENABLE_ONENET_CORE               1   // OneNetƽ̨����
    #define ENABLE_ONENET_S12SD              1   // S12SD������
    #define ENABLE_ONENET_SGP30              1   // SGP30���崫����
#endif


/* ----------------- �û�����ģ�飨Task ��ģ�飩 ----------------- */
#if ENABLE_USER_TASK_MODULE
    #define ENABLE_TASK_ALGORITHM            1   // �㷨����
    #define ENABLE_TASK_CHASSIS              1   // ���̿�������
    #define ENABLE_TASK_CMD                  1   // ָ���������
    #define ENABLE_TASK_DJMOTOR              1   // DJmotor�������
    #define ENABLE_TASK_DMMOTOR              1   // DMmotor�������
    #define ENABLE_TASK_INS                  1   // ���Ե�������
    #define ENABLE_TASK_ONENET_ESP8266       1   // ����������
    #define ENABLE_TASK_REFEREE              1   // ����ϵͳ����
    #define ENABLE_TASK_ROBOT_CONFIG         1   // ��������������
    #define ENABLE_TASK_TRANSMISSION         1   // ͨ�Ŵ�������
    #define ENABLE_TASK_USART                1   // USART��������
#endif

#endif //CTRBOARD_H7_ALL_ROBOT_CONFIG_H
