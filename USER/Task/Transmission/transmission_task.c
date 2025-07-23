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
#include "transmission_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"
#include "adc.h"
#include "msg_freertos.h"
#include "robot_task.h"


/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
static struct ins_msg transmission_subscribe_ins_data;

static subscriber_t *subscribe_ins_topic;

static void transmission_topic_publish_init(void);
static void transmission_topic_subscribe_init(void);
static void transmission_topic_publish_push(void);
static void transmission_topic_subscribe_pull(void);
/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
/* -------------------------------- ���Լ���߳���� --------------------------------- */
static uint32_t transmission_task_dwt = 0;   // ������
static float transmission_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float transmission_task_delta = 0;    // ����߳�����ʱ��
static float transmission_task_start_dt = 0; // ����߳̿�ʼʱ��
/* -------------------------------- ���Լ���߳���� --------------------------------- */
uint32_t adc_val = 0; // ADC����ֵ����
float vbus;

/* -------------------------------- �߳���� ------------------------------- */
void TransmissionTask_Entry(void const * argument)
{
/* -------------------------------- �����ʼ������ ------------------------------- */
//    WS2812_SetBrightness(10);
//    WS2812_SetRGB(COLOR_RED);
//    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
//    HAL_ADC_Start_DMA(&hadc1, &adc_val,1);
//    // ����LCD����
//    LCD_Init();//LCD��ʼ��
//    LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
/* -------------------------------- �����ʼ������ ------------------------------- */

/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
//    chassis_pub_init();
    transmission_topic_subscribe_init();
/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
    transmission_task_start_dt = dwt_get_time_ms();
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    for(;;)
    {
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
        transmission_task_delta = dwt_get_time_ms() - transmission_task_start_dt;
        transmission_task_start_dt = dwt_get_time_ms();
        transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */
        transmission_topic_subscribe_pull();
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */

/* -------------------------------- �̴߳����д���� ------------------------------- */
//        WS2812_Show(); // ��ʾ���õ���ɫ
//        vbus = (adc_val*3.3f/65535)*11.0f;
//        LCD_ShowString(120, 72,(uint8_t *)"dmBot", BRRED, BLACK, 24, 0);
//        LCD_ShowChinese(84, 100, (uint8_t *)"����Ƽ�", WHITE, BLACK, 32, 0);
//        LCD_DrawLine(10, 0, 10,  280,WHITE);
//        LCD_DrawLine(270,0, 270, 280,WHITE);
//        LCD_ShowIntNum(50, 170, adc_val, 5, WHITE, BLACK, 32);
//        LCD_ShowPicture(180, 150, 80, 80, gImage_1);
/* -------------------------------- �̴߳����д���� ------------------------------- */

/* -------------------------------- �̷߳���Topics��Ϣ ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- �̷߳���Topics��Ϣ ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- �߳̽��� ------------------------------- */

/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
static void transmission_topic_publish_init(void)
{

}

static void transmission_topic_subscribe_init(void)
{
    subscribe_ins_topic = sub_register("ins_pub", sizeof(struct ins_msg));
}

static void transmission_topic_publish_push(void)
{

}

static void transmission_topic_subscribe_pull(void)
{
    sub_get_msg(subscribe_ins_topic, &transmission_subscribe_ins_data);
}
/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */