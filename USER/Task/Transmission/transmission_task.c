/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */
#include "transmission_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"
#include "adc.h"
#include "msg_freertos.h"
#include "robot_task.h"


/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static struct ins_msg transmission_subscribe_ins_data;

static subscriber_t *subscribe_ins_topic;

static void transmission_topic_publish_init(void);
static void transmission_topic_subscribe_init(void);
static void transmission_topic_publish_push(void);
static void transmission_topic_subscribe_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t transmission_task_dwt = 0;   // 毫秒监测
static float transmission_task_dt = 0;       // 线程实际运行时间dt
static float transmission_task_delta = 0;    // 监测线程运行时间
static float transmission_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */
uint32_t adc_val = 0; // ADC采样值数组
float vbus;

/* -------------------------------- 线程入口 ------------------------------- */
void TransmissionTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
//    WS2812_SetBrightness(10);
//    WS2812_SetRGB(COLOR_RED);
//    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
//    HAL_ADC_Start_DMA(&hadc1, &adc_val,1);
//    // 开启LCD背光
//    LCD_Init();//LCD初始化
//    LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
    transmission_topic_subscribe_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
    transmission_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        transmission_task_delta = dwt_get_time_ms() - transmission_task_start_dt;
        transmission_task_start_dt = dwt_get_time_ms();
        transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
        transmission_topic_subscribe_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
//        WS2812_Show(); // 显示设置的颜色
//        vbus = (adc_val*3.3f/65535)*11.0f;
//        LCD_ShowString(120, 72,(uint8_t *)"dmBot", BRRED, BLACK, 24, 0);
//        LCD_ShowChinese(84, 100, (uint8_t *)"达妙科技", WHITE, BLACK, 32, 0);
//        LCD_DrawLine(10, 0, 10,  280,WHITE);
//        LCD_DrawLine(270,0, 270, 280,WHITE);
//        LCD_ShowIntNum(50, 170, adc_val, 5, WHITE, BLACK, 32);
//        LCD_ShowPicture(180, 150, 80, 80, gImage_1);
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
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
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */