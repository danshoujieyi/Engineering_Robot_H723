//
// Created by 刘嘉俊 on 25-6-1.
//

#include "onenet_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"

//网络协议层
//#include "onenet.h"
////网络设备
//#include "esp8266.h"
////硬件驱动
//#include "usart.h"
//#include "i2c.h"
//C库
#include <string.h>

#define ESP8266_ONENET_INFO		"AT+CIPSTART=\"TCP\",\"183.230.40.96\",1883\r\n"

/* ------------------------------ 调试监测线程调度 ------------------------------ */
static uint32_t onenet_task_dwt = 0;   // 毫秒监测
static float onenet_task_dt = 0;       // 线程实际运行时间dt
static float onenet_task_delta = 0;    // 监测线程运行时间
static float onenet_task_start_dt = 0; // 监测线程开始时间
/* ------------------------------ 调试监测线程调度 ------------------------------ */

void OnenetTask_Entry(void const * argument)
{
//    unsigned short timeCount = 0;	//发送间隔变量
//    unsigned char *dataPtr = NULL;
//    Hardware_Init();				//初始化外围硬件
//    ESP8266_Init();					//初始化ESP8266
//    OneNET_RegisterDevice();
//    UsartPrintf(USART_DEBUG, "Connect MQTTs Server...\r\n");
//    while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
//        DelayXms(500);
//    while(OneNet_DevLink())			//接入OneNET
//        DelayXms(500);
//    Beep_Set(BEEP_ON);				//鸣叫提示接入成功
//    DelayXms(250);
//    Beep_Set(BEEP_OFF);

/* ------------------------------ 调试监测线程调度 ------------------------------ */
    onenet_task_dt = dwt_get_delta(&onenet_task_dwt);
    onenet_task_start_dt = dwt_get_time_ms();
/* ------------------------------ 调试监测线程调度 ------------------------------ */
    for(;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        onenet_task_delta = dwt_get_time_ms() - onenet_task_start_dt;
        onenet_task_start_dt = dwt_get_time_ms();

        onenet_task_dt = dwt_get_delta(&onenet_task_dwt);
/* ------------------------------ 调试监测线程调度 ------------------------------ */

//        if(++timeCount >= 500)									//发送间隔5s
//        {
//            SHT20_GetValue();
//
//            UsartPrintf(USART_DEBUG, "OneNet_SendData\r\n");
//            OneNet_SendData();									//发送数据
//
//            timeCount = 0;
//            ESP8266_Clear();
//        }
//
//        dataPtr = ESP8266_GetIPD(0);
//        if(dataPtr != NULL)
//            OneNet_RevPro(dataPtr);

        vTaskDelay(1000);
    }
    /* USER CODE END AlgorithmTask_Entry */
}