//
// Created by ���ο� on 25-6-1.
//

#include "onenet_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"

//����Э���
//#include "onenet.h"
////�����豸
//#include "esp8266.h"
////Ӳ������
//#include "usart.h"
//#include "i2c.h"
//C��
#include <string.h>

#define ESP8266_ONENET_INFO		"AT+CIPSTART=\"TCP\",\"183.230.40.96\",1883\r\n"

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
static uint32_t onenet_task_dwt = 0;   // ������
static float onenet_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float onenet_task_delta = 0;    // ����߳�����ʱ��
static float onenet_task_start_dt = 0; // ����߳̿�ʼʱ��
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

void OnenetTask_Entry(void const * argument)
{
//    unsigned short timeCount = 0;	//���ͼ������
//    unsigned char *dataPtr = NULL;
//    Hardware_Init();				//��ʼ����ΧӲ��
//    ESP8266_Init();					//��ʼ��ESP8266
//    OneNET_RegisterDevice();
//    UsartPrintf(USART_DEBUG, "Connect MQTTs Server...\r\n");
//    while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
//        DelayXms(500);
//    while(OneNet_DevLink())			//����OneNET
//        DelayXms(500);
//    Beep_Set(BEEP_ON);				//������ʾ����ɹ�
//    DelayXms(250);
//    Beep_Set(BEEP_OFF);

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    onenet_task_dt = dwt_get_delta(&onenet_task_dwt);
    onenet_task_start_dt = dwt_get_time_ms();
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    for(;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        onenet_task_delta = dwt_get_time_ms() - onenet_task_start_dt;
        onenet_task_start_dt = dwt_get_time_ms();

        onenet_task_dt = dwt_get_delta(&onenet_task_dwt);
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

//        if(++timeCount >= 500)									//���ͼ��5s
//        {
//            SHT20_GetValue();
//
//            UsartPrintf(USART_DEBUG, "OneNet_SendData\r\n");
//            OneNet_SendData();									//��������
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