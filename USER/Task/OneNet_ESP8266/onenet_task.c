////
//// Created by ���ο� on 25-6-1.
////
//
//#include "onenet_task.h"
//#include "cmsis_os.h"
//#include "drv_dwt.h"
//#include "usart_task.h"
//#include "usart.h"
////����Э���
////#include "onenet.h"
//////�����豸
////#include "esp8266.h"
//////Ӳ������
////#include "usart.h"
////#include "i2c.h"
////C��
//#include <string.h>
//#include <stdio.h>
//
////Ҫ�������ݵĺ궨�壬�����޸�
//#define ESP8266_SSID       "aaaaa" //wifi����
//#define ESP8266_PASSWORD   "88888888"//WiFi�˺�
//#define ESP8266_MQTT_INFO		"AT+MQTTCONN=0,\"mqtts.heclouds.com\",1883,1\r\n"//��վ��ַ��һ�㲻���޸�
//#define ESP8266_ONENET_INFO "AT+MQTTUSERCFG=0,1,\"d1\",\"3h7XtWS80P\",0,0,\"\r\n" //ONENETƽ̨�豸���ƺ�ID����Կtoken
//#define ESP8266_MQTT_PUBLISH "AT+MQTTPUB=0,\"$sys/3h7XtWS80P/d1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"temp\\\":{\\\"value\\\":10.4\\}\\,\\\"LEDState\\\":{\\\"value\\\":true}}}\",0,0\r\n" //�������ݵĸ�ʽʾ��
//
//int pub_index=0;//������������Ϊ�����Է������������ʱ���ȶ�����Ȼ���Է������������ǲ��ȶ�(�пտ�������һ���Է���)
//float temp ,humi, shine, CO1, alcohol;//Ҫ������ȥ�ı�������һ�飩 //�¶ȣ�ʪ�ȣ�����ǿ�ȣ�һ����̼Ũ�ȣ��ƾ�Ũ��
//float jiaquan ,yiquan, wumai, ziwaixian;//Ҫ������ȥ�ı������ڶ��飩 //��ȩŨ�ȣ���ȩŨ�ȣ�����Ũ�ȣ�������ǿ��
//int leida;//Ҫ������ȥ�ı���(�ڶ���) //�״︽������
//
///* ------------------------------ ���Լ���̵߳��� ------------------------------ */
//
//static uint32_t onenet_task_dwt = 0;   // ������
//static float onenet_task_dt = 0;       // �߳�ʵ������ʱ��dt
//static float onenet_task_delta = 0;    // ����߳�����ʱ��
//static float onenet_task_start_dt = 0; // ����߳̿�ʼʱ��
///* ------------------------------ ���Լ���̵߳��� ------------------------------ */
//
//void OnenetTask_Entry(void const * argument)
//{
//
//       ESP8266_Init();					//��ʼ��ESP8266
//
///* ------------------------------ ���Լ���̵߳��� ------------------------------ */
//    onenet_task_dt = dwt_get_delta(&onenet_task_dwt);
//    onenet_task_start_dt = dwt_get_time_ms();
///* ------------------------------ ���Լ���̵߳��� ------------------------------ */
//    for(;;)
//    {
///* ------------------------------ ���Լ���̵߳��� ------------------------------ */
//        onenet_task_delta = dwt_get_time_ms() - onenet_task_start_dt;
//        onenet_task_start_dt = dwt_get_time_ms();
//
//        onenet_task_dt = dwt_get_delta(&onenet_task_dwt);
///* ------------------------------ ���Լ���̵߳��� ------------------------------ */
//
//        if(pub_index==0)
//        {
//            char* command1 = generateMqttPublishCommand1(temp,humi,shine,CO1,alcohol);//������ONENETƽ̨���ʹ�����������
//            USART7_DebugPrintf(command1);//��������
//            pub_index = pub_index ^ 1;//�л���������2
//
//        }
//        else if(pub_index==1)
//        {
//            char *command2 = generateMqttPublishCommand2(jiaquan, yiquan, wumai, leida, ziwaixian);//������ONENETƽ̨���ʹ�����������
//            USART7_DebugPrintf(command2);//��������
//            pub_index = pub_index ^ 1;//�л���������1
//        }
//          vTaskDelay(500);
//    }
//    /* USER CODE END AlgorithmTask_Entry */
//}
//
//// ESP8266��ʼ������
//void ESP8266_Init(void) {
//    //���wifiģ���Ƿ�����,�����԰������޳�
//    USART7_DebugPrintf("AT\r\n");
//    vTaskDelay(1000);
//
//
//    // 2. ����Stationģʽ��1=Station, 2=SoftAP, 3=���棩
//    USART7_DebugPrintf("AT+CWMODE=1\r\n");
//    vTaskDelay(1000);
//
//
//    // 3. ����DHCP��Stationģʽ�Զ���ȡIP��
//    USART7_DebugPrintf("AT+CWDHCP=1,1\r\n");
//    vTaskDelay(1000);
//
//
//    // 4. ����WiFi�����滻Ϊʵ��SSID�����룩
//    char wifi_cmd[128];
//    sprintf(wifi_cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ESP8266_SSID, ESP8266_PASSWORD);
//    USART7_DebugPrintf(wifi_cmd);
//    vTaskDelay(3000);
//
//
//    //���ö����豸����
//    HAL_UART_Transmit(&huart7,"AT+MQTTUSERCFG=0,1,\"d1\",\"3h7XtWS80P\",\"version=2018-10-31&res=products%2F3h7XtWS80P%2Fdevices%2Fd1&et=1779725613&method=md5&sign=zmN4kX%2BhS1n1zAGGfRcOiw%3D%3D\",0,0,\"\"\r\n",170,HAL_MAX_DELAY);
//    vTaskDelay(5000);
//
//
//    //����MQTT
//    USART7_DebugPrintf(ESP8266_MQTT_INFO);
//    vTaskDelay(3000);
//
//       temp=0.0;    humi=0.0;     shine=0.0;    CO1=0.0;        alcohol=0.0;//��ʼ������
//       jiaquan=0.0; yiquan=0.0;   wumai=0.0;    ziwaixian=0.0;  leida=0;
//
//}
//
////������һ������
//char* generateMqttPublishCommand1(float temp,float humi,float shine,float CO1,float alcohol) {
//    static char command[512];  // ��̬�������洢ָ��
//    const char* template = "AT+MQTTPUB=0,\"$sys/3h7XtWS80P/d1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"temp\\\":{\\\"value\\\":%.1f\\}\\,\\\"humi\\\":{\\\"value\\\":%.1f}\\,\\\"shine\\\":{\\\"value\\\":%.1f}\\,\\\"CO1\\\":{\\\"value\\\":%.1f}\\,\\\"jj\\\":{\\\"value\\\":%.1f}}}\",0,0\r\n          ";
//    // ��ʽ���ַ���
//    snprintf(command, sizeof(command), template,  temp, humi, shine, CO1, alcohol);
//    return command;
//
//}
//
////�����ڶ�������
//char* generateMqttPublishCommand2(float jiaquan,float yiquan,float wumai,float leida,float ziwaixian) {
//    static char command[512];  // ��̬�������洢ָ��
//    const char* template = "AT+MQTTPUB=0,\"$sys/3h7XtWS80P/d1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"jq\\\":{\\\"value\\\":%.1f\\}\\,\\\"yq\\\":{\\\"value\\\":%.1f}\\,\\\"wumai\\\":{\\\"value\\\":%.1f}\\,\\\"leida\\\":{\\\"value\\\":%d}\\,\\\"zw\\\":{\\\"value\\\":%.1f}}}\",0,0\r\n              ";
//    // ��ʽ���ַ���
//    snprintf(command, sizeof(command), template,  jiaquan, yiquan, wumai, leida, ziwaixian);
//    return command;
//
//}