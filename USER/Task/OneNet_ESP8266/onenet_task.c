////
//// Created by 刘嘉俊 on 25-6-1.
////
//
//#include "onenet_task.h"
//#include "cmsis_os.h"
//#include "drv_dwt.h"
//#include "usart_task.h"
//#include "usart.h"
////网络协议层
////#include "onenet.h"
//////网络设备
////#include "esp8266.h"
//////硬件驱动
////#include "usart.h"
////#include "i2c.h"
////C库
//#include <string.h>
//#include <stdio.h>
//
////要发送数据的宏定义，按需修改
//#define ESP8266_SSID       "aaaaa" //wifi密码
//#define ESP8266_PASSWORD   "88888888"//WiFi账号
//#define ESP8266_MQTT_INFO		"AT+MQTTCONN=0,\"mqtts.heclouds.com\",1883,1\r\n"//网站地址，一般不用修改
//#define ESP8266_ONENET_INFO "AT+MQTTUSERCFG=0,1,\"d1\",\"3h7XtWS80P\",0,0,\"\r\n" //ONENET平台设备名称和ID和密钥token
//#define ESP8266_MQTT_PUBLISH "AT+MQTTPUB=0,\"$sys/3h7XtWS80P/d1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"temp\\\":{\\\"value\\\":10.4\\}\\,\\\"LEDState\\\":{\\\"value\\\":true}}}\",0,0\r\n" //发布数据的格式示例
//
//int pub_index=0;//发布索引，因为经测试发布五个变量的时候稳定，虽然可以发布六个，但是不稳定(有空可以试试一次性发布)
//float temp ,humi, shine, CO1, alcohol;//要发布出去的变量（第一组） //温度，湿度，光照强度，一氧化碳浓度，酒精浓度
//float jiaquan ,yiquan, wumai, ziwaixian;//要发布出去的变量（第二组） //甲醛浓度，乙醛浓度，雾霾浓度，紫外线强度
//int leida;//要发布出去的变量(第二组) //雷达附件人数
//
///* ------------------------------ 调试监测线程调度 ------------------------------ */
//
//static uint32_t onenet_task_dwt = 0;   // 毫秒监测
//static float onenet_task_dt = 0;       // 线程实际运行时间dt
//static float onenet_task_delta = 0;    // 监测线程运行时间
//static float onenet_task_start_dt = 0; // 监测线程开始时间
///* ------------------------------ 调试监测线程调度 ------------------------------ */
//
//void OnenetTask_Entry(void const * argument)
//{
//
//       ESP8266_Init();					//初始化ESP8266
//
///* ------------------------------ 调试监测线程调度 ------------------------------ */
//    onenet_task_dt = dwt_get_delta(&onenet_task_dwt);
//    onenet_task_start_dt = dwt_get_time_ms();
///* ------------------------------ 调试监测线程调度 ------------------------------ */
//    for(;;)
//    {
///* ------------------------------ 调试监测线程调度 ------------------------------ */
//        onenet_task_delta = dwt_get_time_ms() - onenet_task_start_dt;
//        onenet_task_start_dt = dwt_get_time_ms();
//
//        onenet_task_dt = dwt_get_delta(&onenet_task_dwt);
///* ------------------------------ 调试监测线程调度 ------------------------------ */
//
//        if(pub_index==0)
//        {
//            char* command1 = generateMqttPublishCommand1(temp,humi,shine,CO1,alcohol);//不断向ONENET平台发送传感器的数据
//            USART7_DebugPrintf(command1);//发布数据
//            pub_index = pub_index ^ 1;//切换发布命令2
//
//        }
//        else if(pub_index==1)
//        {
//            char *command2 = generateMqttPublishCommand2(jiaquan, yiquan, wumai, leida, ziwaixian);//不断向ONENET平台发送传感器的数据
//            USART7_DebugPrintf(command2);//发布数据
//            pub_index = pub_index ^ 1;//切换发布命令1
//        }
//          vTaskDelay(500);
//    }
//    /* USER CODE END AlgorithmTask_Entry */
//}
//
//// ESP8266初始化流程
//void ESP8266_Init(void) {
//    //检测wifi模块是否正常,还可以把乱码剔除
//    USART7_DebugPrintf("AT\r\n");
//    vTaskDelay(1000);
//
//
//    // 2. 设置Station模式（1=Station, 2=SoftAP, 3=共存）
//    USART7_DebugPrintf("AT+CWMODE=1\r\n");
//    vTaskDelay(1000);
//
//
//    // 3. 启用DHCP（Station模式自动获取IP）
//    USART7_DebugPrintf("AT+CWDHCP=1,1\r\n");
//    vTaskDelay(1000);
//
//
//    // 4. 连接WiFi（需替换为实际SSID和密码）
//    char wifi_cmd[128];
//    sprintf(wifi_cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ESP8266_SSID, ESP8266_PASSWORD);
//    USART7_DebugPrintf(wifi_cmd);
//    vTaskDelay(3000);
//
//
//    //配置订阅设备设置
//    HAL_UART_Transmit(&huart7,"AT+MQTTUSERCFG=0,1,\"d1\",\"3h7XtWS80P\",\"version=2018-10-31&res=products%2F3h7XtWS80P%2Fdevices%2Fd1&et=1779725613&method=md5&sign=zmN4kX%2BhS1n1zAGGfRcOiw%3D%3D\",0,0,\"\"\r\n",170,HAL_MAX_DELAY);
//    vTaskDelay(5000);
//
//
//    //连接MQTT
//    USART7_DebugPrintf(ESP8266_MQTT_INFO);
//    vTaskDelay(3000);
//
//       temp=0.0;    humi=0.0;     shine=0.0;    CO1=0.0;        alcohol=0.0;//初始化变量
//       jiaquan=0.0; yiquan=0.0;   wumai=0.0;    ziwaixian=0.0;  leida=0;
//
//}
//
////发布第一组数据
//char* generateMqttPublishCommand1(float temp,float humi,float shine,float CO1,float alcohol) {
//    static char command[512];  // 静态缓冲区存储指令
//    const char* template = "AT+MQTTPUB=0,\"$sys/3h7XtWS80P/d1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"temp\\\":{\\\"value\\\":%.1f\\}\\,\\\"humi\\\":{\\\"value\\\":%.1f}\\,\\\"shine\\\":{\\\"value\\\":%.1f}\\,\\\"CO1\\\":{\\\"value\\\":%.1f}\\,\\\"jj\\\":{\\\"value\\\":%.1f}}}\",0,0\r\n          ";
//    // 格式化字符串
//    snprintf(command, sizeof(command), template,  temp, humi, shine, CO1, alcohol);
//    return command;
//
//}
//
////发布第二组数据
//char* generateMqttPublishCommand2(float jiaquan,float yiquan,float wumai,float leida,float ziwaixian) {
//    static char command[512];  // 静态缓冲区存储指令
//    const char* template = "AT+MQTTPUB=0,\"$sys/3h7XtWS80P/d1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"jq\\\":{\\\"value\\\":%.1f\\}\\,\\\"yq\\\":{\\\"value\\\":%.1f}\\,\\\"wumai\\\":{\\\"value\\\":%.1f}\\,\\\"leida\\\":{\\\"value\\\":%d}\\,\\\"zw\\\":{\\\"value\\\":%.1f}}}\",0,0\r\n              ";
//    // 格式化字符串
//    snprintf(command, sizeof(command), template,  jiaquan, yiquan, wumai, leida, ziwaixian);
//    return command;
//
//}