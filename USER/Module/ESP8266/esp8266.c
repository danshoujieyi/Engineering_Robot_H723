///**
//	************************************************************
//	************************************************************
//	************************************************************
//	*	�ļ����� 	esp8266.c
//	*
//	*	���ߣ� 		�ż���
//	*
//	*	���ڣ� 		2017-05-08
//	*
//	*	�汾�� 		V1.0
//	*
//	*	˵���� 		ESP8266�ļ�����
//	*
//	*	�޸ļ�¼��
//	************************************************************
//	************************************************************
//	************************************************************
//**/
//
////��Ƭ��ͷ�ļ�
//#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"
//
////�����豸����
//#include "esp8266.h"
//
//
////C��
//#include <string.h>
//#include <stdio.h>
//#include "usart_task.h"
//
////Ӳ������
//#define Usart_SendString Usart2_SendString
//
//#define ESP8266_WIFI_INFO		"AT+CWJAP=\"ONENET\",\"IOT@Chinamobile123\"\r\n"
//
//
////unsigned char esp8266_buf[512];
////unsigned short esp8266_cnt = 0, esp8266_cntPre = 0;
//
//
////==========================================================
////	�������ƣ�	ESP8266_Clear
////
////	�������ܣ�	��ջ���
////
////	��ڲ�����	��
////
////	���ز�����	��
////
////	˵����
////==========================================================
//void ESP8266_Clear(void)
//{
//
//	memset(esp8266_buf, 0, sizeof(esp8266_buf));
//	esp8266_cnt = 0;
//
//}
//
////==========================================================
////	�������ƣ�	ESP8266_WaitRecive
////
////	�������ܣ�	�ȴ��������
////
////	��ڲ�����	��
////
////	���ز�����	REV_OK-�������		REV_WAIT-���ճ�ʱδ���
////
////	˵����		ѭ�����ü���Ƿ�������
////==========================================================
//_Bool ESP8266_WaitRecive(void)
//{
//
//	if(esp8266_cnt == 0) 							//������ռ���Ϊ0 ��˵��û�д��ڽ��������У�����ֱ����������������
//		return REV_WAIT;
//
//	if(esp8266_cnt == esp8266_cntPre)				//�����һ�ε�ֵ�������ͬ����˵���������
//	{
//		esp8266_cnt = 0;							//��0���ռ���
//
//		return REV_OK;								//���ؽ�����ɱ�־
//	}
//
//	esp8266_cntPre = esp8266_cnt;					//��Ϊ��ͬ
//
//	return REV_WAIT;								//���ؽ���δ��ɱ�־
//
//}
//
////==========================================================
////	�������ƣ�	ESP8266_SendCmd
////
////	�������ܣ�	��������
////
////	��ڲ�����	cmd������
////				res����Ҫ���ķ���ָ��
////
////	���ز�����	0-�ɹ�	1-ʧ��
////
////	˵����
////==========================================================
////_Bool ESP8266_SendCmd(char *cmd, char *res)
////{
////
////	unsigned char timeOut = 200;
////
////	Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
////
////	while(timeOut--)
////	{
////		if(ESP8266_WaitRecive() == REV_OK)							//����յ�����
////		{
////			if(strstr((const char *)esp8266_buf, res) != NULL)		//����������ؼ���
////			{
////				ESP8266_Clear();									//��ջ���
////
////				return 0;
////			}
////		}
////
////		DelayXms(10);
////	}
////
////	return 1;
////
////}
//
//_Bool ESP8266_SendCmd(char *cmd, char *res)
//{
//    uint8_t timeOut = 20;  // ��Ϊѭ������ (20 * 50ms = 1000ms �ܳ�ʱ)
//
//    Usart_SendString((unsigned char *)cmd, strlen((const char *)cmd));
//
//    // ��ս��ջ�����
//    ESP8266_Clear();
//
//    while(timeOut--)
//    {
//        if(ESP8266_WaitRecive() == REV_OK)  // �޸ĺ�����Ϊ ESP8266_WaitReceive
//        {
//            if(strstr((const char *)esp8266_buf, res) != NULL)
//            {
//                ESP8266_Clear();
//                return 0;  // �ɹ�
//            }
//        }
//        // ʹ�� FreeRTOS ��ʱ (������)
//        vTaskDelay(50);  // 50ms �����
//    }
//
//    return 1;  // ��ʱʧ��
//}
//
////==========================================================
////	�������ƣ�	ESP8266_SendData
////
////	�������ܣ�	��������
////
////	��ڲ�����	data������
////				len������
////
////	���ز�����	��
////
////	˵����
////==========================================================
//void ESP8266_SendData(unsigned char *data, unsigned short len)
//{
//
//	char cmdBuf[32];
//
//	ESP8266_Clear();								//��ս��ջ���
//	sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);		//��������
//	if(!ESP8266_SendCmd(cmdBuf, ">"))				//�յ���>��ʱ���Է�������
//	{
//		Usart_SendString(data, len);		//�����豸������������
//	}
//
//}
//
////==========================================================
////	�������ƣ�	ESP8266_GetIPD
////
////	�������ܣ�	��ȡƽ̨���ص�����
////
////	��ڲ�����	�ȴ���ʱ��(����10ms)
////
////	���ز�����	ƽ̨���ص�ԭʼ����
////
////	˵����		��ͬ�����豸���صĸ�ʽ��ͬ����Ҫȥ����
////				��ESP8266�ķ��ظ�ʽΪ	"+IPD,x:yyy"	x�������ݳ��ȣ�yyy����������
////==========================================================
//unsigned char *ESP8266_GetIPD(unsigned short timeOut)
//{
//
//	char *ptrIPD = NULL;
//
//	do
//	{
//		if(ESP8266_WaitRecive() == REV_OK)								//����������
//		{
//			ptrIPD = strstr((char *)esp8266_buf, "IPD,");				//������IPD��ͷ
//			if(ptrIPD == NULL)											//���û�ҵ���������IPDͷ���ӳ٣�������Ҫ�ȴ�һ�ᣬ�����ᳬ���趨��ʱ��
//			{
//				//UsartPrintf(USART_DEBUG, "\"IPD\" not found\r\n");
//			}
//			else
//			{
//				ptrIPD = strchr(ptrIPD, ':');							//�ҵ�':'
//				if(ptrIPD != NULL)
//				{
//					ptrIPD++;
//					return (unsigned char *)(ptrIPD);
//				}
//				else
//					return NULL;
//
//			}
//		}
//
//        vTaskDelay(5);													//��ʱ�ȴ�
//	} while(timeOut--);
//
//	return NULL;														//��ʱ��δ�ҵ������ؿ�ָ��
//
//}
//
////==========================================================
////	�������ƣ�	ESP8266_Init
////
////	�������ܣ�	��ʼ��ESP8266
////
////	��ڲ�����	��
////
////	���ز�����	��
////
////	˵����
////==========================================================
////void ESP8266_Init(void)
////{
////
////	GPIO_InitTypeDef GPIO_Initure;
////
////	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
////
////	//ESP8266��λ����
////	GPIO_Initure.GPIO_Mode = GPIO_Mode_Out_PP;
////	GPIO_Initure.GPIO_Pin = GPIO_Pin_14;					//GPIOC14-��λ
////	GPIO_Initure.GPIO_Speed = GPIO_Speed_50MHz;
////	GPIO_Init(GPIOC, &GPIO_Initure);
////
////	GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_RESET);
////	DelayXms(250);
////	GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_SET);
////	DelayXms(500);
////
////	ESP8266_Clear();
////
////	UsartPrintf(USART_DEBUG, "1. AT\r\n");
////	while(ESP8266_SendCmd("AT\r\n", "OK"))
////		DelayXms(500);
////
////	UsartPrintf(USART_DEBUG, "2. CWMODE\r\n");
////	while(ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK"))
////		DelayXms(500);
////
////	UsartPrintf(USART_DEBUG, "3. AT+CWDHCP\r\n");
////	while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))
////		DelayXms(500);
////
////	UsartPrintf(USART_DEBUG, "4. CWJAP\r\n");
////	while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP"))
////		DelayXms(500);
////
////	UsartPrintf(USART_DEBUG, "5. ESP8266 Init OK\r\n");
////
////}
//
//void ESP8266_Init(void)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    // ���� GPIOC ʱ��
//    __HAL_RCC_GPIOC_CLK_ENABLE();
//
//    // ���� ESP8266 ��λ���� (PC14)
//    GPIO_InitStruct.Pin = GPIO_PIN_14;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//    // ��λ����
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
//    vTaskDelay(100);  // FreeRTOS ��ʱ
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
//    vTaskDelay(100);
//
//    // ��մ��ڻ�����
//    ESP8266_Clear();
//
//    // ���� ESP8266
//    DebugPrintf("1. AT\r\n");
//    ESP8266_SendCmd("AT\r\n", "OK");  // ��ӳ�ʱ����
//    vTaskDelay(100);
//
//    DebugPrintf("2. CWMODE\r\n");
//    ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK");
//    vTaskDelay(100);
//
//    DebugPrintf("3. AT+CWDHCP\r\n");
//    ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK");
//    vTaskDelay(100);
//
//    DebugPrintf("4. CWJAP\r\n");
//    ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP");  // ����WiFi��Ҫ����ʱ��
//    vTaskDelay(200);
//
//    DebugPrintf("5. ESP8266 Init OK\r\n");
//}
//
//
//////==========================================================
//////	�������ƣ�	USART2_IRQHandler
//////
//////	�������ܣ�	����2�շ��ж�
//////
//////	��ڲ�����	��
//////
//////	���ز�����	��
//////
//////	˵����
//////==========================================================
////void USART2_IRQHandler(void)
////{
////
////	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //�����ж�
////	{
////		if(esp8266_cnt >= sizeof(esp8266_buf))	esp8266_cnt = 0; //��ֹ���ڱ�ˢ��
////		esp8266_buf[esp8266_cnt++] = USART2->DR;
////
////		USART_ClearFlag(USART2, USART_FLAG_RXNE);
////	}
////
////}
