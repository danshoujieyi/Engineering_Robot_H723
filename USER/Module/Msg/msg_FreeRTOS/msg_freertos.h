//
// Created by ���ο� on 25-5-18.
//

#ifndef CTRBOARD_H7_ALL_MSG_FREERTOS_H
#define CTRBOARD_H7_ALL_MSG_FREERTOS_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "string.h"

#define MAX_TOPIC_COUNT 20    // ���֧�ֵĻ�������
#define MSG_NAME_MAX    12    // ����������󳤶�

/**
 * @brief ��������
 *
 */
typedef struct topic
{
    char name[MSG_NAME_MAX];
    void *msg;              // ָ��msgʵ����ָ��
    SemaphoreHandle_t  sem;           // ��֤������Ϣ��ԭ����
} topic_t;

/**
 * @brief ����������.ÿ��������ӵ�з�����ʵ��,���ҿ���ͨ������������ж������Լ������Ļ���Ķ�����
 *
 */
typedef struct sublisher
{
    const char *topic_name;
    topic_t *tp;               // �����ָ��
    uint8_t len;               // ��Ϣ���ͳ���
   // StaticSemaphore_t sem_buffer; // ��̬�����ź����ڴ�
} subscriber_t;

/**
 * @brief ����������.ÿ��������ӵ�з�����ʵ��,���ҿ���ͨ������������ж������Լ������Ļ���Ķ�����
 *
 */
typedef struct publisher
{
    const char *topic_name;
    topic_t *tp;               // �����ָ��
    uint8_t len;               // ��Ϣ���ͳ���
   // StaticSemaphore_t sem_buffer; // ��̬�����ź����ڴ�
} publisher_t;

/**
 * @brief ����name�Ļ�����Ϣ
 *
 * @param name ��������
 * @param len ��Ϣ����,ͨ��sizeof()��ȡ
 * @return subscriber_t* ���ض�����ʵ��
 */
subscriber_t *sub_register(char *name, uint8_t len);

/**
 * @brief ע���Ϊ��Ϣ������
 *
 * @param name �����߷����Ļ�������(����)
 * @param len  ��Ϣ���ͳ���,ͨ��sizeof()��ȡ
 * @return publisher_t* ���ط�����ʵ��
 */
publisher_t *pub_register(char *name, uint8_t len);

/**
 * @brief ������Ϣ
 *
 * @param pub ������ʵ��ָ��
 * @param data ����ָ��,��Ҫ��������Ϣ�ŵ��˴�
 * @return uint8_t ����ֵΪ0˵������ʧ��,Ϊ1˵�������ɹ�
 */
uint8_t sub_get_msg(subscriber_t *sub, void *data);

/**
 * @brief ��ȡ��Ϣ
 *
 * @param sub ������ʵ��ָ��
 * @param data ����ָ��,���յ���Ϣ����ŵ��˴�
 * @return uint8_t ����ֵΪ1˵����ȡ�����µ���Ϣ
 */
uint8_t pub_push_msg(publisher_t *pub, void *data);



#endif //CTRBOARD_H7_ALL_MSG_FREERTOS_H
