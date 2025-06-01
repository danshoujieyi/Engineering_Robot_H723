//
// Created by ���ο� on 25-5-18.
//

#include "msg_freertos.h"
#include "usart_task.h"

#define LOG_E USART7_DebugPrintf

// ��̬�ź����洢��
static StaticSemaphore_t semaphore_pool[MAX_TOPIC_COUNT];

static uint8_t idx = 0; // ���ڼ�¼��ǰ�Ѿ�ע��Ļ�������
static topic_t *topic_obj[MAX_TOPIC_COUNT] = {NULL};  // ����ÿ��ע��Ļ���ʵ��ָ�룬ringbuffer Ϊ���������

/**
 * @brief ��黰��ʵ���Ƿ�����
 *
 * @param name ��������
 * @return int 0:�����������Ļ���ʵ��;��0:���������Ļ���ʵ��+1(�����һ��Ԫ������ʱ����0)
 */
static uint8_t check_topic_name(char *name){
    for(int i = 0; i < MAX_TOPIC_COUNT; i++){
        if(topic_obj[i] != NULL){
            if(strcmp(topic_obj[i]->name, name) == 0){   // ����������Ļ���ʵ��
                return i+1;
            }
        }
    }
    return 0;
}

/**
 * @brief ע���Ϊ��Ϣ������
 *
 * @param name �����߷����Ļ�������(����)
 * @param len  ��Ϣ���ͳ���,ͨ��sizeof()��ȡ
 * @return publisher_t* ���ط�����ʵ��
 */
publisher_t *pub_register(char *name, uint8_t len){
    uint8_t check_num = check_topic_name(name);
    publisher_t *pub = (publisher_t *)pvPortMalloc(sizeof(publisher_t));

    if(!check_num){  // ��������������Ļ���ʵ��
        topic_obj[idx] = (topic_t *)pvPortMalloc(sizeof(topic_t));
        memset(topic_obj[idx], 0, sizeof(topic_t));

        if(topic_obj[idx] == NULL){
            LOG_E("malloc failed!");
            vPortFree(pub);
            return NULL;
        }
        topic_obj[idx]->msg = (void *)pvPortMalloc(len);
        if(topic_obj[idx]->msg == NULL){
            LOG_E("malloc failed!");
            vPortFree(topic_obj[idx]);
            vPortFree(pub);
            return NULL;
        }
        // ��̬�����ź���
        topic_obj[idx]->sem = xSemaphoreCreateBinaryStatic(&semaphore_pool[idx]);
        // configASSERT(topic_obj[idx]->sem != NULL);
        if(topic_obj[idx]->sem == NULL) {
            LOG_E("sem create failed!");
            vPortFree(topic_obj[idx]->msg);
            vPortFree(topic_obj[idx]);
            vPortFree(pub);
            return NULL;
        }
        xSemaphoreGive(topic_obj[idx]->sem); // ��ʼ���ź���

        strcpy(topic_obj[idx]->name, name);
        pub->tp = topic_obj[idx];
        pub->topic_name = topic_obj[idx]->name;
        pub->len = len;
        idx++;
    }
    else{
        pub->tp = topic_obj[check_num - 1];
        pub->topic_name = topic_obj[check_num - 1]->name;
        pub->len = len;
    }

    return pub;
}

/**
 * @brief ����name�Ļ�����Ϣ
 *
 * @param name ��������
 * @param data ��Ϣ����,ͨ��sizeof()��ȡ
 * @return subscriber_t* ���ض�����ʵ��
 */
subscriber_t *sub_register(char *name, uint8_t len){
    // �ͷ�����ͬ�������̣�ֱ�ӵ��÷����ߵ�ע�ắ��
    subscriber_t *sub = (subscriber_t *)pub_register(name, len);

    return sub;
}

/**
 * @brief ������Ϣ
 *
 * @param pub ������ʵ��ָ��
 * @param data ����ָ��,��Ҫ��������Ϣ�ŵ��˴�
 * @return uint8_t ����ֵΪ0˵������ʧ��,Ϊ1˵�������ɹ�
 */
uint8_t pub_push_msg(publisher_t *pub, void *data){
    /* ���жϱ�������ѡ�� */
//    rt_base_t level;
//    level = rt_hw_interrupt_disable();

    if(xSemaphoreTake(pub->tp->sem, 0) == pdFALSE){
//        LOG_W("take sem failed!");
//        rt_hw_interrupt_enable(level);
        return 0;
    }
    memcpy(pub->tp->msg, data, pub->len);
    xSemaphoreGive(pub->tp->sem);

//    rt_hw_interrupt_enable(level);
    return 1;
}

/**
 * @brief ��ȡ��Ϣ
 *
 * @param sub ������ʵ��ָ��
 * @param data ����ָ��,���յ���Ϣ����ŵ��˴�
 * @return uint8_t ����ֵΪ1˵����ȡ�����µ���Ϣ
 */
uint8_t sub_get_msg(subscriber_t *sub, void *data){
    /* ���жϱ�������ѡ�� */
//    rt_base_t level;
//    level = rt_hw_interrupt_disable();

//    if (rt_sem_take(sub->tp->sem, RT_WAITING_NO)) {
//        LOG_W("take sem failed!");
//            rt_hw_interrupt_enable(level);
//        return 0;
//    }
    memcpy(data, sub->tp->msg, sub->len);
//    rt_sem_release(sub->tp->sem);

//    rt_hw_interrupt_enable(level);
    return 1;
}
