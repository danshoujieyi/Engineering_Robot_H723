//
// Created by ���ο� on 25-4-21.
//

#include "robot.h"
#include "usart_task.h"

void robot_init(void)
{
    // �ر��ж�,��ֹ�ڳ�ʼ�������з����ж�
    // �벻Ҫ�ڳ�ʼ��������ʹ���жϺ���ʱ������
    // ������,��ֻ����ʹ�� dwt ������ʱ
//    __disable_irq();
    usart_semaphore_init(); // �����ź�����ʼ��
//    OS_task_init(); // ������������
//    MX_FREERTOS_Init(); // ���񴴽���cubemax�˺�������

//    mcn_topic_init(); // ����ע���ʼ��

//    motor_task_init();
//    cmd_task_init();
//    chassis_task_init();
//    trans_task_init();
//    referee_UI_task_init();

    // ��ʼ�����,�����ж�
//    __enable_irq();
}