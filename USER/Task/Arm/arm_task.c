//
// Created by ���ο� on 25-1-18.
//

#include "arm_task.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"
unsigned char DMmotor_init_flag = 0; // ��ʼ����־λ

/* USER CODE END Header_ArmTask_Entry */
void ArmTask_Entry(void const * argument)
{
    dm_motor_enable(&hfdcan3, &motor[Motor1]);
    vTaskDelay(220); // ��ʱ���ȴ�����ȶ�
    pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 0.7f); // ���Ϳ�������
    vTaskDelay(300); // ��ʱ���ȴ�����ȶ�

    for(int i=1;i<6;i++)
    {
        dm_motor_enable(&hfdcan2, &motor[i]);
        vTaskDelay(220); // ��ʱ���ȴ�����ȶ�
        pos_ctrl(&hfdcan2, motor[i].id, 0, 0.7f); // ���Ϳ�������
        vTaskDelay(300); // ��ʱ���ȴ�����ȶ�
    }
    DMmotor_init_flag = 1; // ��ǳ�ʼ�����

    /* USER CODE BEGIN ArmTask_Entry */
    /* Infinite loop */
    for(;;)
    {

        vTaskDelay(1);
    }
    /* USER CODE END ArmTask_Entry */
}
