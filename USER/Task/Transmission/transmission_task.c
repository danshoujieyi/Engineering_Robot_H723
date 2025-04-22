//
// Created by ���ο� on 25-3-31.
//

#include "transmission_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
static uint32_t transmission_task_dwt = 0;   // ������
static float transmission_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float transmission_task_delta = 0;    // ����߳�����ʱ��
static float transmission_task_start_dt = 0; // ����߳̿�ʼʱ��
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

void TransmissionTask_Entry(void const * argument)
{
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
    transmission_task_start_dt = dwt_get_time_ms();
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    for(;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        transmission_task_delta = dwt_get_time_ms() - transmission_task_start_dt;
        transmission_task_start_dt = dwt_get_time_ms();

        transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */



        vTaskDelay(1);
    }
}