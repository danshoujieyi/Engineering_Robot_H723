//
// Created by ���ο� on 25-3-4.
//

#include "referee_task.h"
#include "cmsis_os.h"
#include "referee_system.h"
#include "drv_dwt.h"
#include "dj_motor.h"


/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
static uint32_t referee_task_dwt = 0;   // ������
static float referee_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float referee_task_delta = 0;    // ����߳�����ʱ��
static float referee_task_start_dt = 0; // ����߳̿�ʼʱ��
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

void RefereeTask_Entry(void const * argument)
{
    /*����ϵͳ��ʼ��*/
    referee_system_init();

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    referee_task_dt = dwt_get_delta(&referee_task_dwt);
    referee_task_start_dt = dwt_get_time_ms();
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    for(;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        referee_task_delta = dwt_get_time_ms() - referee_task_start_dt;
        referee_task_start_dt = dwt_get_time_ms();

        referee_task_dt = dwt_get_delta(&referee_task_dwt);
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

      //  dji_motor_control();

        vTaskDelay(1);
    }
}
