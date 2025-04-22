//
// Created by ���ο� on 25-3-21.
//
#include <stdio.h>
#include "algorithm_task.h"
#include "cmsis_os.h"
#include "KalmanFilterOne.h"
#include "drv_dwt.h"

static mat_type_t filtered_data[NUM_JOINTS];
static float angles[7] = {0}; // �Ӷ����ж�ȡ�ĽǶ�ֵ��������

extern QueueHandle_t xKalmanOneQueue;
extern QueueHandle_t xControlQueue; // ���о��

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
static uint32_t algorithm_task_dwt = 0;   // ������
static float algorithm_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float algorithm_task_delta = 0;    // ����߳�����ʱ��
static float algorithm_task_start_dt = 0; // ����߳̿�ʼʱ��
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

void AlgorithmTask_Entry(void const * argument)
{
    Init_KalmanFiltersOne(KALMAN_F, KALMAN_H, KALMAN_Q, KALMAN_R);

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    for(;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();

        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

        if (xQueueReceive(xKalmanOneQueue, angles, 0) == pdTRUE) {
            // �Խ��յ����ݽ����˲�����
            KalmanFilterOne_Data(angles, filtered_data);
            xQueueSend(xControlQueue, filtered_data, 0);
        }
        vTaskDelay(1);
    }
    /* USER CODE END AlgorithmTask_Entry */
}