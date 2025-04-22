//
// Created by 刘嘉俊 on 25-3-21.
//
#include <stdio.h>
#include "algorithm_task.h"
#include "cmsis_os.h"
#include "KalmanFilterOne.h"
#include "drv_dwt.h"

static mat_type_t filtered_data[NUM_JOINTS];
static float angles[7] = {0}; // 从队列中读取的角度值（度数）

extern QueueHandle_t xKalmanOneQueue;
extern QueueHandle_t xControlQueue; // 队列句柄

/* ------------------------------ 调试监测线程调度 ------------------------------ */
static uint32_t algorithm_task_dwt = 0;   // 毫秒监测
static float algorithm_task_dt = 0;       // 线程实际运行时间dt
static float algorithm_task_delta = 0;    // 监测线程运行时间
static float algorithm_task_start_dt = 0; // 监测线程开始时间
/* ------------------------------ 调试监测线程调度 ------------------------------ */

void AlgorithmTask_Entry(void const * argument)
{
    Init_KalmanFiltersOne(KALMAN_F, KALMAN_H, KALMAN_Q, KALMAN_R);

/* ------------------------------ 调试监测线程调度 ------------------------------ */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
/* ------------------------------ 调试监测线程调度 ------------------------------ */
    for(;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();

        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
/* ------------------------------ 调试监测线程调度 ------------------------------ */

        if (xQueueReceive(xKalmanOneQueue, angles, 0) == pdTRUE) {
            // 对接收的数据进行滤波处理
            KalmanFilterOne_Data(angles, filtered_data);
            xQueueSend(xControlQueue, filtered_data, 0);
        }
        vTaskDelay(1);
    }
    /* USER CODE END AlgorithmTask_Entry */
}