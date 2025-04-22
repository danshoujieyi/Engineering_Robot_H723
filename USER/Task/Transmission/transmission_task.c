//
// Created by 刘嘉俊 on 25-3-31.
//

#include "transmission_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"

/* ------------------------------ 调试监测线程调度 ------------------------------ */
static uint32_t transmission_task_dwt = 0;   // 毫秒监测
static float transmission_task_dt = 0;       // 线程实际运行时间dt
static float transmission_task_delta = 0;    // 监测线程运行时间
static float transmission_task_start_dt = 0; // 监测线程开始时间
/* ------------------------------ 调试监测线程调度 ------------------------------ */

void TransmissionTask_Entry(void const * argument)
{
/* ------------------------------ 调试监测线程调度 ------------------------------ */
    transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
    transmission_task_start_dt = dwt_get_time_ms();
/* ------------------------------ 调试监测线程调度 ------------------------------ */
    for(;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        transmission_task_delta = dwt_get_time_ms() - transmission_task_start_dt;
        transmission_task_start_dt = dwt_get_time_ms();

        transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
/* ------------------------------ 调试监测线程调度 ------------------------------ */



        vTaskDelay(1);
    }
}