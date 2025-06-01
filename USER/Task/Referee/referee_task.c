//
// Created by 刘嘉俊 on 25-3-4.
//

#include "referee_task.h"
#include "cmsis_os.h"
#include "referee_system.h"
#include "drv_dwt.h"
#include "dj_motor.h"


/* ------------------------------ 调试监测线程调度 ------------------------------ */
static uint32_t referee_task_dwt = 0;   // 毫秒监测
static float referee_task_dt = 0;       // 线程实际运行时间dt
static float referee_task_delta = 0;    // 监测线程运行时间
static float referee_task_start_dt = 0; // 监测线程开始时间
/* ------------------------------ 调试监测线程调度 ------------------------------ */

void RefereeTask_Entry(void const * argument)
{
    /*裁判系统初始化*/
    referee_system_init();

/* ------------------------------ 调试监测线程调度 ------------------------------ */
    referee_task_dt = dwt_get_delta(&referee_task_dwt);
    referee_task_start_dt = dwt_get_time_ms();
/* ------------------------------ 调试监测线程调度 ------------------------------ */
    for(;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        referee_task_delta = dwt_get_time_ms() - referee_task_start_dt;
        referee_task_start_dt = dwt_get_time_ms();

        referee_task_dt = dwt_get_delta(&referee_task_dwt);
/* ------------------------------ 调试监测线程调度 ------------------------------ */

      //  dji_motor_control();

        vTaskDelay(1);
    }
}
