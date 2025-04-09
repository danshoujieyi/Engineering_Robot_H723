//
// Created by 刘嘉俊 on 25-3-4.
//

#include "referee_task.h"
#include "FreeRTOS.h"
#include "usart.h"
#include "referee_system.h"
#include "semphr.h"
#include <string.h>
#include "cmd_task.h"
#include "rc_sbus.h"
#include "arm_math.h"

extern sbus_data_t sbus_data_fdb;




/*裁判系统线程入口*/
void RefereeTask_Entry(void const * argument)
{
    /*裁判系统初始化*/
    referee_system_init();
    sbus_data_init();

    for (;;) {

            remote_to_cmd_sbus();


        vTaskDelay(1);
    }
}
