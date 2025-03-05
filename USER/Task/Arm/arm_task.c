//
// Created by Áõ¼Î¿¡ on 25-1-18.
//

#include "arm_task.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"


/* USER CODE END Header_ArmTask_Entry */
void ArmTask_Entry(void const * argument)
{

    /* USER CODE BEGIN ArmTask_Entry */
    /* Infinite loop */
    for(;;)
    {

        vTaskDelay(1);
    }
    /* USER CODE END ArmTask_Entry */
}
