//
// Created by Áõ¼Î¿¡ on 25-3-31.
//

#include "transmission_task.h"
#include "cmsis_os.h"

/* USER CODE END Header_TransmissionTask_Entry */
void TransmissionTask_Entry(void const * argument)
{
    /* USER CODE BEGIN TransmissionTask_Entry */
    /* Infinite loop */
    for(;;)
    {
        vTaskDelay(1);
    }
    /* USER CODE END TransmissionTask_Entry */
}