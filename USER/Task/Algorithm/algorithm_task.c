//
// Created by Áõ¼Î¿¡ on 25-3-21.
//

#include "algorithm_task.h"
#include "cmsis_os.h"

void AlgorithmTask_Entry(void const * argument)
{
    /* USER CODE BEGIN AlgorithmTask_Entry */
    /* Infinite loop */
    for(;;)
    {

        vTaskDelay(1);
    }
    /* USER CODE END AlgorithmTask_Entry */
}