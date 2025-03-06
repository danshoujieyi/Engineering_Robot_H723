//
// Created by 刘嘉俊 on 25-1-13.
//

#ifndef CTRBOARD_H7_ALL_USART1REC_TASK_H
#define CTRBOARD_H7_ALL_USART1REC_TASK_H
#include "cmsis_os.h"

void USART10_DMA_Init(void);

void USART10_RecEntry(void const *argument);


#endif //CTRBOARD_H7_ALL_USART1REC_TASK_H
