//
// Created by 刘嘉俊 on 25-1-13.
//

#ifndef CTRBOARD_H7_ALL_USART10REC_TASK_H
#define CTRBOARD_H7_ALL_USART10REC_TASK_H
#include "cmsis_os.h"
#define FRAME_SIZE 28
uint8_t ParseFrame(uint8_t *buffer, float *angles);

#endif //CTRBOARD_H7_ALL_USART10REC_TASK_H
