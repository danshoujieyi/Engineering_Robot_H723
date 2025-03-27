//
// Created by Áõ¼Î¿¡ on 25-3-13.
//

#ifndef CTRBOARD_H7_ALL_PUMP_H
#define CTRBOARD_H7_ALL_PUMP_H

#include "stm32h7xx_hal.h"
#include "keyboard.h"

typedef enum
{
    PUMP_CLOSE,
    PUMP_OPEN,
    PUMP_INIT,
} pump_mode_e;


void set_pump(uint8_t state);

void pump_control(key_status_t key);

void set_pump_left(uint8_t state);

void set_pump_right(uint8_t state);

void set_pump_all(uint8_t state);

#endif //CTRBOARD_H7_ALL_PUMP_H
