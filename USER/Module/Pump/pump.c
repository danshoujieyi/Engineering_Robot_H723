//
// Created by Áõ¼Î¿¡ on 25-3-13.
//

#include "pump.h"
#include "gpio.h"


extern pump_mode_e pump_mode;

void set_pump(uint8_t state)
{
    if(state) {
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_RESET);
    }
}

void pump_control(key_status_t key) {
    static uint8_t last_state = KEY_RELEASE;

    if(key.state == KEY_PRESS_ONCE &&
       last_state != KEY_PRESS_ONCE) {
        // ÇÐ»»Æø±Ã×´Ì¬
        pump_mode = (pump_mode == PUMP_OPEN) ? PUMP_CLOSE : PUMP_OPEN;
        set_pump(pump_mode);  // Ó²¼þ¿ØÖÆº¯Êý
    }
    last_state = key.state;
}

void set_pump_left(uint8_t state)
{
    if(state) {
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_RESET);
    }
}

void set_pump_right(uint8_t state)
{
    if(state) {
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_RESET);
    }
}

void set_pump_all(uint8_t state)
{
    if(state) {
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_RESET);
    }
}
