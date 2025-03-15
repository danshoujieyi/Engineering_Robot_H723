//
// Created by Áõ¼Î¿¡ on 25-3-13.
//

#include "pump.h"
#include "gpio.h"


void set_pump(uint8_t state)
{
    if(state) {
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_RESET);
    }
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
