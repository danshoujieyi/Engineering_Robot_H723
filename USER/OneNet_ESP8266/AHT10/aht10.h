//
// Created by 刘嘉俊 on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_AHT10_H
#define CTRBOARD_H7_ALL_AHT10_H

#include "stm32h7xx_hal.h"

#define AHT10_Write_ADDRESS  0x70 // 7位地址+写 = 0x38<<1 = 0x70
#define AHT10_Read_ADDRESS   0x71

#define AHT10_Init_com       0xE1
#define AHT10_SoftReset_com  0xBA
#define AHT10_TrigeMea_com   0xAC

typedef struct {
    float temperature;  // 单位：°C
    float humidity;     // 单位：%RH
    uint8_t valid;      // 是否读取成功：1 = 成功，0 = 失败
} aht10_data_t;

void aht10_init(void);
aht10_data_t aht10_get_data(void);

#endif //CTRBOARD_H7_ALL_AHT10_H
