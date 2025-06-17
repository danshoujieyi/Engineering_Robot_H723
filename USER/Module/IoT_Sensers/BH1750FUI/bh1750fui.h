//
// Created by 刘嘉俊 on 25-6-5.
//

#ifndef F407VGT6_CARARM_BH1750FUI_H
#define F407VGT6_CARARM_BH1750FUI_H

#include "stm32f4xx_hal.h"

// 修改为你在 CubeMX 中启用的 I2C 句柄
#ifndef BH1750_I2C_HANDLE
#define BH1750_I2C_HANDLE hi2c3
#endif

// BH1750 I2C 地址：0x23 或 0x5C（取决于 ADDR 引脚接 GND 还是 VCC）
#define BH1750_ADDR        (0x23 << 1)  // 左移1位以适配 HAL 库要求
#define BH1750_POWER_ON    0x01
#define BH1750_RESET       0x07
#define BH1750_CONT_H_MODE 0x10  // 连续高分辨率模式

typedef struct {
    float lux;
} bh1750_data_t;

void bh1750_init(void);
uint8_t bh1750_read_lux(bh1750_data_t *data);

#endif //F407VGT6_CARARM_BH1750FUI_H
