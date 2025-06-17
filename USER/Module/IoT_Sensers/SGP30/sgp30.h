//
// Created by 刘嘉俊 on 25-6-5.
//

#ifndef CTRBOARD_H7_ALL_SGP30_H
#define CTRBOARD_H7_ALL_SGP30_H

#include "stm32h7xx_hal.h"

#define SGP30_ADDR          0x58
#define	SGP30_ADDR_WRITE	SGP30_ADDR<<1       //0xb0
#define	SGP30_ADDR_READ		(SGP30_ADDR<<1)+1   //0xb1

typedef struct sgp30_data_st {
    uint16_t co2;
    uint16_t tvoc;
} sgp30_data_t;

typedef enum sgp30_cmd_en {
    /* 初始化空气质量测量 */
    INIT_AIR_QUALITY = 0x2003,

    /* 开始空气质量测量 */
    MEASURE_AIR_QUALITY = 0x2008

} sgp30_cmd_t;

int sgp30_init(void);
sgp30_data_t sgp30_get_data(void);

#endif //CTRBOARD_H7_ALL_SGP30_H
