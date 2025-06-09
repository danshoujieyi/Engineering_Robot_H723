//
// Created by 刘嘉俊 on 25-6-5.
//

#ifndef F407VGT6_CARARM_GP2Y1014AU_H
#define F407VGT6_CARARM_GP2Y1014AU_H


#include "stdint.h"

typedef struct {
    uint16_t adc_raw;
    float voltage;
    float concentration_mgm3_interp;  // 拟合浓度
    float concentration_ugm3_interp;
} dust_data_t;


dust_data_t dust_get_data(void);

#endif //F407VGT6_CARARM_GP2Y1014AU_H
