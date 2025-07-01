//
// Created by 刘嘉俊 on 25-4-28.
//

#ifndef F407VGT6_CARARM_ENCODER_H
#define F407VGT6_CARARM_ENCODER_H

#include "stm32f4xx_hal.h"

#define TIM10_CLK_FREQ       84000000

/* 编码器工作模式 */
typedef enum {
    ENCODER_MODE_TI1,     // 仅TI1计数
    ENCODER_MODE_TI2,     // 仅TI2计数
    ENCODER_MODE_TI12     // TI1+TI2正交编码
} Encoder_Mode_e;

/* 编码器方向定义 */
typedef enum {
    ENCODER_DIR_FORWARD,  // 正方向计数递增
    ENCODER_DIR_REVERSE   // 正方向计数递减
} Encoder_Direction_e;

/* 高频定时器结构体 */
typedef struct {
    TIM_HandleTypeDef *htim;       // 定时器句柄
    uint32_t clk_freq;             // 定时器时钟频率（Hz）
    volatile uint32_t overflow;    // 溢出计数器
    volatile uint16_t last_count;  // 上次计数值
} HighFreqTimer_HandleTypeDef;

/* 编码器配置结构体 */
typedef struct {
    TIM_HandleTypeDef *enc_tim;    // 编码器定时器
    HighFreqTimer_HandleTypeDef *hf_timer; // 高频定时器
    uint32_t pulses_per_rev;       // 每转总脉冲数
    int32_t last_count;            // 上次编码器计数值
    Encoder_Direction_e direction; // 计数方向
} Encoder_HandleTypeDef;

/* 初始化函数 */
void Encoder_Init(Encoder_HandleTypeDef *henc);
float Encoder_GetRPM_MT(Encoder_HandleTypeDef *henc);

/* 高频定时器接口 */
void HFTimer_Init(HighFreqTimer_HandleTypeDef *hft);
uint32_t HFTimer_GetTotalCount(HighFreqTimer_HandleTypeDef *hft);

#endif //F407VGT6_CARARM_ENCODER_H
