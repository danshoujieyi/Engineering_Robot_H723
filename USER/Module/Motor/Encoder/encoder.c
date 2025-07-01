//
// Created by 刘嘉俊 on 25-4-28.
//

#include <stdio.h>
#include "encoder.h"
#include "math.h"

extern HighFreqTimer_HandleTypeDef g_hftimer;

/* 高频定时器初始化 */
void HFTimer_Init(HighFreqTimer_HandleTypeDef *hft) {
    hft->overflow = 0;
    hft->last_count = 0;
    HAL_TIM_Base_Start_IT(hft->htim);
}

/* 编码器初始化 */
void Encoder_Init(Encoder_HandleTypeDef *henc) {
    // 配置编码器方向
    if(henc->direction == ENCODER_DIR_REVERSE) {
        henc->enc_tim->Instance->CR1 |= TIM_CR1_DIR;
    }

    // 启动编码器
    HAL_TIM_Encoder_Start(henc->enc_tim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(henc->enc_tim, 0);
    henc->last_count = 0;
}

// 高频定时器操作
uint32_t HFTimer_GetTotalCount(HighFreqTimer_HandleTypeDef *hft) {
    uint16_t cnt;
    uint32_t overflow;

    __disable_irq();
    cnt = __HAL_TIM_GET_COUNTER(hft->htim);
    overflow = hft->overflow;
    if(__HAL_TIM_GET_FLAG(hft->htim, TIM_FLAG_UPDATE)) {
        overflow++;
        cnt = __HAL_TIM_GET_COUNTER(hft->htim);
        __HAL_TIM_CLEAR_FLAG(hft->htim, TIM_FLAG_UPDATE);
    }
    __enable_irq();

    return (overflow << 16) | cnt;
}

// 转速计算
float Encoder_GetRPM_MT(Encoder_HandleTypeDef *henc) {
    static uint32_t last_M1 = 0;
    uint32_t current_M1 = HFTimer_GetTotalCount(henc->hf_timer);

    int32_t M0 = __HAL_TIM_GET_COUNTER(henc->enc_tim) - henc->last_count;
   // printf("M0:%d\n", M0);
    henc->last_count = __HAL_TIM_GET_COUNTER(henc->enc_tim);

    float delta_time = (current_M1 - last_M1) / (float)henc->hf_timer->clk_freq;
   // printf("current_M1:%d, last_M1:%d\n", current_M1, last_M1);

    last_M1 = current_M1;
    float rpm = (fabsf(M0) * 60.0f) / (henc->pulses_per_rev * delta_time);
   // printf("rpm:%f\n", rpm);
   // printf("\n");
    return rpm;
}

// HAL库溢出回调
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == TIM10) {
//        g_hftimer.overflow++;
//    }
//}

//int32_t Get_Encoder_Delta(TIM_HandleTypeDef *htim) {
//    int32_t current_cnt = __HAL_TIM_GET_COUNTER(htim);
//    int32_t delta = current_cnt - last_encoder_cnt;
//    last_encoder_cnt = current_cnt;
//    return delta;
//}
///* 获取累计计数值（带方向） */
//int32_t encoder_get_count(Encoder_HandleTypeDef *henc)
//{
//    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(henc->htim);
//    return (henc->direction == ENCODER_DIR_FORWARD) ? count : -count;
//}
//
///* 清零计数器 */
//void encoder_clear(Encoder_HandleTypeDef *henc)
//{
//    __HAL_TIM_SET_COUNTER(henc->htim, 0);
//}
//
