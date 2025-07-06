//
// Created by 刘嘉俊 on 25-4-28.
//

#include <stdio.h>
#include "encoder.h"
#include "math.h"
//
// Created by 刘嘉俊 on 25-4-28.
//

#include <stdio.h>
#include "encoder.h"
#include "math.h"

#include "tim_dwt.h"

static Encoder_Speed_t motor_encoder_left = {
        .temp_count = 0,
        .count = 0,
        .direction = ENCODER_DIR_FORWARD,
        .mode = ENCODER_MODE_TI12
};
static Encoder_Speed_t motor_encoder_right = {
        .temp_count = 0,
        .count = 0,
        .direction = ENCODER_DIR_FORWARD,
        .mode = ENCODER_MODE_TI12
};

//编码器初始化
void encoder_init(Encoder_Speed_t *left_encoder, Encoder_Speed_t *right_encoder)
{
    left_encoder->temp_count = 0;
    left_encoder->count = 0;
    left_encoder->total_count = 0;
    left_encoder->mode = ENCODER_MODE_TI12;
    left_encoder->direction = ENCODER_DIR_FORWARD;
    right_encoder->temp_count = 0;
    right_encoder->count = 0;
    right_encoder->total_count = 0;
    right_encoder->mode = ENCODER_MODE_TI12;
    right_encoder->direction = ENCODER_DIR_FORWARD;

    //左编码器引脚外部中断
    NVIC_ClearPendingIRQ(ENCODER1_INT_IRQN);
    NVIC_EnableIRQ(ENCODER1_INT_IRQN);
    //右编码器引脚外部中断
    NVIC_ClearPendingIRQ(ENCODER2_INT_IRQN);
    NVIC_EnableIRQ(ENCODER2_INT_IRQN);
    //定时器中断
}


void GROUP1_IRQHandler(void) {
    // 读取编码器1的中断状态
    uint32_t gpio_status1 = DL_GPIO_getEnabledInterruptStatus(ENCODER1_PORT,
                                                              ENCODER1_PIN_A_PIN | ENCODER1_PIN_B_PIN);
    // 读取编码器2的中断状态
    uint32_t gpio_status2 = DL_GPIO_getEnabledInterruptStatus(ENCODER2_PORT,
                                                              ENCODER2_PIN_A2_PIN | ENCODER2_PIN_B2_PIN);

    // 处理编码器1
    if (gpio_status1 & (ENCODER1_PIN_A_PIN | ENCODER1_PIN_B_PIN)) {
        // 读取当前AB相电平状态
        bool pinA1_state = DL_GPIO_readPins(ENCODER1_PORT, ENCODER1_PIN_A_PIN);
        bool pinB1_state = DL_GPIO_readPins(ENCODER1_PORT, ENCODER1_PIN_B_PIN);

        static uint8_t last_state1 = 0;
        uint8_t current_state1 = (pinA1_state ? 2 : 0) + (pinB1_state ? 1 : 0);

        static const int8_t state_table1[16] = {
                0, 1, -1, 0,
                -1, 0, 0, 1,
                1, 0, 0, -1,
                0, -1, 1, 0
        };

        int8_t count_change1 = state_table1[(current_state1 << 2) | last_state1];
        motor_encoder_left.temp_count += count_change1;

        last_state1 = current_state1;
        DL_GPIO_clearInterruptStatus(ENCODER1_PORT, ENCODER1_PIN_A_PIN | ENCODER1_PIN_B_PIN);
    }

    // 处理编码器2
    if (gpio_status2 & (ENCODER2_PIN_A2_PIN | ENCODER2_PIN_B2_PIN)) {
        // 读取当前AB相电平状态
        bool pinA2_state = DL_GPIO_readPins(ENCODER2_PORT, ENCODER2_PIN_A2_PIN);
        bool pinB2_state = DL_GPIO_readPins(ENCODER2_PORT, ENCODER2_PIN_B2_PIN);

        static uint8_t last_state2 = 0;
        uint8_t current_state2 = (pinA2_state ? 2 : 0) + (pinB2_state ? 1 : 0);

        static const int8_t state_table2[16] = {
                0, 1, -1, 0,
                -1, 0, 0, 1,
                1, 0, 0, -1,
                0, -1, 1, 0
        };

        int8_t count_change2 = state_table2[(current_state2 << 2) | last_state2];
        motor_encoder_right.temp_count += count_change2;

        last_state2 = current_state2;
        DL_GPIO_clearInterruptStatus(ENCODER2_PORT, ENCODER2_PIN_A2_PIN | ENCODER2_PIN_B2_PIN);
    }
}



//编码器数据更新
//请间隔一定时间更新
void encoder_update(Encoder_Speed_t *left_encoder, Encoder_Speed_t *right_encoder)
{
    int64_t left_old_count = left_encoder->count;
    int64_t right_old_count = right_encoder->count;

    left_encoder->count = motor_encoder_left.temp_count;
    right_encoder->count = motor_encoder_right.temp_count;

    left_encoder->direction = ( motor_encoder_left.count >= 0 ) ? ENCODER_DIR_FORWARD : ENCODER_DIR_REVERSE;
    right_encoder->direction = ( motor_encoder_right.count >= 0 ) ? ENCODER_DIR_FORWARD : ENCODER_DIR_REVERSE;

    // 更新总计数（累加计数值变化）
    left_encoder->total_count += (left_encoder->count - left_old_count);
    right_encoder->total_count += (right_encoder->count - right_old_count);
    left_encoder->angle_deg = left_encoder->total_count / (float)ENCODER_COUNTS_PER_OUTPUT_REV * 360.0f;
    right_encoder->angle_deg = right_encoder->total_count / (float)ENCODER_COUNTS_PER_OUTPUT_REV * 360.0f;
    // 一阶低通滤波，减少噪声
    if (left_encoder->filtered_angle_deg == 0.0f) {
        // 首次初始化
        left_encoder->filtered_angle_deg = left_encoder->angle_deg;
    } else {
        // 低通滤波公式: y(n) = α*x(n) + (1-α)*y(n-1)
        left_encoder->filtered_angle_deg = ANGLE_FILTER_FACTOR * left_encoder->angle_deg +
                                           (1.0f - ANGLE_FILTER_FACTOR) * left_encoder->filtered_angle_deg;
    }
}

// M法速度计算函数
void calculate_speed_M_method(Encoder_Speed_t *encoder) {
    // 使用systick_get_delta获取时间差(秒)
    float time_delta_s = timer_dwt_get_delta(&encoder->DWT_CNT);
    //printf("time_delta: %.6f s\n", time_delta_s);
    // 防止除零错误
    if (time_delta_s < 0.0001f) {
        return;
    }

    // 当达到采样时间间隔时计算速度
    int32_t count_delta = encoder->count - encoder->last_count;
    // M法公式: speed(rpm) = (脉冲变化数 * 60秒) / (采样时间 * 编码器分辨率 * 减速比)
    encoder->speed_rpm = (float)count_delta * 60.0f /
                         (time_delta_s * ENCODER_RESOLUTION * GEAR_RATIO);

    // 一阶低通滤波，减少噪声
    if (encoder->filtered_speed_rpm == 0.0f) {
        // 首次初始化
        encoder->filtered_speed_rpm = encoder->speed_rpm;
    } else {
        // 低通滤波公式: y(n) = α*x(n) + (1-α)*y(n-1)
        encoder->filtered_speed_rpm = SPEED_FILTER_FACTOR * encoder->speed_rpm +
                                      (1.0f - SPEED_FILTER_FACTOR) * encoder->filtered_speed_rpm;
    }

    // 更新时间和计数
    encoder->last_count = encoder->count;
}

// T法速度计算函数

// M/T法速度计算函数


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
