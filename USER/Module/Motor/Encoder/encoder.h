//
// Created by 刘嘉俊 on 25-4-28.
//

#ifndef F407VGT6_CARARM_ENCODER_H
#define F407VGT6_CARARM_ENCODER_H

#include "ti_msp_dl_config.h"

// 编码器参数配置
#define ENCODER_PPR           11      // 编码器原始线数
#define ENCODER_MULTIPLIER    4       // 4倍频
#define ENCODER_RESOLUTION    (ENCODER_PPR * ENCODER_MULTIPLIER)  // 44脉冲/转
#define GEAR_RATIO            30.0f   // 减速比
#define ENCODER_COUNTS_PER_OUTPUT_REV 1320 //(ENCODER_RESOLUTION * MOTOR_GEAR_RATIO) // 输出轴每转计数 = 1320

// M法配置参数
#define SPEED_FILTER_FACTOR   0.3f    // 速度滤波系数(0.0~1.0)
#define ANGLE_FILTER_FACTOR   0.15f    // 速度滤波系数(0.0~1.0)

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

/* 编码器配置结构体 */
typedef struct {
    Encoder_Mode_e mode;           // 编码器工作模式
    Encoder_Direction_e direction; // 计数方向

    volatile int64_t temp_count; //保存实时计数值
    volatile int64_t count;      //根据定时器时间更新的计数值
    int32_t last_count;             // 上次采样时的计数值

    volatile int64_t total_count;       // 编码器累计总计数（用于位置计算）

    uint32_t DWT_CNT;             // 上次采样的时间戳

    float speed_rpm;                // 当前速度(rpm)
    float filtered_speed_rpm;       // 滤波后的速度(rpm)
    float angle_deg;
    float filtered_angle_deg;
    float angle_rad;
    float filtered_angle_rad;
} Encoder_Speed_t;

void encoder_init(Encoder_Speed_t *left_encoder, Encoder_Speed_t *right_encoder);
void encoder_update(Encoder_Speed_t *left_encoder, Encoder_Speed_t *right_encoder);

void calculate_speed_M_method(Encoder_Speed_t *encoder);

#endif //F407VGT6_CARARM_ENCODER_H
