//
// Created by 刘嘉俊 on 25-4-28.
//

#ifndef F407VGT6_CARARM_PWM_MOTOR_H
#define F407VGT6_CARARM_PWM_MOTOR_H

#include "stm32f4xx_hal.h"

/* 电机控制结构体 */
typedef struct {
    TIM_HandleTypeDef *htim_pwm;     // PWM定时器句柄
    uint32_t pwm_channel;           // PWM通道
    GPIO_TypeDef *ain1_port;        // AIN1端口
    uint16_t ain1_pin;              // AIN1引脚
    GPIO_TypeDef *ain2_port;        // AIN2端口
    uint16_t ain2_pin;              // AIN2引脚
    TIM_HandleTypeDef *htim_encoder; // 编码器定时器
} Motor_Config_t;

/* 电机方向枚举 */
typedef enum {
    MOTOR_FORWARD, // 正转
    MOTOR_REVERSE,  // 反转
    MOTOR_BRAKE  // 制动刹车
} Motor_Direction_e;

/* 初始化函数 */
void Motor_GPIO_Init(Motor_Config_t *motor);

/* 基础控制函数 */
void Motor_Set_Speed(Motor_Config_t *motor, int32_t speed);
void Motor_Set_Direction(Motor_Config_t *motor, Motor_Direction_e dir);
void Motor_Stop(Motor_Config_t *motor);

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#endif //F407VGT6_CARARM_PWM_MOTOR_H
