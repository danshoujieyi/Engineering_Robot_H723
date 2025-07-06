#ifndef F407VGT6_CARARM_PWM_MOTOR_H
#define F407VGT6_CARARM_PWM_MOTOR_H

#include "stm32h7xx_hal.h"

/* 电机方向枚举 */
#define MOTOR_FORWARD 0 // 正转
#define MOTOR_REVERSE 1 // 反转
#define MOTOR_STOP    2// 制动刹车

#define SPEED_PWM_MIN -8000
#define SPEED_PWM_MAX 8000

#define DEAD_ZONE_POS  460    // 正向死区阈值（电机正转最小启动值）  // 510
#define DEAD_ZONE_NEG  -460    // 反向死区阈值（电机反转最小启动值）

/* 电机控制结构体 */
typedef struct {
    TIM_HandleTypeDef *htim_pwm;     // PWM定时器句柄
    uint32_t pwm_channel;           // PWM通道
    GPIO_TypeDef *ain1_port;        // AIN1端口
    uint16_t ain1_pin;              // AIN1引脚
    GPIO_TypeDef *ain2_port;        // AIN2端口
    uint16_t ain2_pin;              // AIN2引脚
} Motor_Config_t;

/* 电机控制结构体 */
typedef struct {
    GPTIMER_Regs *htim_pwm;     // PWM定时器句柄
    DL_TIMER_CC_INDEX pwm_channel;           // PWM通道
    GPIO_Regs *ain1_port;        // AIN1端口
    uint32_t ain1_pin;              // AIN1引脚
    GPIO_Regs *ain2_port;        // AIN2端口
    uint32_t ain2_pin;              // AIN2引脚
} Motor_Config_t;

/* 初始化函数 */
void Motor_GPIO_Init(Motor_Config_t *motor);

/* 基础控制函数 */
void Motor_Set_Speed(Motor_Config_t *motor, float speed_pwm);
void Motor_Set_Direction(Motor_Config_t *motor, uint8_t direction);
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
