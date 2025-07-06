//
// Created by 刘嘉俊 on 25-4-28.
//

#include "pwm_motor.h"
#include "encoder.h"

/* GPIO初始化（CubeMX已配置外设，这里只做补充配置） */
void Motor_GPIO_Init(Motor_Config_t *motor)
{
    /* 确保PWM定时器已启动 */


    /* 初始化控制引脚为输出模式（假设CubeMX已配置GPIO） */
    DL_GPIO_clearPins(motor->ain1_port, motor->ain1_pin);
    DL_GPIO_clearPins(motor->ain2_port, motor->ain2_pin);

    /* 默认停止电机 */
    Motor_Stop(motor);
}

/* 设置电机方向 */
void Motor_Set_Direction(Motor_Config_t *motor, uint8_t direction)
{
    switch(direction) {
        case MOTOR_FORWARD:
            DL_GPIO_clearPins(motor->ain1_port, motor->ain1_pin);
            DL_GPIO_setPins(motor->ain2_port, motor->ain2_pin);
            break;

        case MOTOR_REVERSE:
            DL_GPIO_setPins(motor->ain1_port, motor->ain1_pin);
            DL_GPIO_clearPins(motor->ain2_port, motor->ain2_pin);
            break;

        case MOTOR_STOP:
            DL_GPIO_clearPins(motor->ain1_port, motor->ain1_pin);
            DL_GPIO_clearPins(motor->ain2_port, motor->ain2_pin);
            break;
    }
}

/* 设置电机速度（0~10000对应0~100%） ，最大输入8000*/
void Motor_Set_Speed(Motor_Config_t *motor, float speed_pwm)
{
    uint32_t abs_speed;  // 速度绝对值
    /* 限幅处理 */
    VAL_LIMIT(speed_pwm, SPEED_PWM_MIN, SPEED_PWM_MAX);
    /* 解析方向和大小 */
    if (speed_pwm > 0) {
        DL_GPIO_clearPins(motor->ain1_port, motor->ain1_pin);
        DL_GPIO_setPins(motor->ain2_port, motor->ain2_pin);
        abs_speed = (uint32_t)speed_pwm + DEAD_ZONE_POS;
    } else if (speed_pwm < 0) {
        DL_GPIO_setPins(motor->ain1_port, motor->ain1_pin);
        DL_GPIO_clearPins(motor->ain2_port, motor->ain2_pin);
        abs_speed = (uint32_t)(-(speed_pwm + DEAD_ZONE_NEG));  // 取绝对值
    } else {
        DL_GPIO_clearPins(motor->ain1_port, motor->ain1_pin);
        DL_GPIO_clearPins(motor->ain2_port, motor->ain2_pin);
        abs_speed = 0;
    }

    DL_TimerG_setCaptureCompareValue(motor->htim_pwm, abs_speed, motor->pwm_channel);
}

/* 停止电机 */
void Motor_Stop(Motor_Config_t *motor)
{
    Motor_Set_Speed(motor, 0);
    Motor_Set_Direction(motor, MOTOR_STOP);
}
