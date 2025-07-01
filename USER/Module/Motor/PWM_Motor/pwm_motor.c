//
// Created by 刘嘉俊 on 25-4-28.
//

#include "pwm_motor.h"

/* GPIO初始化（CubeMX已配置外设，这里只做补充配置） */
void Motor_GPIO_Init(Motor_Config_t *motor)
{

    /* 确保PWM定时器已启动 */
    HAL_TIM_PWM_Start(motor->htim_pwm, motor->pwm_channel);

    /* 初始化控制引脚为输出模式（假设CubeMX已配置GPIO） */
    HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_RESET);

    /* 默认停止电机 */
    Motor_Stop(motor);
}

/* 设置电机方向 */
void Motor_Set_Direction(Motor_Config_t *motor, Motor_Direction_e dir)
{
    switch(dir) {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_RESET);
            break;

        case MOTOR_REVERSE:
            HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_SET);
            break;

        case MOTOR_BRAKE:
            HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_SET);
            break;
    }
}

/* 设置电机速度（0~8400对应0~100%） */
void Motor_Set_Speed(Motor_Config_t *motor, int32_t speed)
{
    /* 限幅处理 */
    VAL_LIMIT(speed, 0, 8400);

    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, speed);
}

/* 停止电机 */
void Motor_Stop(Motor_Config_t *motor)
{
    Motor_Set_Speed(motor, 0);
    Motor_Set_Direction(motor, MOTOR_BRAKE);
}