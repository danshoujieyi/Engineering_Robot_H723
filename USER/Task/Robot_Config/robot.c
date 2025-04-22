//
// Created by 刘嘉俊 on 25-4-21.
//

#include "robot.h"
#include "stm32h7xx_hal.h"

void robot_init()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用 dwt 进行延时
    __disable_irq();

//    OS_task_init(); // 创建基础任务
//    MX_FREERTOS_Init(); // 任务创建由cubemax此函数生成

//    mcn_topic_init(); // 话题注册初始化

//    motor_task_init();
//    cmd_task_init();
//    chassis_task_init();
//    trans_task_init();
//    referee_UI_task_init();

    // 初始化完成,开启中断
    __enable_irq();
}