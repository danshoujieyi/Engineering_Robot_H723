//
// Created by 刘嘉俊 on 25-1-2.
//

#ifndef CTRBOARD_H7_ALL_CHASSIS_TASK_H
#define CTRBOARD_H7_ALL_CHASSIS_TASK_H
#include "cmsis_os.h"

/**
 * @brief 底盘模式
 */
typedef enum
{
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_ENABLE,     //底盘开环
    CHASSIS_STOP,          //底盘停止
    CHASSIS_FOLLOW_GIMBAL, //底盘跟随云台
    CHASSIS_SPIN,          //底盘陀螺模式
    CHASSIS_FLY,           //底盘飞坡模式
    CHASSIS_AUTO           //底盘自动模式
} chassis_mode_e;


#endif //CTRBOARD_H7_ALL_CHASSIS_TASK_H
