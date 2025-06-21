//
// Created by 刘嘉俊 on 25-6-21.
//

#ifndef CTRBOARD_H7_ALL_ROBOT_TASK_H
#define CTRBOARD_H7_ALL_ROBOT_TASK_H

#include <cmsis_os.h>
#include "chassis_task.h"

#ifdef BSP_USING_EXAMPLE_TASK
#include "example_task.h"
#endif /* BSP_USING_EXAMPLE_TASK */
#ifdef BSP_USING_INS_TASK
#include "ins_task.h"
#endif /* BSP_USING_INS_TASK */
#ifdef BSP_USING_MOTOR_TASK
#include "motor_task.h"
#endif /* BSP_USING_MOTOR_TASK */
#ifdef BSP_USING_CMD_TASK
#include "cmd_task.h"
#endif /* BSP_USING_CMD_TASK */
#ifdef BSP_USING_CHASSIS_TASK
#include "chassis_task.h"
#endif /* BSP_USING_CHASSIS_TASK */
#ifdef BSP_USING_GIMBAL_TASK
#include "gimbal_task.h"
#endif /* BSP_USING_GIMBAL_TASK */
#ifdef BSP_USING_TRANSMISSION_TASK
#include "transmission_task.h"
#endif /* BSP_USING_TRANSMISSION_TASK */
#ifdef BSP_USING_SHOOT_TASK
#include "shoot_task.h"
#endif /* BSP_USING_SHOOT_TASK */
#ifdef BSP_USING_REFEREE_TASK
#include "referee_task.h"
#include "Referee_system.h"
#endif /* BSP_USING_REFEREE_TASK */


/** -------------------------------- Ins_Task Topics_Msg ------------------------------- **/
struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    float motion_accel_b[3]; // 机体坐标加速度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
};
/** -------------------------------- Ins_Task Topics_Msg ------------------------------- **/

/** -------------------------------- Cmd_Task Topics_Msg ------------------------------- **/
struct cmd_chassis_msg
{
    float vx;                  // 前进方向速度
    float vy;                  // 横移方向速度
    float vw;                  // 旋转速度
    float offset_angle;        // 底盘和归中位置的夹角
    chassis_mode_e ctrl_mode;  // 当前底盘控制模式
    chassis_mode_e last_mode;  // 上一次底盘控制模式
};
/** -------------------------------- Cmd_Task Topics_Msg ------------------------------- **/

/** --------------------------- Transmission_Task Topics_Msg --------------------------- **/
struct transmission_msg
{
    float yaw;
    float pitch;
    uint8_t heartbeat;
};
/** --------------------------- Transmission_Task Topics_Msg --------------------------- **/

/** ------------------------- Chassis_Task Feedback Topics_Msg ------------------------- **/
struct chassis_feedback_msg
{
    float x_pos_gim;
    float y_pos_gim;
};
/** ------------------------- Chassis_Task Feedback Topics_Msg ------------------------ **/


#endif //CTRBOARD_H7_ALL_ROBOT_TASK_H
