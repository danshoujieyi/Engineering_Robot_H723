//
// Created by ���ο� on 25-6-21.
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
    // IMU����ֵ
    float gyro[3];  // ���ٶ�
    float accel[3]; // ���ٶ�
    float motion_accel_b[3]; // ����������ٶ�
    // λ��
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
};
/** -------------------------------- Ins_Task Topics_Msg ------------------------------- **/

/** -------------------------------- Cmd_Task Topics_Msg ------------------------------- **/
struct cmd_chassis_msg
{
    float vx;                  // ǰ�������ٶ�
    float vy;                  // ���Ʒ����ٶ�
    float vw;                  // ��ת�ٶ�
    float offset_angle;        // ���̺͹���λ�õļн�
    chassis_mode_e ctrl_mode;  // ��ǰ���̿���ģʽ
    chassis_mode_e last_mode;  // ��һ�ε��̿���ģʽ
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
