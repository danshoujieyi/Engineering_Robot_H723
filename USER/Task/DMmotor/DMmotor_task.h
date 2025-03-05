//
// Created by 刘嘉俊 on 25-1-14.
//

#ifndef CTRBOARD_H7_ALL_DMMOTOR_TASK_H
#define CTRBOARD_H7_ALL_DMMOTOR_TASK_H
#include "cmsis_os.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"

// 转换宏定义
#define DEG_TO_RAD(x) ((x) * (M_PI / 180.0f)) // 度数转弧度
#define RAD_TO_DEG(x) ((x) * (180.0f / M_PI)) // 弧度转度数
#define NUM_INITIAL_READINGS 10 // 设置读取次数

// 齿轮比定义
#define GEAR_RATIO_6 3.24074f   // 6号电机转动 1圈，末端齿轮转动 3.24圈
#define GEAR_RATIO_5 1.55556f  // 5号电机转动 1圈，末端齿轮转动 1.5556圈
#define GEAR_RATIO_2 2.4f      // 2号电机转动 1圈，末端齿轮转动 2.4圈

// 末端齿轮角度限制
#define END_GEAR_MIN_LIMIT_5 (-91.0f+180.0f)
#define END_GEAR_MAX_LIMIT_5 (91.0f+180.0f)  // 编码器从180度开始

// 三号电机的角度限制 (0° 到 -290°)
#define MOTOR_3_MIN_LIMIT (-0.5f)
#define MOTOR_3_MAX_LIMIT 0.5f

// 计算二号电机的角度限制（通过齿轮比）
#define MOTOR_2_MIN_LIMIT (-1.0f)//(0.0f * 2.4f)
#define MOTOR_2_MAX_LIMIT (1.0f)

// 计算一号电机的角度限制
#define MOTOR_1_MIN_LIMIT (-3.05f)
#define MOTOR_1_MAX_LIMIT 3.05f

typedef struct {
    float motor_min_limit;       // 电机最小角度限制
    float motor_max_limit;       // 电机最大角度限制
    float initial_offset;        // 电机的初始偏差（弧度）
    float last_angle;            // 电机上一次的角度（弧度）
    int calibrated;              // 校准状态
} DMmotorControl;


float normalize_radians(float radians);

float clamp_radians(float radians, float min_limit, float max_limit);

float ease_in_out(float t);

void smooth_motion_1(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms);

void smooth_motion_0(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms);

float handle_angle_jump(float current_radians, float last_radians);

void DMcontrol_motor_1(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_2(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_3(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_4(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_5(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_6(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);



#endif //CTRBOARD_H7_ALL_DMMOTOR_TASK_H
