//
// Created by ���ο� on 25-1-14.
//

#ifndef CTRBOARD_H7_ALL_DMMOTOR_TASK_H
#define CTRBOARD_H7_ALL_DMMOTOR_TASK_H
#include "cmsis_os.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"

// ת���궨��
#define DEG_TO_RAD(x) ((x) * (M_PI / 180.0f)) // ����ת����
#define RAD_TO_DEG(x) ((x) * (180.0f / M_PI)) // ����ת����
#define NUM_INITIAL_READINGS 10 // ���ö�ȡ����

// ���ֱȶ���
#define GEAR_RATIO_6 3.24074f   // 6�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 3.24Ȧ
#define GEAR_RATIO_5 1.55556f  // 5�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 1.5556Ȧ
#define GEAR_RATIO_2 2.4f      // 2�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 2.4Ȧ

// ĩ�˳��ֽǶ�����
#define END_GEAR_MIN_LIMIT_5 (-91.0f+180.0f)
#define END_GEAR_MAX_LIMIT_5 (91.0f+180.0f)  // ��������180�ȿ�ʼ

// ���ŵ���ĽǶ����� (0�� �� -290��)
#define MOTOR_3_MIN_LIMIT (-0.5f)
#define MOTOR_3_MAX_LIMIT 0.5f

// ������ŵ���ĽǶ����ƣ�ͨ�����ֱȣ�
#define MOTOR_2_MIN_LIMIT (-1.0f)//(0.0f * 2.4f)
#define MOTOR_2_MAX_LIMIT (1.0f)

// ����һ�ŵ���ĽǶ�����
#define MOTOR_1_MIN_LIMIT (-3.05f)
#define MOTOR_1_MAX_LIMIT 3.05f

typedef struct {
    float motor_min_limit;       // �����С�Ƕ�����
    float motor_max_limit;       // ������Ƕ�����
    float initial_offset;        // ����ĳ�ʼƫ����ȣ�
    float last_angle;            // �����һ�εĽǶȣ����ȣ�
    int calibrated;              // У׼״̬
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
