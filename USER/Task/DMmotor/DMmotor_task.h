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
#define GEAR_RATIO_6 3.24074f   // 6�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 3.24Ȧ   // �����޸����������ּ�϶���
#define GEAR_RATIO_5 1.55556f  // 5�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 1.5556Ȧ
#define GEAR_RATIO_2 2.4f                  // 2�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 2.4Ȧ

// ��ŵ���ĽǶ�����
#define MOTOR_5_MIN_LIMIT (-1.0f)  //  5�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 1.5556Ȧ,�޷���ʮ��
#define MOTOR_5_MAX_LIMIT (1.0f)

// �ĺŵ���ĽǶ�����
#define MOTOR_4_MIN_LIMIT (-3.0f)  // ���3.14
#define MOTOR_4_MAX_LIMIT 3.0f

// ���ŵ���ĽǶ�����
#define MOTOR_3_MIN_LIMIT 0.0f
#define MOTOR_3_MAX_LIMIT 2.0f    // ��λ˵����2Ϊ����Խ����λ��2.6���죬3.1Խ����λ��4.2����ֱ��4.9��ֱ���̣�5.2����

// ������ŵ���ĽǶ����ƣ�ͨ�����ֱȣ�
#define MOTOR_2_MIN_LIMIT (-5.82f/GEAR_RATIO_2) // �ȳ����ֱȵȽ��ձ��������ݺ��ٳ˻���
#define MOTOR_2_MAX_LIMIT (-0.01f)

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
