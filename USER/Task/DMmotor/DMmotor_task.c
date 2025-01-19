#include <stdio.h>
#include <math.h>
#include <string.h>
#include "DMmotor_task.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"

extern QueueHandle_t xQueueMotor; // �ⲿ���о������
static float dm_motor_angles[6] = {0}; // �Ӷ����ж�ȡ�ĽǶ�ֵ��������
static const int INTERPOLATION_STEPS = 5; // ��ֵ����
static const int TIME_STEP_MS = 1;        // ÿ����ֵ��ʱ���������룩
static float current_angle = 0.0f;        // ��ǰ��ֵ�Ƕ�
static int calibrated[6] = {0};           // У׼״̬
const float MAX_ANGLE_CHANGE = 0.05f;

// ת���궨��
#define DEG_TO_RAD(x) ((x) * (M_PI / 180.0f)) // ����ת����
#define RAD_TO_DEG(x) ((x) * (180.0f / M_PI)) // ����ת����
#define NUM_INITIAL_READINGS 10 // ���ö�ȡ����

// ���ֱȶ���
#define GEAR_RATIO_6 3.24074f   // 6�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 3.24Ȧ
#define GEAR_RATIO_5 1.55556f  // 5�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 1.5556Ȧ
#define GEAR_RATIO_2 2.4f      // 2�ŵ��ת�� 1Ȧ��ĩ�˳���ת�� 2.4Ȧ

// ĩ�˳��ֽǶ�����
#define END_GEAR_MIN_LIMIT_5 -91.0f+180.0f
#define END_GEAR_MAX_LIMIT_5 91.0f+180.0f  // ��������180�ȿ�ʼ

// ���ŵ���ĽǶ����� (0�� �� -290��)
#define MOTOR_3_MIN_LIMIT -5.06145f
#define MOTOR_3_MAX_LIMIT 0.0f

// ������ŵ���ĽǶ����ƣ�ͨ�����ֱȣ�
#define MOTOR_2_MIN_LIMIT (0.0f * 2.4f)
#define MOTOR_2_MAX_LIMIT (2.41658f * 2.4f)

// ����һ�ŵ���ĽǶ����ƣ�ͨ�����ֱȣ�
#define MOTOR_1_MIN_LIMIT -5.06f
#define MOTOR_1_MAX_LIMIT 0.0f

float normalize_radians(float radians) {
    while (radians >= M_PI) radians -= 2.0f * M_PI;
    while (radians < -M_PI) radians += 2.0f * M_PI;
    return radians;
}

float clamp_radians(float radians, float min_limit, float max_limit) {
    if (radians > max_limit) return max_limit;
    if (radians < min_limit) return min_limit;
    return radians;
}

float ease_in_out(float t) {
    return t < 0.5f ? 2.0f * t * t : -1.0f + (4.0f - 2.0f * t) * t;
}

void smooth_motion(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps; // ��һ��ʱ��
        float smooth_t = ease_in_out(t); // Ӧ��ƽ������
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        // ������λС��
        current_angle = roundf(current_angle * 1000.0f) / 1000.0f;
        pos_ctrl(hcan, motor->id, current_angle, 10.0f); // ���Ϳ�������
        //pos_ctrl(hcan, motor->id, current_angle, 10.0f); // ���Ϳ�������
        vTaskDelay(pdMS_TO_TICKS(time_step_ms)); // ��ֵ��֮�����ʱ
    }
}

float handle_angle_jump(float current_radians, float last_radians) {
    float diff = current_radians - last_radians;
    // ������Խ�߽������
    if (current_radians > M_PI && last_radians < -M_PI) {
        // �� -�� ���� �У������Լ�������
        diff -= 2.0f * M_PI;
    } else if (current_radians < -M_PI && last_radians > M_PI) {
        // �� �� ���� -�У������Լ�������
        diff += 2.0f * M_PI;
    }
    return diff;
}

typedef struct {
    float motor_min_limit;       // �����С�Ƕ�����
    float motor_max_limit;       // ������Ƕ�����
    float initial_offset;        // ����ĳ�ʼƫ����ȣ�
    float last_angle;            // �����һ�εĽǶȣ����ȣ�
    int calibrated;              // У׼״̬
} DMmotorControl;

DMmotorControl motor_controls[6] = {
        { MOTOR_1_MIN_LIMIT, MOTOR_1_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 0 (FDCAN3)
        { MOTOR_2_MIN_LIMIT, MOTOR_2_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 1 (FDCAN2)
        { MOTOR_3_MIN_LIMIT, MOTOR_3_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 2 (FDCAN2) �� ����Ϊ 0 �� 290
        { -M_PI, M_PI, 0.0f, 0.0f, 0 }, // Motor 3 (FDCAN2)
        { -M_PI, M_PI, 0.0f, 0.0f, 0 }, // Motor 4 (FDCAN2)
        { -M_PI, M_PI, 0.0f, 0.0f, 0 }  // Motor 5 (FDCAN2)
};

void DMcontrol_motor_1(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor1] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset = DEG_TO_RAD(dm_motor_angles[Motor1]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1) {
        target_angle = target_angle;
        float target_radians =DEG_TO_RAD(target_angle) - motor_control->initial_offset;
        //float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff,motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion(hcan, &motor[Motor1], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS,TIME_STEP_MS);
        motor_control->last_angle = clamped_target_angle;
    }
}

void DMcontrol_motor_2(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor2] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset = DEG_TO_RAD(dm_motor_angles[Motor2]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        target_angle = target_angle; // ��ĩ�˳��ֵĽǶ�ת��Ϊ����Ƕ�
        float target_radians = DEG_TO_RAD(target_angle) - motor_control->initial_offset;
        //float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion(hcan, &motor[Motor2], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
        motor_control->last_angle = clamped_target_angle;
    }
}

void DMcontrol_motor_3(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor3] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset = DEG_TO_RAD(dm_motor_angles[Motor3]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        target_angle = target_angle;
        float target_radians = DEG_TO_RAD(target_angle) - motor_control->initial_offset;
       // float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion(hcan, &motor[Motor3], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
        motor_control->last_angle = clamped_target_angle;
    }
}

void DMcontrol_motor_4(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor4] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset = DEG_TO_RAD(dm_motor_angles[Motor4]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        target_angle = target_angle;
        float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion(hcan, &motor[Motor4], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
        motor_control->last_angle = clamped_target_angle;
    }
}

void DMcontrol_motor_5(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor5] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset = DEG_TO_RAD(dm_motor_angles[Motor5]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        // ����ĩ�˳��ֽǶ��� -91�� �� 91�� ֮��
        if (target_angle < END_GEAR_MIN_LIMIT_5) {
            target_angle = END_GEAR_MIN_LIMIT_5;
        } else if (target_angle > END_GEAR_MAX_LIMIT_5) {
            target_angle = END_GEAR_MAX_LIMIT_5;
        }
        target_angle = target_angle / GEAR_RATIO_5; // ��ĩ�˳��ֵĽǶ�ת��Ϊ����Ƕ�
        float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset/ GEAR_RATIO_5);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion(hcan, &motor[Motor5], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
        motor_control->last_angle = clamped_target_angle;
    }
}

void DMcontrol_motor_6(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor6] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset = DEG_TO_RAD(dm_motor_angles[Motor6]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        target_angle = target_angle/GEAR_RATIO_6; // ��ĩ�˳��ֵĽǶ�ת��Ϊ����Ƕ�
        float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset/GEAR_RATIO_6);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion(hcan, &motor[Motor6], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
        motor_control->last_angle = clamped_target_angle;

    }
}

extern unsigned char DMmotor_init_flag; // ��ʼ����־λ
/**
 * @brief ��������ں���
 */
void DMmotor_Entry(void const * argument) {
    for (int i = 0; i < 6; i++) {
        motor_controls[i].last_angle = 0.0f;
        motor_controls[i].initial_offset = 0.0f;
        motor_controls[i].calibrated = 0;
    }
    for (;;) {
        // �Ӷ����н��սǶ�ֵ��������������ʱ��Ϊ 10ms
        if (xQueueReceive(xQueueMotor, dm_motor_angles, pdMS_TO_TICKS(10)) == pdTRUE) {
            printf("Data received (degrees): %f, %f, %f, %f, %f, %f\n",
                   dm_motor_angles[0], dm_motor_angles[1], dm_motor_angles[2],
                   dm_motor_angles[3], dm_motor_angles[4], dm_motor_angles[5]);
            if(DMmotor_init_flag == 1){
                // ����ÿ�����
                DMcontrol_motor_1(&hfdcan3, &motor_controls[Motor1], dm_motor_angles[Motor1]);
                DMcontrol_motor_2(&hfdcan2, &motor_controls[Motor2], dm_motor_angles[Motor2]);
                DMcontrol_motor_3(&hfdcan2, &motor_controls[Motor3], dm_motor_angles[Motor3]);
                DMcontrol_motor_4(&hfdcan2, &motor_controls[Motor4], dm_motor_angles[Motor4]);
                DMcontrol_motor_5(&hfdcan2, &motor_controls[Motor5], dm_motor_angles[Motor5]);
                DMcontrol_motor_6(&hfdcan2, &motor_controls[Motor6], dm_motor_angles[Motor6]);
            }
        } else {
            //printf("Queue read timeout.\n");
        }
        vTaskDelay(1);
    }
}

