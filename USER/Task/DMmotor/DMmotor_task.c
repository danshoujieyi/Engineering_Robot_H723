#include <stdio.h>
#include <math.h>
#include "DMmotor_task.h"

static float dm_motor_angles[6] = {0}; // �Ӷ����ж�ȡ�ĽǶ�ֵ��������
static const int INTERPOLATION_STEPS = 5; // ��ֵ����  // ����ʹ�ò�ֵ�㷨�������ӳ�
static const int TIME_STEP_MS = 1;//0;        // ÿ����ֵ��ʱ���������룩
static float current_angle = 0.0f;        // ��ǰ��ֵ�Ƕ�
static int calibrated[6] = {0};           // У׼״̬
const float MAX_ANGLE_CHANGE = 0.05f;

extern float float_values[7]; // ʵ�ʸ�ֵ���ؽڵ���Ƕ�
unsigned char DMmotor_init_flag = 0; // ��ʼ����־λ

DMmotorControl motor_controls[6] = {
        { MOTOR_1_MIN_LIMIT, MOTOR_1_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 0 (FDCAN3)
        { MOTOR_2_MIN_LIMIT, MOTOR_2_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 1 (FDCAN2)
        { MOTOR_3_MIN_LIMIT, MOTOR_3_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 2 (FDCAN2)
        { MOTOR_4_MIN_LIMIT, MOTOR_4_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 3 (FDCAN2)
        { MOTOR_5_MIN_LIMIT, MOTOR_5_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 4 (FDCAN2)
        { -M_PI, M_PI, 0.0f, 0.0f, 0 }  // Motor 5 (FDCAN2)
};

struct arm_cmd_msg arm_cmd = {
        .ctrl_mode = ARM_DISABLE,
        .last_mode = ARM_DISABLE
};


/* ------------------------------- ң������ת��Ϊ����ָ�� ------------------------------ */
static void remote_to_cmd_sbus(void);

void arm_cmd_enable(void) {
    if (arm_cmd.last_mode == ARM_DISABLE && arm_cmd.ctrl_mode == ARM_ENABLE) {
        dm_motor_enable(&hfdcan3, &motor[Motor1]);
        for(int i=1;i<6;i++)
        {
            dm_motor_enable(&hfdcan2, &motor[i]);
            vTaskDelay(1);
        }
        arm_cmd.last_mode = ARM_ENABLE;
    }
}
void arm_cmd_disable(void) {
    if (arm_cmd.last_mode == ARM_ENABLE && arm_cmd.ctrl_mode == ARM_DISABLE) {
        dm_motor_disable(&hfdcan3, &motor[Motor1]);
        for(int i=1;i<6;i++)
        {
            dm_motor_disable(&hfdcan2, &motor[i]);
            vTaskDelay(1);
        }
        arm_cmd.last_mode = ARM_DISABLE;
    }
}

void arm_cmd_init(void) {
    if (arm_cmd.last_mode == ARM_ENABLE && arm_cmd.ctrl_mode == ARM_INIT) {
        pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 0.7f); // ���Ϳ�������
        vTaskDelay(200); // ��ʱ���ȴ�����ȶ�

        for(int i=1;i<6;i++)
        {
            dm_motor_enable(&hfdcan2, &motor[i]);
            pos_ctrl(&hfdcan2, motor[i].id, 0, 0.7f); // ���Ϳ�������
            vTaskDelay(200); // ��ʱ���ȴ�����ȶ�
        }
        arm_cmd.last_mode = ARM_ENABLE;  //TODO:BUGһ��
    }
}

void arm_cmd_state_machine(void) {

    switch (arm_cmd.ctrl_mode) {
        case ARM_ENABLE:
            arm_cmd_enable();
            break;
        case ARM_DISABLE:
            arm_cmd_disable();
            break;
//        case ARM_INIT:
//            arm_cmd_init();
//            break;
        default:
//            arm_cmd_disable();
            break;
    }
}


/**
 * @brief ��������ں���
 */
void DMmotorTask_Entry(void const * argument) {
    for (int i = 0; i < 6; i++) {
        motor_controls[i].last_angle = 0.0f;
        motor_controls[i].initial_offset = 0.0f;
        motor_controls[i].calibrated = 0;
    }

    dm_motor_enable(&hfdcan3, &motor[Motor1]);
    vTaskDelay(200); // ��ʱ���ȴ�����ȶ�
    pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 0.7f); // ���Ϳ�������
    vTaskDelay(200); // ��ʱ���ȴ�����ȶ�

    for(int i=1;i<6;i++)
    {
        dm_motor_enable(&hfdcan2, &motor[i]);
        vTaskDelay(200); // ��ʱ���ȴ�����ȶ�
        pos_ctrl(&hfdcan2, motor[i].id, 0, 0.7f); // ���Ϳ�������
        vTaskDelay(200); // ��ʱ���ȴ�����ȶ�
    }
    DMmotor_init_flag = 1; // ��ǳ�ʼ�����
    arm_cmd.ctrl_mode = ARM_ENABLE; // ʹ�ܻ�е��
    arm_cmd.last_mode = ARM_ENABLE;
    for (;;) {
        if(DMmotor_init_flag == 1){
            for (int i = 0; i < 6; i++) {
                dm_motor_angles[i] = float_values[i];
                printf("dm_motor_angles: %f %f %f %f %f %f\n",float_values[0],float_values[1],float_values[2],
                       float_values[3],float_values[4],float_values[5]);
            }
            // ����ÿ�����
            DMcontrol_motor_1(&hfdcan3, &motor_controls[Motor1], dm_motor_angles[Motor1]);
            DMcontrol_motor_2(&hfdcan2, &motor_controls[Motor2], dm_motor_angles[Motor2]);
            DMcontrol_motor_3(&hfdcan2, &motor_controls[Motor3], dm_motor_angles[Motor3]);
            DMcontrol_motor_4(&hfdcan2, &motor_controls[Motor4], dm_motor_angles[Motor4]);
            DMcontrol_motor_5(&hfdcan2, &motor_controls[Motor5], dm_motor_angles[Motor5]);
            DMcontrol_motor_6(&hfdcan2, &motor_controls[Motor6], dm_motor_angles[Motor6]);
        }


        vTaskDelay(1);
    }
}




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

////    current_angle = target_angle;  // ֱ��Ŀ��Ƕ�
////    // ���Ϳ����������ʹ��ƽ������
////    pos_ctrl(hcan, motor->id, current_angle, 10.0f); // ���Ϳ�������
////    // ��ʱ��ÿһ��֮�����ʱ
////    vTaskDelay(pdMS_TO_TICKS(time_step_ms)); // ��ֵ��֮�����ʱ

void smooth_motion_1(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float smooth_t = ease_in_out(t);
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        current_angle = roundf(current_angle * 1000.0f) / 1000.0f;
        pos_ctrl(hcan, motor->id, -current_angle, 10.0f);  // ��current_angle���Ų�ͬ�����ڴ����������ؽڵ����ͬ������
        vTaskDelay(pdMS_TO_TICKS(time_step_ms));
    }
}

void smooth_motion_2(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float smooth_t = ease_in_out(t);
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        current_angle = ((roundf(current_angle * 1000.0f) / 1000.0f)*GEAR_RATIO_2);  //���ϳ��ֱ�
        pos_ctrl(hcan, motor->id, -current_angle, 10.0f);  // ��current_angle���Ų�ͬ�����ڴ����������ؽڵ����ͬ������
        vTaskDelay(pdMS_TO_TICKS(time_step_ms));
    }
}

void smooth_motion_3(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float smooth_t = ease_in_out(t);
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        current_angle = roundf(current_angle * 1000.0f) / 1000.0f;
        pos_ctrl(hcan, motor->id, -current_angle, 10.0f);  // ��current_angle���Ų�ͬ�����ڴ����������ؽڵ����ͬ������
        vTaskDelay(pdMS_TO_TICKS(time_step_ms));
    }
}

void smooth_motion_4(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float smooth_t = ease_in_out(t);
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        current_angle = roundf(current_angle * 1000.0f) / 1000.0f;
        pos_ctrl(hcan, motor->id, -current_angle, 10.0f);  // ��current_angle���Ų�ͬ�����ڴ����������ؽڵ����ͬ������
        vTaskDelay(pdMS_TO_TICKS(time_step_ms));
    }
}

void smooth_motion_5(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float smooth_t = ease_in_out(t);
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        current_angle = roundf(current_angle * 1000.0f) / 1000.0f;
        pos_ctrl(hcan, motor->id, -current_angle, 10.0f);  // ��current_angle���Ų�ͬ�����ڴ����������ؽڵ����ͬ������
        vTaskDelay(pdMS_TO_TICKS(time_step_ms));
    }
}

void smooth_motion_6(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float smooth_t = ease_in_out(t);
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        current_angle = roundf(current_angle * 1000.0f) / 1000.0f;
        pos_ctrl(hcan, motor->id, -current_angle, 10.0f);  // ��current_angle���Ų�ͬ�����ڴ����������ؽڵ����ͬ������
        vTaskDelay(pdMS_TO_TICKS(time_step_ms));
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
        //float target_radians =DEG_TO_RAD(target_angle) - motor_control->initial_offset;
        float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff,motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_1(hcan, &motor[Motor1], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS,TIME_STEP_MS);
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
        smooth_motion_2(hcan, &motor[Motor2], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
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
        smooth_motion_3(hcan, &motor[Motor3], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
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
        smooth_motion_4(hcan, &motor[Motor4], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
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
        target_angle = target_angle; // ��ĩ�˳��ֵĽǶ�ת��Ϊ����Ƕ�
        float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_5(hcan, &motor[Motor5], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
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
        target_angle = target_angle; // ��ĩ�˳��ֵĽǶ�ת��Ϊ����Ƕ�
        float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // ���ƽǶȱ仯����
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_6(hcan, &motor[Motor6], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
        motor_control->last_angle = clamped_target_angle;

    }
}

