#include <stdio.h>
#include <math.h>
#include "DMmotor_task.h"

static float dm_motor_angles[6] = {0}; // 从队列中读取的角度值（度数）
static const int INTERPOLATION_STEPS = 5; // 插值步数  // 不再使用插值算法，降低延迟
static const int TIME_STEP_MS = 1;        // 每步插值的时间间隔（毫秒）
static float current_angle = 0.0f;        // 当前插值角度
static int calibrated[6] = {0};           // 校准状态
const float MAX_ANGLE_CHANGE = 0.05f;

extern unsigned char DMmotor_init_flag; // 初始化标志位
extern float float_values[7]; // 实际赋值给关节电机角度

unsigned char DMmotor_init_flag = 0; // 初始化标志位

DMmotorControl motor_controls[6] = {
        { MOTOR_1_MIN_LIMIT, MOTOR_1_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 0 (FDCAN3)
        { MOTOR_2_MIN_LIMIT, MOTOR_2_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 1 (FDCAN2)
        { MOTOR_3_MIN_LIMIT, MOTOR_3_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 2 (FDCAN2) — 限制为 0 到 290
        { -M_PI, M_PI, 0.0f, 0.0f, 0 }, // Motor 3 (FDCAN2)
        { -M_PI, M_PI, 0.0f, 0.0f, 0 }, // Motor 4 (FDCAN2)
        { -M_PI, M_PI, 0.0f, 0.0f, 0 }  // Motor 5 (FDCAN2)
};

/**
 * @brief 主任务入口函数
 */
void DMmotor_Entry(void const * argument) {
    for (int i = 0; i < 6; i++) {
        motor_controls[i].last_angle = 0.0f;
        motor_controls[i].initial_offset = 0.0f;
        motor_controls[i].calibrated = 0;
    }

    dm_motor_enable(&hfdcan3, &motor[Motor1]);
    vTaskDelay(220); // 延时，等待电机稳定
    pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 0.7f); // 发送控制命令
    vTaskDelay(300); // 延时，等待电机稳定

    for(int i=1;i<6;i++)
    {
        dm_motor_enable(&hfdcan2, &motor[i]);
        vTaskDelay(220); // 延时，等待电机稳定
        pos_ctrl(&hfdcan2, motor[i].id, 0, 0.7f); // 发送控制命令
        vTaskDelay(300); // 延时，等待电机稳定
    }
    DMmotor_init_flag = 1; // 标记初始化完成


    for (;;) {
        if(DMmotor_init_flag == 1){
            for (int i = 0; i < 6; i++) {
                dm_motor_angles[i] = float_values[i];
                printf("dm_motor_angles: %f %f %f %f %f %f\n",float_values[0],float_values[1],float_values[2],
                       float_values[3],float_values[4],float_values[5]);
            }
            // 控制每个电机
            DMcontrol_motor_1(&hfdcan3, &motor_controls[Motor1], dm_motor_angles[Motor1]);
            DMcontrol_motor_2(&hfdcan2, &motor_controls[Motor2], dm_motor_angles[Motor2]);
            DMcontrol_motor_3(&hfdcan2, &motor_controls[Motor3], dm_motor_angles[Motor3]);
            DMcontrol_motor_4(&hfdcan2, &motor_controls[Motor4], dm_motor_angles[Motor4]);
            DMcontrol_motor_5(&hfdcan2, &motor_controls[Motor5], dm_motor_angles[Motor5]);
            DMcontrol_motor_6(&hfdcan2, &motor_controls[Motor6], dm_motor_angles[Motor6]);
        } else {

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

void smooth_motion_1(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    // 插值算法
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps; // 归一化时间
        float smooth_t = ease_in_out(t); // 应用平滑函数
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        // 保留三位小数
        current_angle = roundf(current_angle * 1000.0f) / 1000.0f;
        pos_ctrl(hcan, motor->id, current_angle, 10.0f); // 发送控制命令
        //pos_ctrl(hcan, motor->id, current_angle, 10.0f); // 发送控制命令
        vTaskDelay(pdMS_TO_TICKS(time_step_ms)); // 插值步之间的延时
    }
//    current_angle = target_angle;  // 直接目标角度
//    // 发送控制命令，不再使用平滑过渡
//    pos_ctrl(hcan, motor->id, current_angle, 10.0f); // 发送控制命令
//    // 延时，每一步之间的延时
//    vTaskDelay(pdMS_TO_TICKS(time_step_ms)); // 插值步之间的延时
}

void smooth_motion_0(hcan_t* hcan, motor_t* motor, float start_angle, float target_angle, int steps, int time_step_ms) {
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float smooth_t = ease_in_out(t);
        current_angle = start_angle + smooth_t * (target_angle - start_angle);
        current_angle = roundf(current_angle * 1000.0f) / 1000.0f;
        pos_ctrl(hcan, motor->id, -current_angle, 10.0f);  // 仅current_angle符号不同，用于处理编码器与关节电机不同向问题
        vTaskDelay(pdMS_TO_TICKS(time_step_ms));
    }
}

float handle_angle_jump(float current_radians, float last_radians) {
    float diff = current_radians - last_radians;
    // 修正跨越边界的跳变
    if (current_radians > M_PI && last_radians < -M_PI) {
        // 从 -π 跳到 π，调整以继续减少
        diff -= 2.0f * M_PI;
    } else if (current_radians < -M_PI && last_radians > M_PI) {
        // 从 π 跳到 -π，调整以继续增加
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
        // 限制角度变化幅度
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff,motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_0(hcan, &motor[Motor1], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS,TIME_STEP_MS);
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
        target_angle = target_angle; // 将末端齿轮的角度转换为电机角度
        float target_radians = DEG_TO_RAD(target_angle) - motor_control->initial_offset;
        //float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // 限制角度变化幅度
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_0(hcan, &motor[Motor2], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
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
        // 限制角度变化幅度
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_0(hcan, &motor[Motor3], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
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
        // 限制角度变化幅度
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_0(hcan, &motor[Motor4], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
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
        // 限制末端齿轮角度在 -91° 到 91° 之间
        if (target_angle < END_GEAR_MIN_LIMIT_5) {
            target_angle = END_GEAR_MIN_LIMIT_5;
        } else if (target_angle > END_GEAR_MAX_LIMIT_5) {
            target_angle = END_GEAR_MAX_LIMIT_5;
        }
        target_angle = target_angle / GEAR_RATIO_5; // 将末端齿轮的角度转换为电机角度
        float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset/ GEAR_RATIO_5);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // 限制角度变化幅度
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_1(hcan, &motor[Motor5], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
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
        target_angle = target_angle/GEAR_RATIO_6; // 将末端齿轮的角度转换为电机角度
        float target_radians = normalize_radians(DEG_TO_RAD(target_angle) - motor_control->initial_offset/GEAR_RATIO_6);
        float angle_diff = handle_angle_jump(target_radians, motor_control->last_angle);
        // 限制角度变化幅度
        if (fabs(angle_diff) > MAX_ANGLE_CHANGE) {
            angle_diff = (angle_diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
        }
        float clamped_target_angle = clamp_radians(motor_control->last_angle + angle_diff, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_1(hcan, &motor[Motor6], motor_control->last_angle, clamped_target_angle, INTERPOLATION_STEPS, TIME_STEP_MS);
        motor_control->last_angle = clamped_target_angle;

    }
}

