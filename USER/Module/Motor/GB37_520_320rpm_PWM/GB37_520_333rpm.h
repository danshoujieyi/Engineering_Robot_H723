#ifndef GB37_520_333RPM_H
#define GB37_520_333RPM_H

#include "ti_msp_dl_config.h"
#include "PID.h"

// 电机基本参数
#define MOTOR_VOLTAGE_RATING        12      // 额定电压12V
#define MOTOR_NO_LOAD_SPEED         9600   // 减速前空载转速(rpm) - 修正为10000rpm
#define MOTOR_RATED_SPEED           320     // 减速后额定转速(rpm)

#define SPEED_KP_Left_motor              360.0f
#define SPEED_KI_Left_motor              50.0f
#define SPEED_KD_Left_motor              1.5f
#define SPEED_INTEGRAL_LIMIT_left_motor        3000 // 2160
#define SPEED_MAX_OUT_Left_motor               8000    // 16000

// 定义位置环PID参数
#define POSITION_KP_Left_motor                5.0f
#define POSITION_KI_Left_motor                0.21f
#define POSITION_KD_Left_motor                0.01f
#define POSITION_INTEGRAL_LIMIT_Left_motor    100.0f
#define POSITION_MAX_OUT_Left_motor        400.0f  // 位置环最大输出作为速度环设定

// 输出轴分辨率：
// 电机轴：44 计数/转
// 输出轴：44 × 减速比30 = 1320 计数/转
// 角度分辨率：360°/1320 ≈ 0.27°/计数
// 编码器计数计算
#define ENCODER_COUNTS_PER_MOTOR_REV  44 // (HALL_ENCODER_LINES * HALL_QUADRATURE_FACTOR)  // 电机轴每转计数 = 44
#define ENCODER_COUNTS_PER_OUTPUT_REV 1320 //(ENCODER_COUNTS_PER_MOTOR_REV * MOTOR_GEAR_RATIO) // 输出轴每转计数 = 1320
// 角度分辨率计算
#define DEGREES_PER_REV           360.0f  // 每转角度
#define OUTPUT_ANGLE_RESOLUTION   0.2727f // (DEGREES_PER_REV / ENCODER_COUNTS_PER_OUTPUT_REV) // ≈0.2727°/计数
// 弧度分辨率（可选）
#define RADIANS_PER_REV           (2 * 3.1415926535f)  // 每转弧度
#define OUTPUT_RAD_RESOLUTION     0.00476f // (RADIANS_PER_REV / ENCODER_COUNTS_PER_OUTPUT_REV) // ≈0.00476 rad/计数

//#define POSITION_DEADZONE 0.273f // 位置环死区，单位：度
#define POSITION_DEADZONE 0.5f // 位置环死区，单位：度
// 电机控制参数
typedef struct
{
    volatile float left_speed;
    volatile float left_position;
    volatile float left_position_error;
    volatile float right_speed;
    volatile float right_position;
    volatile float right_position_error;
} GB37Motor_feedback_t;

// 新增：位置信息结构
typedef struct {
    float current_position;
    float target_position;
    float position_error;
    float target_speed;  // 位置环输出作为速度环输入
} MotorPosition_target_t;


typedef struct {
    pid_obj_t *left_speed;
    pid_obj_t *left_position;
    pid_obj_t *right_speed;
    pid_obj_t *right_position;
} MotorPIDs_t;

GB37Motor_feedback_t *getGB37Motor(void);
MotorPIDs_t *getMotorPIDs(void);

void GB37_520_Init(void);
void GB37Motor_Set_Speed(float left_speed, float right_speed);

void GB37Motor_Set_Position(float left_position_target, float right_position_target);
void GB37Motor_Set_Position_Relative(float left_position_delta, float right_position_delta);

#endif