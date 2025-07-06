#include "GB37_520_333rpm.h"
#include "pwm_motor.h"
#include "encoder.h"

#include "stdio.h"

#include "tim_dwt.h"

static pid_obj_t *left_speed_pid;
static pid_obj_t *right_speed_pid;
static pid_obj_t *left_position_pid;
static pid_obj_t *right_position_pid;

 Encoder_Speed_t left_encoder;
static Encoder_Speed_t right_encoder;

static GB37Motor_feedback_t GB37_520_Motor = {0, 0, 0, 0};
static MotorPIDs_t result;

static MotorPosition_target_t left_position;
static MotorPosition_target_t right_position;

static volatile float left_measure_speed;
static volatile float right_measure_speed;

static Motor_Config_t motor_left = 
{
    .htim_pwm = PWM_MOTOR_INST,
    .pwm_channel = GPIO_PWM_MOTOR_C0_IDX,
    .ain1_port = TB6612_PIN_AIN1_PORT,
    .ain1_pin = TB6612_PIN_AIN1_PIN,
    .ain2_port = TB6612_PIN_AIN2_PORT,
    .ain2_pin = TB6612_PIN_AIN2_PIN,
};

static Motor_Config_t motor_right = 
{
    .htim_pwm = PWM_MOTOR_INST,
    .pwm_channel = GPIO_PWM_MOTOR_C1_IDX,
    .ain1_port = TB6612_PIN_BIN1_PORT,
    .ain1_pin = TB6612_PIN_BIN1_PIN,
    .ain2_port = TB6612_PIN_BIN2_PORT,
    .ain2_pin = TB6612_PIN_BIN2_PIN,
};

void GB37_520_Init(void)
{
   Motor_GPIO_Init(&motor_left);
   Motor_GPIO_Init(&motor_right);
   encoder_init(&left_encoder, &right_encoder);

    pid_config_t left_speed_config = INIT_PID_CONFIG(SPEED_KP_Left_motor, SPEED_KI_Left_motor, SPEED_KD_Left_motor, SPEED_INTEGRAL_LIMIT_left_motor, SPEED_MAX_OUT_Left_motor,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
        // 初始化位置环PID
    pid_config_t left_position_config = INIT_PID_CONFIG(
        POSITION_KP_Left_motor, 
        POSITION_KI_Left_motor, 
        POSITION_KD_Left_motor, 
        POSITION_INTEGRAL_LIMIT_Left_motor, 
        POSITION_MAX_OUT_Left_motor,
        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement)
    );
        // 初始化位置信息
    left_position.current_position = 0.0f;
    left_position.target_position = 0.0f;
    left_position.target_speed = 0.0f;
    
    right_position.current_position = 0.0f;
    right_position.target_position = 0.0f;
    right_position.target_speed = 0.0f;

    left_position_pid = pid_register(&left_position_config);
    right_position_pid = pid_register(&left_position_config);

    left_speed_pid = pid_register(&left_speed_config);
    right_speed_pid = pid_register(&left_speed_config);      

        //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_Encoder_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_Encoder_INST_INT_IRQN);
                                              
}



void GB37Motor_Set_Speed(float left_speed, float right_speed)
{
    encoder_update(&left_encoder, &right_encoder);
	calculate_speed_M_method(&left_encoder);
    // // 获取当前速度
    left_measure_speed = left_encoder.filtered_speed_rpm;
    right_measure_speed = right_encoder.filtered_speed_rpm;


  //  printf("left_measure_speed:%f,right_measure_speed:%f\n",left_measure_speed,right_measure_speed);
  //      printf("\n");
    float speed_L = pid_calculate(left_speed_pid, left_measure_speed, left_speed);
    float speed_R = pid_calculate(right_speed_pid, right_measure_speed, right_speed);
  //  printf("speed_L:%f,speed_R:%f\n",speed_L,speed_R);
   // printf("\n");
     Motor_Set_Speed(&motor_left, speed_L);
     Motor_Set_Speed(&motor_right, speed_R);

}

// 新增：位置控制函数
void GB37Motor_Set_Position(float left_position_target, float right_position_target)
{
    // 更新目标位置
    left_position.target_position = left_position_target;
    right_position.target_position = right_position_target;
    
    // 更新编码器数据
    encoder_update(&left_encoder, &right_encoder);
    calculate_speed_M_method(&left_encoder);
    
    // 更新当前位置（假设编码器提供位置信息）
    left_position.current_position = left_encoder.filtered_angle_deg;
    right_position.current_position = right_encoder.filtered_angle_deg;
    
    // 位置误差计算
    left_position.position_error = left_position.target_position - left_position.current_position;
    right_position.position_error = right_position.target_position - right_position.current_position;
    
    // 位置环计算（输出作为速度环的目标）
    if (fabs(left_position.position_error) <= POSITION_DEADZONE) {
        left_position.position_error = 0;
        left_position.target_speed = 0; // 直接设为0，停止运动
		left_position_pid->Iout = 0;
    }
    
    if (fabs(right_position.position_error) <= POSITION_DEADZONE) {
        right_position.position_error = 0;
        right_position.target_speed = 0; // 直接设为0，停止运动
		right_position_pid->Iout = 0;
    }
    
    // 位置环计算（输出作为速度环的目标）
    if (left_position.position_error != 0) { // 仅在误差超出死区时计算PID
        left_position.target_speed = pid_calculate(left_position_pid, left_position.current_position, left_position.target_position);
    }
    
    if (right_position.position_error != 0) { // 仅在误差超出死区时计算PID
        right_position.target_speed = pid_calculate(right_position_pid, right_position.current_position, right_position.target_position);
    }
    // 调用速度控制（速度环）
    GB37Motor_Set_Speed(left_position.target_speed, right_position.target_speed);
}

// 新增：位置增量控制
void GB37Motor_Set_Position_Relative(float left_position_delta, float right_position_delta)
{
    GB37Motor_Set_Position(
        left_position.current_position + left_position_delta,
        right_position.current_position + right_position_delta
    );
}

GB37Motor_feedback_t *getGB37Motor(void) {
    GB37_520_Motor.left_speed = left_measure_speed;
    GB37_520_Motor.right_speed = right_measure_speed;
    GB37_520_Motor.left_position = left_position.current_position;
    GB37_520_Motor.right_position = right_position.current_position;
    GB37_520_Motor.left_position_error = left_position.position_error;
    GB37_520_Motor.right_position_error = right_position.position_error;
    return &GB37_520_Motor;
}

// 返回两个PID指针的函数
MotorPIDs_t *getMotorPIDs(void) {
    
    result.left_speed = left_speed_pid;
    result.right_speed = right_speed_pid;
    result.left_position = left_position_pid;
    result.right_position = right_position_pid;
    return &result;
}



//定时器的中断服务函数 已配置为10ms的周期
void TIMER_Encoder_INST_IRQHandler(void)
{

}