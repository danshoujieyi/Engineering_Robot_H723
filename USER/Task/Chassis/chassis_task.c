//
// Created by 刘嘉俊 on 25-1-2.
//

#include <string.h>
#include "chassis_task.h"
#include "PID.h"
#include "dj_motor.h"
#include "stdio.h"
#include "bsp_fdcan.h"
#include "rm_config.h"
#include "drv_dwt.h"
#include "user_lib.h"
#include "motor_def.h"
#include "rm_task.h"
#include "referee_system.h"

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
struct chassis_cmd_msg chassis_cmd;

extern struct referee_fdb_msg referee_fdb;

static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
}chassis_controller[4];

dji_motor_object_t *chassis_motor[4];  // 底盘电机实例
static int16_t motor_ref[4]; // 电机控制期望值

static void chassis_motor_init();
static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = mecanum_calc;


/* --------------------------------- 电机控制相关 --------------------------------- */
#define CURRENT_POWER_LIMIT_RATE 80
static int16_t motor_control_0(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    static int16_t chassis_max_current=0;
    static int16_t chassis_power_limit=0;
//    /*传参给局部变量防止被更改抽风*/
//    chassis_power_limit=(int16_t)referee_fdb.robot_status.chassis_power_limit;
//    /*底盘功率限制防止buffer溢出*/
//    if(chassis_power_limit>=120)
//    {
//        chassis_power_limit=120;
//    }
//    if(referee_fdb.power_heat_data.buffer_energy<20)
//    {
//        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE*(referee_fdb.power_heat_data.buffer_energy/50);
//    }
//    else
//    {
//        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE;
//    }
    if (chassis_power_limit==0)
    {
        chassis_max_current=8000;
    }
    set =(int16_t) pid_calculate(chassis_controller[0].speed_pid, measure.speed_rpm, motor_ref[0]);
    VAL_LIMIT(set , -chassis_max_current, chassis_max_current);
    return set;
}

static int16_t motor_control_1(dji_motor_measure_t measure)
{

    static int16_t set = 0;
    static int16_t chassis_max_current=0;
    static int16_t chassis_power_limit=0;
//    /*传参给局部变量防止被更改抽风*/
//    chassis_power_limit=(int16_t)referee_fdb.robot_status.chassis_power_limit;
//    /*底盘功率限制防止buffer溢出*/
//    if(chassis_power_limit>=120)
//    {
//        chassis_power_limit=120;
//    }
//    if(referee_fdb.power_heat_data.buffer_energy<20)
//    {
//        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE*(referee_fdb.power_heat_data.buffer_energy/50);
//    }
//    else
//    {
//        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE;
//    }
    if (chassis_power_limit==0)
    {
        chassis_max_current=8000;
    }
    set =(int16_t) pid_calculate(chassis_controller[1].speed_pid, measure.speed_rpm, motor_ref[1]);
    VAL_LIMIT(set , -chassis_max_current, chassis_max_current);
    return set;
}

static int16_t motor_control_2(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    static int16_t chassis_max_current=0;
    static int16_t chassis_power_limit=0;
//    /*传参给局部变量防止被更改抽风*/
//    chassis_power_limit=(int16_t)referee_fdb.robot_status.chassis_power_limit;
//    /*底盘功率限制防止buffer溢出*/
//    if(chassis_power_limit>=120)
//    {
//        chassis_power_limit=120;
//    }
//    if(referee_fdb.power_heat_data.buffer_energy<20)
//    {
//        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE*(referee_fdb.power_heat_data.buffer_energy/50);
//    }
//    else
//    {
//        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE;
//    }
    if (chassis_power_limit==0)
    {
        chassis_max_current=8000;
    }
    set =(int16_t) pid_calculate(chassis_controller[2].speed_pid, measure.speed_rpm, motor_ref[2]);
    VAL_LIMIT(set , -chassis_max_current, chassis_max_current);
    return set;
}

static int16_t motor_control_3(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    static int16_t chassis_max_current=0;
    static int16_t chassis_power_limit=0;
//    /*传参给局部变量防止被更改抽风*/
//    chassis_power_limit=(int16_t)referee_fdb.robot_status.chassis_power_limit;
//    /*底盘功率限制防止buffer溢出*/
//    if(chassis_power_limit>=120)
//    {
//        chassis_power_limit=120;
//    }
//    if(referee_fdb.power_heat_data.buffer_energy<20)
//    {
//        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE*(referee_fdb.power_heat_data.buffer_energy/50);
//    }
//    else
//    {
//        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE;
//    }
    if (chassis_power_limit==0)
    {
        chassis_max_current=8000;
    }
    set =(int16_t) pid_calculate(chassis_controller[3].speed_pid, measure.speed_rpm, motor_ref[3]);
    VAL_LIMIT(set , -chassis_max_current, chassis_max_current);
    return set;
}

/* 底盘每个电机对应的控制函数 */
static void *motor_control[4] ={motor_control_0,motor_control_1,motor_control_2,motor_control_3};

// TODO：将参数都放到配置文件中，通过宏定义进行替换
motor_config_t chassis_motor_config[4] =
        {
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS_NAME,
                        .rx_id = 0x201,
                        .tx_id = 0x201,
                        .controller = &chassis_controller[0],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS_NAME,
                        .rx_id = 0x202,
                        .tx_id = 0x202,
                        .controller = &chassis_controller[1],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS_NAME,
                        .rx_id = 0x203,
                        .tx_id = 0x203,
                        .controller = &chassis_controller[2],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS_NAME,
                        .rx_id = 0x204,
                        .tx_id = 0x204,
                        .controller = &chassis_controller[3],
                }
        };


/**
 * @brief 注册底盘电机及其控制器初始化
 */
static void chassis_motor_init()
{
    pid_config_t chassis_speed_config = INIT_PID_CONFIG(CHASSIS_KP_V_MOTOR, CHASSIS_KI_V_MOTOR, CHASSIS_KD_V_MOTOR, CHASSIS_INTEGRAL_V_MOTOR, CHASSIS_MAX_V_MOTOR,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_controller[i].speed_pid = pid_register(&chassis_speed_config);
        chassis_motor[i] = dji_motor_register(&chassis_motor_config[i], motor_control[i]);
    }
}

static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    static float wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);
    int16_t wheel_rpm[4];

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //rad/s

    wheel_rpm[0] = ( cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = ( cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//forward
    wheel_rpm[2] = (-cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//right
    wheel_rpm[3] = (-cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//back

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}


/* USER CODE END Header_ChassisTask_Entry */
void ChassisTask_Entry(void const * argument)
{

    chassis_motor_init();
    bsp_can_init();
    can_filter_init();

    for(;;)
    {

        chassis_calc_moto_speed(&chassis_cmd, motor_ref);
        dji_motor_control();



        vTaskDelay(1);
    }
    /* USER CODE END ChassisTask_Entry */
}
