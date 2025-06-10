/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */
#include <string.h>
#include "chassis_task.h"
#include "PID.h"
#include "dj_motor.h"
#include "stdio.h"
#include "bsp_fdcan.h"
#include "robot.h"
#include "drv_dwt.h"
#include "user_lib.h"
#include "motor_def.h"
#include "rm_task.h"
#include "referee_system.h"

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
//static struct chassis_cmd_msg chassis_cmd;
//static struct chassis_fdb_msg chassis_fdb;
//static struct trans_fdb_msg trans_fdb;
//static struct ins_msg ins_data;
//
//static publisher_t *pub_chassis;
//static subscriber_t *sub_cmd,*sub_ins,*sub_trans;
//
//static void chassis_pub_init(void);
//static void chassis_sub_init(void);
//static void chassis_pub_push(void);
//static void chassis_sub_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t chassis_task_dwt = 0;   // 毫秒监测
static float chassis_task_dt = 0;       // 线程实际运行时间dt
static float chassis_task_delta = 0;    // 监测线程运行时间
static float chassis_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

struct chassis_cmd_msg chassis_cmd;

extern struct referee_fdb_msg referee_fdb;

static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
}chassis_controller[4];

static dji_motor_object_t *chassis_motor[4];

static int16_t motor_target_speed_rpm[4];

static void chassis_motor_init();
static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);


/* --------------------------------- 电机控制相关 --------------------------------- */
#define CURRENT_POWER_LIMIT_RATE 80
static int16_t motor_control_0(dji_motor_measure_t measure)
{
    static int16_t motor_current_set = 0;
    static int16_t motor_max_current=0;   // 电流值范围：-16380~0~16380
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
        motor_max_current = 8000;
    }
    motor_current_set =(int16_t) pid_calculate(chassis_controller[0].speed_pid, measure.speed_rpm, motor_target_speed_rpm[0]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int16_t motor_control_1(dji_motor_measure_t measure)
{
    static int16_t motor_current_set = 0;
    static int16_t motor_max_current = 0;   // 电流值范围：-16380~0~16380,电流值限幅变量
    static int16_t chassis_power_limit = 0;
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
        motor_max_current = 8000;
    }
    motor_current_set =(int16_t) pid_calculate(chassis_controller[1].speed_pid, measure.speed_rpm, motor_target_speed_rpm[1]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int16_t motor_control_2(dji_motor_measure_t measure)
{
    static int16_t motor_current_set = 0;
    static int16_t motor_max_current = 0;   // 电流值范围：-16380~0~16380
    static int16_t chassis_power_limit = 0;
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
        motor_max_current = 8000;
    }
    motor_current_set =(int16_t) pid_calculate(chassis_controller[2].speed_pid, measure.speed_rpm, motor_target_speed_rpm[2]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int16_t motor_control_3(dji_motor_measure_t measure)
{
    static int16_t motor_current_set = 0;
    static int16_t motor_max_current = 0;   // 电流值范围：-16380~0~16380
    static int16_t chassis_power_limit = 0;
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
        motor_max_current = 8000;
    }
    motor_current_set =(int16_t) pid_calculate(chassis_controller[3].speed_pid, measure.speed_rpm, motor_target_speed_rpm[3]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
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

        chassis_cmd.ctrl_mode = CHASSIS_ENABLE;
        chassis_cmd.last_mode = CHASSIS_ENABLE;
    }
}

static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    // 轮子转速转换系数，转为轮子转速，为rpm/min，每分钟多少转,60是指60秒，转换成分钟，
    // 空载转速482rpm，3Nm满载最高转速469rpm
    // static float wheel_rpm_ratio = 60.0f * CHASSIS_MOTOR_REDUCTION_RATIO / WHEEL_PERIMETER ;
    int16_t wheel_rpm[4];  // 转换电机转子的期望转速，而非实际输出轮子的转速

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //m/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //m/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);  //rad/s

    // Vw的正负取决与遥感通道是否是正的还是负数的
    // 前后运动相反，则反转vx的正负
    // 左右运动相反，则反转vy的正负
    wheel_rpm[3] = (int16_t)(( -cmd->vx - cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO);    // 左後輪
    wheel_rpm[0] = (int16_t)(( -cmd->vx + cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO);    // 左前轮
    wheel_rpm[1] = (int16_t)(( +cmd->vx + cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO);     // 右前轮
    wheel_rpm[2] = (int16_t)(( +cmd->vx - cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO);     // 右後輪

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}

// 定义结构体存储里程计数据
typedef struct {
    float x;           // X轴位移 (m)
    float y;           // Y轴位移 (m)
    float yaw;         // 航向角 (rad)
    float vx;          // X轴速度 (m/s)
    float vy;          // Y轴速度 (m/s)
    float vw;          // 角速度 (rad/s)
    uint32_t last_cyccnt; // DWT时间戳（用于积分）
} odometry_t;


void odometry_update(int16_t *wheel_rpm_actual, odometry_t *odom) {
    // 参数定义
    const float K = (WHEEL_TRACK + WHEEL_BASE) / 2.0f;
    const float ratio_inv = 1.0f / WHELL_RPM_RATIO;

    // 转换电机转速为线速度（m/s）
    float wheel_speed[4];
    for (int i = 0; i < 4; i++) {
        wheel_speed[i] = wheel_rpm_actual[i] * ratio_inv;
    }

    // 计算瞬时速度（逆解核心公式）
    odom->vx = (wheel_speed[1] + wheel_speed[2] - wheel_speed[0] - wheel_speed[3]) / 4.0f;
    odom->vy = (wheel_speed[0] + wheel_speed[1] - wheel_speed[2] - wheel_speed[3]) / 4.0f;
    odom->vw = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / (4.0f * K);

    // 使用DWT计算积分周期（自动处理32位溢出）
    float dt = dwt_get_delta(&odom->last_cyccnt); // 获取精确时间差

    // 更新位姿（带坐标系旋转）
    odom->yaw += odom->vw * dt; // 航向角积分
    const float cos_yaw = cosf(odom->yaw);
    const float sin_yaw = sinf(odom->yaw);

    // 将底盘坐标系速度转换到全局坐标系
    odom->x += (odom->vx * cos_yaw - odom->vy * sin_yaw) * dt;
    odom->y += (odom->vx * sin_yaw + odom->vy * cos_yaw) * dt;
}



void chassis_cmd_enable(void) {
    if (chassis_cmd.last_mode == CHASSIS_RELAX && chassis_cmd.ctrl_mode == CHASSIS_ENABLE)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            dji_motor_enable(chassis_motor[i]);
        }
    }
    chassis_cmd.last_mode = CHASSIS_ENABLE;
}

void chassis_cmd_disable(void) {
    if (chassis_cmd.last_mode == CHASSIS_ENABLE && chassis_cmd.ctrl_mode == CHASSIS_RELAX) {
        for (uint8_t i = 0; i < 4; i++)
        {
            dji_motor_relax(chassis_motor[i]);
        }
        chassis_cmd.last_mode = CHASSIS_RELAX;
    }
}

void chassis_cmd_state_machine(void)
{
    switch (chassis_cmd.ctrl_mode)
    {
        case CHASSIS_RELAX:
            chassis_cmd_disable();
            break;
        case CHASSIS_ENABLE:
            chassis_cmd_enable();
            break;
        case CHASSIS_STOP:
            memset(motor_target_speed_rpm, 0, sizeof(motor_target_speed_rpm));
            break;
        default:
           // chassis_cmd_disable();
            break;
    }
}

/* -------------------------------- 线程入口 ------------------------------- */
void ChassisTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    chassis_motor_init();
    bsp_can_init();
    can_filter_init();
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    chassis_task_dt = dwt_get_delta(&chassis_task_dwt);
    chassis_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        chassis_task_delta = dwt_get_time_ms() - chassis_task_start_dt;
        chassis_task_start_dt = dwt_get_time_ms();
        chassis_task_dt = dwt_get_delta(&chassis_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        mecanum_calc(&chassis_cmd, motor_target_speed_rpm);
        dji_motor_control();
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
///**
// * @brief chassis 线程中所有发布者初始化
// */
//static void chassis_pub_init(void)
//{
//    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
//}
//
///**
// * @brief chassis 线程中所有订阅者初始化
// */
//static void chassis_sub_init(void)
//{
//    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
//    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
//    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
//}
//
///**
// * @brief chassis 线程中所有发布者推送更新话题
// */
//static void chassis_pub_push(void)
//{
//    pub_push_msg(pub_chassis,&chassis_fdb);
//}
///**
// * @brief chassis 线程中所有订阅者获取更新话题
// */
//static void chassis_sub_pull(void)
//{
//    sub_get_msg(sub_cmd, &chassis_cmd);
//    sub_get_msg(sub_trans, &trans_fdb);
//    sub_get_msg(sub_ins, &ins_data);
//}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */