/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2023-10-10      ChenSihan       发射模块状态机
*/

#include <stdio.h>
#include <string.h>
#include "cmd_task.h"
#include "robot_config.h"
#include "rm_task.h"
#include "stm32h7xx_hal.h"
#include "rc_sbus.h"
#include "ramp.h"
#include "drv_dwt.h"
#include "usart.h"
#include "keyboard.h"
#include "DMmotor_task.h"
#include "pump.h"

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
static uint32_t cmd_task_dwt = 0;   // 毫秒监测
static float cmd_task_dt = 0;       // 线程实际运行时间dt
static float cmd_task_delta = 0;    // 监测线程运行时间
static float cmd_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

extern sbus_data_t sbus_data_fdb;
extern keyboard_control_t keyboard;
static pc_control_t pc_data;

extern struct referee_fdb_msg referee_fdb;

extern struct chassis_cmd_msg chassis_cmd;


/* 外部变量声明 */
/*键盘加速度的斜坡*/
ramp_obj_t *km_vx_ramp = NULL;;//x轴控制斜坡
ramp_obj_t *km_vy_ramp = NULL;//y周控制斜坡
ramp_obj_t *km_vw_ramp = NULL;//y周控制斜坡
/* 气泵控制状态 */
static uint8_t pump_state = 0;


/* -------------------------------- 线程入口 ------------------------------- */
void CmdTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    sbus_data_init();
    sbus_data_fdb.sw1 = RC_UP;
    sbus_data_fdb.sw2 = RC_UP;
    sbus_data_fdb.sw3 = RC_UP;
    sbus_data_fdb.sw4 = RC_UP;
    km_vx_ramp = ramp_register(0, 200); //2500000
    km_vy_ramp = ramp_register(0, 200);  // 0 -2的累加次数
    km_vw_ramp = ramp_register(0, 200);
    /* 获取原始键盘数据 */
    memset(&pc_data, 0, sizeof(pc_control_t));
    memset(&keyboard, 0, sizeof(keyboard_control_t));
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    cmd_task_dt = dwt_get_delta(&cmd_task_dwt);
    cmd_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        cmd_task_delta = dwt_get_time_ms() - cmd_task_start_dt;
        cmd_task_start_dt = dwt_get_time_ms();
        cmd_task_dt = dwt_get_delta(&cmd_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        pc_data = convert_remote_to_pc(&referee_fdb.remote_control);
        PC_keyboard_mouse(&pc_data);
        chassis_cmd_state_machine();
        pum_ctrl();
        arm_cmd_state_machine(); // 机械臂状态机
        remote_to_cmd_sbus();
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

float text_vx = 0;
extern pump_mode_e pump_mode;
/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
/**
 * @brief 将遥控数据转换为控制指令（包含键盘与福斯遥控器数据）
 */
void remote_to_cmd_sbus(void)
{
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;

    /*底盘命令*/
//    chassis_cmd.vx = tmp_data.ch4 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED + keyboard.vx * CHASSIS_PC_MOVE_RATIO_X;
//    chassis_cmd.vy = tmp_data.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED + keyboard.vy * CHASSIS_PC_MOVE_RATIO_Y;
//    chassis_cmd.vw = tmp_data.ch1 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED + keyboard.vw * 1.0f;

    // TODO:右手系，逆时针为正。遥控器部分为美国手(左倾斜为正，上抬头为正，左转为正）
    chassis_cmd.vx = (sbus_data_fdb.ch2 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE  + keyboard.vx * CHASSIS_PC_MOVE_RATIO_Y );
    chassis_cmd.vy = (sbus_data_fdb.ch4 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE  + keyboard.vy * CHASSIS_PC_MOVE_RATIO_X );
    chassis_cmd.vw = (sbus_data_fdb.ch1 * CHASSIS_RC_MOVE_RATIO_W / RC_MAX_VALUE  + keyboard.vw * CHASSIS_PC_MOVE_RATIO_W );
    //chassis_cmd.vx = text_vx;

    if (sbus_data_fdb.sw3 == RC_MI)
    {
        pump_mode = PUMP_CLOSE;
    }
    else if (sbus_data_fdb.sw3 == RC_DN)
    {
        pump_mode = PUMP_OPEN;
    }
}

//void pum_ctrl(void)
//{
//    if (pump_mode == PUMP_OPEN)
//    {
//        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
//    }
//    else if (pump_mode == PUMP_CLOSE)
//    {
//        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_RESET);
//    }
//}

void pum_ctrl(void)
{
    if (pump_mode == PUMP_OPEN)
    {
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_SET);
    }
    else if (pump_mode == PUMP_CLOSE)
    {
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_RESET);
    }
}
