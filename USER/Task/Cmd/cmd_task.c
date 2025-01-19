/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2023-10-10      ChenSihan       发射模块状态机
*/

#include <stdio.h>
#include "cmd_task.h"
#include "rm_config.h"
#include "rm_task.h"
#include "stm32h7xx_hal.h"
#include "rc_sbus.h"
#include "ramp.h"
#include "drv_dwt.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#define BSP_USING_RC_SBUS

extern struct chassis_cmd_msg chassis_cmd;
static struct ins_msg ins_data;

#ifdef BSP_USING_RC_DBUS
static rc_dbus_obj_t *rc_now, *rc_last;
#endif
#ifdef BSP_USING_RC_SBUS
static rc_obj_t *rc_now, *rc_last;
#endif

/*发射停止标志位*/
static int trigger_flag=0;
/*堵转电流反转记次*/
static int reverse_cnt;
static float gyro_yaw_inherit;
static float gyro_pitch_inherit;

static int pitch_cnt=0;
/*判断上位机角度是否更新数值*/
static gim_auto_judge yaw_auto;
static gim_auto_judge pitch_auto;
/*键盘按键状态标志位*/
static int key_e_status=-1;
static int key_f_status=-1;
static int key_q_status=-1;
static int key_v_status=-1;
/*用于清除环形缓冲区buffer的指针*/
extern uint8_t *r_buffer_point;
/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
static void remote_to_cmd_sbus(void);
static void remote_to_cmd_dbus(void);
static void remote_to_cmd_pc_controler(void);
//TODO: 添加图传链路的自定义控制器控制方式和键鼠控制方式
/*键盘加速度的斜坡*/
ramp_obj_t *km_vx_ramp;//x轴控制斜坡
ramp_obj_t *km_vy_ramp;//y周控制斜坡
/*自瞄鼠标累计操作值*/
static float mouse_accumulate_x=0;
static float mouse_accumulate_y=0;
/*储存鼠标坐标数据*/
// First_Order_Filter_t mouse_y_lpf,mouse_x_lpf;
float Ballistic;  //对自瞄数据进行手动鼠标弹道补偿
/* --------------------------------- cmd线程入口 -------------------------------- */
static float cmd_dt;

extern uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
extern rc_obj_t rc_obj[2];


void CmdTask_Entry(void const * argument)
{
    static float cmd_start;

#ifdef BSP_USING_RC_DBUS
    rc_now = dbus_rc_init();
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
    First_Order_Filter_Init(&mouse_x_lpf,0.014,0.1);
    First_Order_Filter_Init(&mouse_y_lpf,0.014,0.1);
    /* 初始化拨杆为上位 */
    rc_now->sw1 = RC_UP;
    rc_now->sw2 = RC_UP;
#endif
#ifdef BSP_USING_RC_SBUS
    rc_now = rc_obj;
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
    /* 初始化拨杆为上位 */
    rc_now->sw1 = RC_UP;
    rc_now->sw2 = RC_UP;
    rc_now->sw3 = RC_UP;
    rc_now->sw4 = RC_UP;
#endif
    /* 键盘控制急停斜坡的注册*/
    km_vx_ramp = ramp_register(100, 2500000);
    km_vy_ramp = ramp_register(100, 2500000);
    printf("Cmd Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();

        /* 将遥控器原始数据转换为控制指令 */
#ifdef BSP_USING_RC_SBUS

            remote_to_cmd_sbus();

#endif/* BSP_USING_RC_SBUS */
#ifdef BSP_USING_RC_DBUS
        #ifdef BSP_USING_RC_KEYBOARD
            PC_Handle_kb();//处理PC端键鼠控制
            #endif
            #ifdef WHEEL_OMNI_SENTRY
            #endif/* WHEEL_OMNI_SENTRY */
            #ifdef WHEEL_OMNI_INFANTRY
            remote_to_cmd_pc_controler();
            #endif/* WHEEL_OMNI_INFANTRY */
#endif/* BSP_USING_RC_DBUS */



        /* 用于调试监测线程调度使用 */
        cmd_dt = dwt_get_time_ms() - cmd_start;
        if (cmd_dt > 1)
            printf("Cmd Task is being DELAY! dt = [%f]", &cmd_dt);

        vTaskDelay(1);
    }
}

/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
/**
 * @brief 将遥控器数据转换为控制指令
 */
#ifdef WHEEL_OMNI_SENTRY
struct dbus_mode_flag_t
{
    uint8_t chassis_flag;
    uint8_t shoot_flag;
};
static int cnt_patrol;
static struct dbus_mode_flag_t dbus_mode_flag;
static void remote_to_cmd_dbus(void)
{
    /* 保存上一次数据 */
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);
    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx = rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy = rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis_cmd.vw = rc_now->ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;
    /*chassis_cmd.vx=trans_fdb.line_x*1000;
    chassis_cmd.vy=trans_fdb.line_y*1000;
    chassis_cmd.vw=-trans_fdb.angle_z;*/
    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;
    /*云台命令*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        gim_cmd.yaw += rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW + fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gim_cmd.pitch += rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT - fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit =gim_cmd.yaw;
        mouse_accumulate_x=0;
        mouse_accumulate_y=0;

    }
    if (gim_cmd.ctrl_mode==GIMBAL_AUTO)
    {
        mouse_accumulate_x+=fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        mouse_accumulate_y-=fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gim_cmd.yaw = trans_fdb.yaw + gyro_yaw_inherit + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW + mouse_accumulate_x;
        gim_cmd.pitch = trans_fdb.pitch + 100* rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT + mouse_accumulate_y;

    }
    /* 限制云台角度 */

    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    /*开环状态和遥控器归中*/
    if (gim_cmd.ctrl_mode==GIMBAL_INIT||gim_cmd.ctrl_mode==GIMBAL_RELAX)
    {
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;

    }
    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
    // 左拨杆sw2为上时，底盘和云台均RELAX；为中时，云台为GYRO；为下时，云台为AUTO。
    // 右拨杆sw1为上时，底盘为FOLLOW；为中时，底盘为OPEN；为下时，底盘为SPIN。
    /* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 */
    /*右拨杆三种模式：停止 手动 自动*/
    switch(rc_now->sw2)
    {
    case RC_UP:
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode=SHOOT_STOP;
        /*放开状态下，gim不接收值*/
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
        gyro_yaw_inherit=0;
        break;
    case RC_MI:
        if(gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
        else
        {
            if(gim_fdb.back_mode == BACK_IS_OK)
            {
                gim_cmd.ctrl_mode = GIMBAL_GYRO;
                chassis_cmd.ctrl_mode=CHASSIS_FOLLOW_GIMBAL;
            }
            else chassis_cmd.ctrl_mode=CHASSIS_OPEN_LOOP;
        }

        break;
    case RC_DN:
        if(gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
        else
        {
            if(gim_fdb.back_mode == BACK_IS_OK)
            {/* 判断归中是否完成 */
                gim_cmd.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd.ctrl_mode=CHASSIS_FOLLOW_GIMBAL;
            }
        }
           /* chassis_cmd.ctrl_mode=CHASSIS_OPEN_LOOP;
            chassis_cmd.vx=trans_fdb.line_x*1000;
            chassis_cmd.vy=trans_fdb.line_y*1000;
            chassis_cmd.vw=-trans_fdb.angle_z;*/
        break;
    }

    /*左拨杆计数器*/
    /*摩擦轮开关*/
    if (rc_now->sw1==RC_UP)
    {
        shoot_cmd.friction_status=0;
    }
    else if(rc_now->sw1==RC_MI)/*跟随模式和小陀螺模式切换*/
    {
        shoot_cmd.friction_status=1;
    }
    if (rc_now->sw1==RC_DN)
    {
        chassis_cmd.ctrl_mode=CHASSIS_SPIN;
        shoot_cmd.friction_status=1;
    }

    if (chassis_cmd.ctrl_mode==CHASSIS_SPIN)
    {
        chassis_cmd.vw=3;
    }
    /*--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    /*单发/三联模式发射判断*/
    if (rc_now->wheel>=-640)
        trigger_flag = 1;
    if (rc_now->wheel<=-640 && trigger_flag)
        {//判断是否要开火
                    shoot_cmd.trigger_status = TRIGGER_ON;
                    trigger_flag = 0;
                    shoot_cmd.ctrl_mode=SHOOT_ONE;
        }
    else
    {
        shoot_cmd.trigger_status = TRIGGER_OFF;
    }
     /*连发模式*/
    if(rc_now->wheel>=50)
    {
        shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
         shoot_cmd.shoot_freq=(int16_t)(50+rc_now->wheel*5);
    }
    /*堵弹反转检测*/
    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
    {
        shoot_cmd.ctrl_mode=SHOOT_REVERSE;
        if (reverse_cnt<120)
            reverse_cnt++;
        else
            reverse_cnt=0;
    }
    /*舵机开盖关盖*/
    if(rc_now->sw1==RC_UP&&rc_now->wheel<=-640)
    {
         shoot_cmd.cover_open=1;
    }
    else
{
         shoot_cmd.cover_open=0;
    }
}
#endif
#ifdef BSP_USING_RC_DBUS
#ifdef WHEEL_OMNI_INFANTRY
static void remote_to_cmd_pc_controler(void)
{
    /* 保存上一次数据 */
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);

    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05f;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx =  (float)rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy =  (float)rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis_cmd.vw =  (float)rc_now->ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;

    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;
    /*云台命令*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {

        gim_cmd.yaw +=   (float)rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW + fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gim_cmd.pitch += (float)rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT- fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit =gim_cmd.yaw;
        gyro_pitch_inherit =gim_cmd.pitch;
        mouse_accumulate_x=0;
        mouse_accumulate_y=0;
        //auto_relative_angle_status=RELATIVE_ANGLE_TRANS;
    }
    if (gim_cmd.ctrl_mode==GIMBAL_AUTO)
    {
//        if (auto_relative_angle_status==RELATIVE_ANGLE_TRANS)
//        {
//            trans_fdb.yaw=0;
//            trans_fdb.pitch=0;
//        }
//        mouse_accumulate_x+=fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        mouse_accumulate_y-=fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gim_cmd.yaw = trans_fdb.yaw+gyro_yaw_inherit + mouse_accumulate_x/* + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW*/;//上位机自瞄
        gim_cmd.pitch = trans_fdb.pitch+gyro_pitch_inherit + mouse_accumulate_y/* +100 * rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT */;//上位机自瞄
    }
    /* 限制云台角度 */

    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    /*开环状态和遥控器归中*/
    if (gim_cmd.ctrl_mode==GIMBAL_INIT||gim_cmd.ctrl_mode==GIMBAL_RELAX)
    {
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
    }
    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
    /*TODO:手动模式和自瞄模式状态机*/
    if (rc_now->mouse.r==0) {
        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
        }
        else {
            if (gim_fdb.back_mode == BACK_IS_OK)
            {
                gim_cmd.ctrl_mode = GIMBAL_GYRO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
                memset(r_buffer_point,0,sizeof (*r_buffer_point));
            }
            else if(gim_fdb.back_mode==BACK_STEP)
            {
                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
                gim_cmd.ctrl_mode=GIMBAL_INIT;
            }
        }
    }

    if (rc_now->mouse.r==1||rc_now->sw2==RC_DN)
    {

        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
        }
        else
        {
            if (gim_fdb.back_mode == BACK_IS_OK)
            {/* 判断归中是否完成 */
                gim_cmd.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
            }
            else if(gim_fdb.back_mode==BACK_STEP)
            {
                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
                gim_cmd.ctrl_mode=GIMBAL_INIT;
            }
        }
    }

    /*TODO:小陀螺*/
    if(km.e_sta==KEY_PRESS_ONCE)
    {
        key_e_status=-key_e_status;
    }
    if ( key_e_status==1||rc_now->sw1==RC_DN)
    {
        if (gim_fdb.back_mode==BACK_IS_OK)
        {
            chassis_cmd.ctrl_mode=CHASSIS_SPIN;
        }
        else
        {
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
            gim_cmd.ctrl_mode=GIMBAL_INIT;
        }
    }
    if (chassis_cmd.ctrl_mode==CHASSIS_SPIN)
    {
        chassis_cmd.vw=ROTATE_RATIO;
    }
    /*TODO:--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    /*-----------------------------------------开关摩擦轮--------------------------------------------*/
    if(km.f_sta==KEY_PRESS_ONCE)
    {
        key_f_status=-key_f_status;
    }
    if ( key_f_status==1||rc_now->sw1==RC_MI)
    {
        shoot_cmd.friction_status=1;
    }
    else
    {
        shoot_cmd.friction_status=0;
    }
    /*TODO:------------------------------------------------------------扳机连发模式---------------------------------------------------------*/
    if((rc_now->mouse.l==1||rc_now->wheel>=200)&&shoot_cmd.friction_status==1/*&&(referee_fdb.power_heat_data.shooter_17mm_1_barrel_heat < (referee_fdb.robot_status.shooter_barrel_heat_limit-10))*/)
    {
        shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
        shoot_cmd.shoot_freq=DBUS_FRICTION_AUTO_SPEED_H;

//        if(((int16_t)referee_fdb.robot_status.shooter_barrel_heat_limit-(int16_t)referee_fdb.power_heat_data.shooter_17mm_1_barrel_heat)<=30)
//        {
//            shoot_cmd.shoot_freq=0;
//        }

        if (km.shift_sta==KEY_PRESS_LONG)
        {
            shoot_cmd.shoot_freq=DBUS_FRICTION_AUTO_SPEED_H;
        }

        if(referee_fdb.robot_status.shooter_barrel_heat_limit==0)
        {
            shoot_cmd.shoot_freq=DBUS_FRICTION_AUTO_SPEED_L;
        }
        else
        {

        }

    }
    else
    {
        shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
        shoot_cmd.shoot_freq=0;
    }
    /*-------------------------------------------------------------堵弹反转检测------------------------------------------------------------*/
    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
    {
        shoot_cmd.ctrl_mode=SHOOT_REVERSE;
        if (reverse_cnt<120)
            reverse_cnt++;
        else
            reverse_cnt=0;
    }
    /*-----------------------------------------------------------舵机开盖关盖--------------------------------------------------------------*/
    if(rc_now->kb.bit.R==1||rc_now->wheel<=-200)
    {
        shoot_cmd.cover_open=1;
    }
    else
    {
        shoot_cmd.cover_open=0;
    }

    /*-----------------------------------------使能判断--------------------------------------------*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        memset(r_buffer_point,0,sizeof (*r_buffer_point));
    }
    if (rc_now->sw2==RC_UP)
    {
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode=SHOOT_STOP;
        /*放开状态下，gim不接收值*/
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
        gyro_yaw_inherit=0;
        gyro_pitch_inherit=0;
        /*摩擦轮停止*/
        shoot_cmd.friction_status=0;
        /*键盘按键状态标志位*/
        key_e_status=-1;
        key_f_status=-1;
        memset(r_buffer_point,0,sizeof (*r_buffer_point));
    }

}
#ifdef WHEEL_OMNI_SENTRY
#endif
#endif/* WHEEL_OMNI_INFANTRY */
#endif/* BSP_USING_RC_DBUS */
/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
/**
 * @brief 将遥控器数据转换为控制指令
 */
#ifdef BSP_USING_RC_SBUS
static void remote_to_cmd_sbus(void)
{
    //gim_cmd.last_mode = gim_cmd.ctrl_mode;
   // chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
   // shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx = rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED ;
    chassis_cmd.vy = rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED ;
    chassis_cmd.vw = rc_now->ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED ;
//    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;
//    /*云台命令*/
//    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
//    {
//        gim_cmd.yaw += rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW ;
//        gim_cmd.pitch += rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT ;
//        gyro_yaw_inherit =gim_cmd.yaw;
//        gyro_pitch_inherit =ins_data.pitch;
//    }
//    if (gim_cmd.ctrl_mode==GIMBAL_AUTO) {
//
//        gim_cmd.yaw = trans_fdb.yaw + gyro_yaw_inherit + 150 * rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW;//上位机自瞄
//        gim_cmd.pitch = trans_fdb.pitch + 100* rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT ;//上位机自瞄
//
//    }
//    /* 限制云台角度 */
//    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
//
//    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
//    // 左拨杆sw2为上时，底盘和云台均RELAX；为中时，云台为GYRO；为下时，云台为AUTO。
//    // 右拨杆sw1为上时，底盘为FOLLOW；为中时，底盘为OPEN；为下时，底盘为SPIN。
//    if (gim_cmd.ctrl_mode==GIMBAL_INIT||gim_cmd.ctrl_mode==GIMBAL_RELAX)
//    {
//        gim_cmd.pitch=0;
//        gim_cmd.yaw=0;
//
//    }
    switch (rc_now->sw2)
    {
    case RC_UP:
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_RESET);
        break;
////    case RC_MI:
////        chassis_cmd.ctrl_mode = CHASSIS_OPEN_LOOP;
////        break;
    case RC_DN:
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
        break;
    }
    /* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 */
    switch(rc_now->sw3)
    {
    case RC_UP:
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_RESET);
        break;

    case RC_MI:
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_RESET);
        break;
    case RC_DN:
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_SET);
        break;
    }
//    /*--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
//
//    if(rc_now->sw3!=RC_UP&&gim_cmd.ctrl_mode!=GIMBAL_AUTO)//判断总开关是否停止发射
//    {
//        switch (rc_now->sw1)
//        {
//            /*判断是否处于可发射状态*/
//            //TODO:由于遥控器拨杆档位限制,目前连发模式还未写进状态机。两档拨杆具体值由遥控器确定，现在待定。
//            case RC_DN:
//                if (rc_now->ch6 <= 775)
//                    trigger_flag = 1;
//                if (rc_now->ch6 >= 775 && trigger_flag)
//                {//判断是否要开火
//                    shoot_cmd.trigger_status = TRIGGER_ON;
//                    trigger_flag = 0;
//                }
//                else shoot_cmd.trigger_status = TRIGGER_OFF;
//                /*判断发射模式是三连发还是全自动*/
//                switch (rc_now->sw4)
//                {
//                    case RC_UP:
//                        shoot_cmd.ctrl_mode = SHOOT_ONE;
//                        break;
//                    case RC_DN:
//                        shoot_cmd.ctrl_mode = SHOOT_COUNTINUE;
//                        break;
//                }
//                break;
//            case RC_UP:
//                shoot_cmd.ctrl_mode = SHOOT_STOP;
//                shoot_cmd.trigger_status = TRIGGER_OFF;
//                break;
//        }
//    }
//    else if (gim_cmd.ctrl_mode==GIMBAL_AUTO&&rc_now->sw4==RC_DN)
//    {
//        shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
//    }
//    else
//    {
//        shoot_cmd.ctrl_mode==SHOOT_STOP;
//    }
//    /*堵弹反转检测*/
//    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
//    {
//        shoot_cmd.ctrl_mode=SHOOT_REVERSE;
//        if (reverse_cnt<120)
//            reverse_cnt++;
//        else
//            reverse_cnt=0;
//    }
//
//    // TODO: 添加弹频和弹速控制
//    if (rc_now->ch6>0)
//    {
//        shoot_cmd.shoot_freq = rc_now->ch6 / RC_MAX_VALUE*10;//连发模式拨弹电机转速
//    }
//    else if(rc_now->ch6<=-775)
//    {
//         shoot_cmd.cover_open=1;
//    }
//    else
//    {
//         shoot_cmd.cover_open=0;
//    }
}
#endif/* WHEEL_OMNI_INFANTRY */
#ifdef WHEEL_OMNI_SENTRY
#endif
