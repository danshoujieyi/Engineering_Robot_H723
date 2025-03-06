/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2023-10-10      ChenSihan       发射模块状态机
*/

#include <stdio.h>
#include <string.h>
#include "cmd_task.h"
#include "rm_config.h"
#include "rm_task.h"
#include "stm32h7xx_hal.h"
#include "rc_sbus.h"
#include "ramp.h"
#include "drv_dwt.h"
#include "filter32.h"
#include "usart.h"

#define BSP_USING_RC_SBUS

uint8_t usart5_rx_buffer_index = 0;  // 当前使用的接收缓冲区
uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART5;
extern uint16_t usart5_rec_size;

extern struct chassis_cmd_msg chassis_cmd;

/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
static void remote_to_cmd_sbus(void);

extern sbus_data_t sbus_data[2];


void USART5_DMA_Init(void) {
    memset(usart5_rx_buffer, 0, SBUS_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收完毕后重启
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

static float cmd_dt;

static sbus_data_t *sbus_data_now = NULL;
static sbus_data_t *sbus_data_last = NULL;

void CmdTask_Entry(void const * argument)
{
    static float cmd_start;
    sbus_data_init();
    USART5_DMA_Init();
    uint8_t* active_buff = NULL;

    sbus_data_now = sbus_data;
    sbus_data_last = (sbus_data_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
    /* 初始化拨杆为上位 */
    sbus_data_now->sw1 = RC_UP;
    sbus_data_now->sw2 = RC_UP;
    sbus_data_now->sw3 = RC_UP;
    sbus_data_now->sw4 = RC_UP;

    printf("Cmd Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();
        if (xSemaphoreTake(xSemaphoreUART5, portMAX_DELAY) == pdTRUE) {
            //TODO：有一个BUG
            active_buff = usart5_rx_buffer[usart5_rx_buffer_index];  // 暂时不切换缓冲区 ^1, 有bug
            sbus_data_unpack(active_buff, usart5_rec_size);
            remote_to_cmd_sbus();
        }

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
#ifdef BSP_USING_RC_SBUS
static void remote_to_cmd_sbus(void)
{
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    *sbus_data_last = *sbus_data_now;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx = (float)sbus_data_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED ;
    chassis_cmd.vy = (float)sbus_data_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED ;
    chassis_cmd.vw = (float)sbus_data_now->ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED ;
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
    switch (sbus_data_now->sw2)
    {
    case RC_UP:
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_RESET);
        break;
    case RC_DN:
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
        break;
    }
    /* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 */
    switch(sbus_data_now->sw3)
    {
    case RC_UP:
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_RESET);
        break;

    case RC_MI:
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_RESET);
        break;
    case RC_DN:
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_1_GPIO_Port, PUMP2_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PUMP2_2_GPIO_Port, PUMP2_2_Pin, GPIO_PIN_SET);
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
//
//static void remote_to_cmd_pc_controler(void)
//{
//    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
//    *rc_last = *rc_now;
//    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
//    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);
//
//    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05f;
//
//// TODO: 目前状态机转换较为简单，有很多优化和改进空间
////遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
//    /*底盘命令*/
//    chassis_cmd.vx =  (float)rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
//    chassis_cmd.vy =  (float)rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
//    chassis_cmd.vw =  (float)rc_now->ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;
//
//    /*云台命令*/
//    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
//    {
//
//        gim_cmd.yaw +=   (float)rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW + fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
//        gim_cmd.pitch += (float)rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT- fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
//        gyro_yaw_inherit =gim_cmd.yaw;
//        gyro_pitch_inherit =gim_cmd.pitch;
//        mouse_accumulate_x=0;
//        mouse_accumulate_y=0;
//        //auto_relative_angle_status=RELATIVE_ANGLE_TRANS;
//    }
//
//
//    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
//    /*TODO:手动模式和自瞄模式状态机*/
//    if (rc_now->mouse.r==0) {
//        if (gim_cmd.last_mode == GIMBAL_RELAX)
//        {/* 判断上次状态是否为RELAX，是则先归中 */
//            gim_cmd.ctrl_mode = GIMBAL_INIT;
//            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
//        }
//        else {
//            if (gim_fdb.back_mode == BACK_IS_OK)
//            {
//                gim_cmd.ctrl_mode = GIMBAL_GYRO;
//                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
//                shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
//                memset(r_buffer_point,0,sizeof (*r_buffer_point));
//            }
//            else if(gim_fdb.back_mode==BACK_STEP)
//            {
//                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
//                gim_cmd.ctrl_mode=GIMBAL_INIT;
//            }
//        }
//    }
//
//    if (rc_now->mouse.r==1||rc_now->sw2==RC_DN)
//    {
//
//        if (gim_cmd.last_mode == GIMBAL_RELAX)
//        {/* 判断上次状态是否为RELAX，是则先归中 */
//            gim_cmd.ctrl_mode = GIMBAL_INIT;
//            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
//        }
//        else
//        {
//            if (gim_fdb.back_mode == BACK_IS_OK)
//            {/* 判断归中是否完成 */
//                gim_cmd.ctrl_mode = GIMBAL_AUTO;
//                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
//                shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
//            }
//            else if(gim_fdb.back_mode==BACK_STEP)
//            {
//                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
//                gim_cmd.ctrl_mode=GIMBAL_INIT;
//            }
//        }
//    }
//
//    /*TODO:小陀螺*/
//    if(km.e_sta==KEY_PRESS_ONCE)
//    {
//        key_e_status=-key_e_status;
//    }
//    if ( key_e_status==1||rc_now->sw1==RC_DN)
//    {
//        if (gim_fdb.back_mode==BACK_IS_OK)
//        {
//            chassis_cmd.ctrl_mode=CHASSIS_SPIN;
//        }
//        else
//        {
//            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
//            gim_cmd.ctrl_mode=GIMBAL_INIT;
//        }
//    }
//    if (chassis_cmd.ctrl_mode==CHASSIS_SPIN)
//    {
//        chassis_cmd.vw=ROTATE_RATIO;
//    }
//    /*TODO:--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
//    /*-----------------------------------------开关摩擦轮--------------------------------------------*/
//    if(km.f_sta==KEY_PRESS_ONCE)
//    {
//        key_f_status=-key_f_status;
//    }
//    if ( key_f_status==1||rc_now->sw1==RC_MI)
//    {
//        shoot_cmd.friction_status=1;
//    }
//    else
//    {
//        shoot_cmd.friction_status=0;
//    }
//    /*TODO:------------------------------------------------------------扳机连发模式---------------------------------------------------------*/
//    if((rc_now->mouse.l==1||rc_now->wheel>=200)&&shoot_cmd.friction_status==1/*&&(referee_fdb.power_heat_data.shooter_17mm_1_barrel_heat < (referee_fdb.robot_status.shooter_barrel_heat_limit-10))*/)
//    {
//        shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
//        shoot_cmd.shoot_freq=DBUS_FRICTION_AUTO_SPEED_H;
//
////        if(((int16_t)referee_fdb.robot_status.shooter_barrel_heat_limit-(int16_t)referee_fdb.power_heat_data.shooter_17mm_1_barrel_heat)<=30)
////        {
////            shoot_cmd.shoot_freq=0;
////        }
//
//        if (km.shift_sta==KEY_PRESS_LONG)
//        {
//            shoot_cmd.shoot_freq=DBUS_FRICTION_AUTO_SPEED_H;
//        }
//
//        if(referee_fdb.robot_status.shooter_barrel_heat_limit==0)
//        {
//            shoot_cmd.shoot_freq=DBUS_FRICTION_AUTO_SPEED_L;
//        }
//        else
//        {
//
//        }
//
//    }
//    else
//    {
//        shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
//        shoot_cmd.shoot_freq=0;
//    }
//    /*-------------------------------------------------------------堵弹反转检测------------------------------------------------------------*/
//    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
//    {
//        shoot_cmd.ctrl_mode=SHOOT_REVERSE;
//        if (reverse_cnt<120)
//            reverse_cnt++;
//        else
//            reverse_cnt=0;
//    }
//    /*-----------------------------------------------------------舵机开盖关盖--------------------------------------------------------------*/
//    if(rc_now->kb.bit.R==1||rc_now->wheel<=-200)
//    {
//        shoot_cmd.cover_open=1;
//    }
//    else
//    {
//        shoot_cmd.cover_open=0;
//    }
//
//    /*-----------------------------------------使能判断--------------------------------------------*/
//    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
//    {
//        memset(r_buffer_point,0,sizeof (*r_buffer_point));
//    }
//    if (rc_now->sw2==RC_UP)
//    {
//        gim_cmd.ctrl_mode = GIMBAL_RELAX;
//        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
//        shoot_cmd.ctrl_mode=SHOOT_STOP;
//        /*放开状态下，gim不接收值*/
//        gim_cmd.pitch=0;
//        gim_cmd.yaw=0;
//        gyro_yaw_inherit=0;
//        gyro_pitch_inherit=0;
//        /*摩擦轮停止*/
//        shoot_cmd.friction_status=0;
//        /*键盘按键状态标志位*/
//        key_e_status=-1;
//        key_f_status=-1;
//        memset(r_buffer_point,0,sizeof (*r_buffer_point));
//    }
//
//}


#endif
