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

#define BSP_USING_RC_SBUS

extern struct chassis_cmd_msg chassis_cmd;
static struct ins_msg ins_data;

#ifdef BSP_USING_RC_DBUS
static rc_dbus_obj_t *rc_now, *rc_last;
#endif
#ifdef BSP_USING_RC_SBUS
static rc_obj_t *rc_now, *rc_last;
#endif

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
/* C板预留的遥控器接口（具备取反电路）为 uart3 */
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;


void USART5_DMA_Init(void) {
    memset(sbus_rx_buf, 0, SBUS_RX_BUF_NUM);
    // 关闭DMA的传输过半中断，仅保留完成中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM*2); // 接收完毕后重启
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}


void CmdTask_Entry(void const * argument)
{
    static float cmd_start;
    USART5_DMA_Init();
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
        sbus_rc_decode( sbus_rx_buf);
        //	memset(rx_buff, 0, BUFF_SIZE);

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
    case RC_DN:
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
        break;
    }
    /* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 */
    switch(rc_now->sw3)
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

#endif
