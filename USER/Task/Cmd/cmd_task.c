/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2023-10-10      ChenSihan       发射模块状态机
*/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "cmd_task.h"
#include "rm_config.h"
#include "rm_task.h"
#include "stm32h7xx_hal.h"
#include "rc_sbus.h"
#include "ramp.h"
#include "drv_dwt.h"
#include "filter32.h"
#include "usart.h"
#include "keyboard.h"
#include "user_lib.h"
#include "DMmotor_task.h"
#include "pump.h"

//volatile uint8_t usart5_rx_buffer_index = 0;      // 当前使用的接收缓冲区索引
//volatile uint16_t usart5_rx_size = 0;               // 本次接收到的数据长度
//uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];      // 双缓冲区
//extern SemaphoreHandle_t xSemaphoreUART5;           // 通知任务处理信号量
//
extern sbus_data_t sbus_data_fdb;
extern keyboard_control_t keyboard;
static pc_control_t pc_data;

extern struct referee_fdb_msg referee_fdb;

extern struct chassis_cmd_msg chassis_cmd;
extern SemaphoreHandle_t sbus_cmd_mutex;

/* 外部变量声明 */
/*键盘加速度的斜坡*/
ramp_obj_t *km_vx_ramp = NULL;;//x轴控制斜坡
ramp_obj_t *km_vy_ramp = NULL;//y周控制斜坡
ramp_obj_t *km_vw_ramp = NULL;//y周控制斜坡
/* 气泵控制状态 */
static uint8_t pump_state = 0;

/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
//void remote_to_cmd_sbus(void);


//void USART5_DMA_Init(void) {
//    memset(usart5_rx_buffer, 0, SBUS_RX_BUF_SIZE);
//    // 关闭DMA的传输过半中断，仅保留完成中断
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收完毕后重启
//    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
//}

static void keyboard_to_chassis_cmd(void);
void CmdTask_Entry(void const * argument)
{
//    sbus_data_init();
//    USART5_DMA_Init();
//    uint8_t finishedBuffer;
//
//    /* 初始化拨杆为上位 */
//    sbus_data_fdb.sw1 = RC_UP;
//    sbus_data_fdb.sw2 = RC_UP;
//    sbus_data_fdb.sw3 = RC_UP;
//    sbus_data_fdb.sw4 = RC_UP;
    km_vx_ramp = ramp_register(0, 200); //2500000
    km_vy_ramp = ramp_register(0, 200);  // 0 -2的累加次数
    km_vw_ramp = ramp_register(0, 200);

    /* 获取原始键盘数据 */

    memset(&pc_data, 0, sizeof(pc_control_t));
    memset(&keyboard, 0, sizeof(keyboard_control_t));

    for (;;)
    {

//        if (xSemaphoreTake(xSemaphoreUART5, pdMS_TO_TICKS(10)) == pdTRUE) {
//            /* 使用刚完成接收数据的缓冲区 */
//            finishedBuffer = usart5_rx_buffer_index ^ 1;
//            sbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);
//            memset(usart5_rx_buffer[finishedBuffer], 0, SBUS_RX_BUF_SIZE);
//            remote_to_cmd_sbus();
//
//        }
        pc_data = convert_remote_to_pc(&referee_fdb.remote_control);
        PC_keyboard_mouse(&pc_data);
        chassis_cmd_state_machine();
        pum_ctrl();
        arm_cmd_state_machine(); // 机械臂状态机
        keyboard_to_chassis_cmd();
        //        cmd_sbus_keyboard();


        vTaskDelay(1);
    }
}

/**
  * @brief 将键盘数据转换为底盘控制命令
  */
static void keyboard_to_chassis_cmd(void)
{
    /* 模式状态保存 */
    static uint8_t last_ctrl_mode = 0;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;

    /* 底盘速度命令生成 */
    chassis_cmd.vx = keyboard.vx * CHASSIS_PC_MOVE_RATIO_X + sbus_data_fdb.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED;
    chassis_cmd.vy = keyboard.vy * CHASSIS_PC_MOVE_RATIO_Y + sbus_data_fdb.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED;
    chassis_cmd.vw = keyboard.vw * 1.0f + sbus_data_fdb.ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED ;

}

extern pump_mode_e pump_mode;
/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
/**
 * @brief 将遥控器数据转换为控制指令
 */
void remote_to_cmd_sbus(void)
{
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    xSemaphoreTake(sbus_cmd_mutex, portMAX_DELAY);
    sbus_data_t tmp_data = sbus_data_fdb;  // 复制到临时变量
    xSemaphoreGive(sbus_cmd_mutex);

    /*底盘命令*/
    chassis_cmd.vx = tmp_data.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED + keyboard.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy = tmp_data.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED + keyboard.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis_cmd.vw = tmp_data.ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED + keyboard.vw * 1.0f;

    if (tmp_data.sw3 == RC_MI)
    {
        pump_mode = PUMP_CLOSE;
    }
    else if (tmp_data.sw3 == RC_DN)
    {
        pump_mode = PUMP_OPEN;
    }
}

void pum_ctrl(void)
{
    if (pump_mode == PUMP_OPEN)
    {
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
    }
    else if (pump_mode == PUMP_CLOSE)
    {
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_RESET);
    }
}

extern struct arm_cmd_msg arm_cmd;

void cmd_sbus_keyboard(void)
{
    //chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    xSemaphoreTake(sbus_cmd_mutex, portMAX_DELAY);
    sbus_data_t tmp_data = sbus_data_fdb;  // 复制到临时变量
    xSemaphoreGive(sbus_cmd_mutex);

    /* 模式状态保存 */
    static uint8_t last_ctrl_mode = 0;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;

    /*底盘命令*/
    chassis_cmd.vx = tmp_data.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED + keyboard.vx * CHASSIS_PC_MOVE_RATIO_X;;
    chassis_cmd.vy = tmp_data.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED + keyboard.vy * CHASSIS_PC_MOVE_RATIO_Y;;
    chassis_cmd.vw = tmp_data.ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED + keyboard.vw * 1.0f;

    // 合并 ARM 控制模式和遥控器控制模式
    if (arm_cmd.ctrl_mode == ARM_ENABLE || tmp_data.sw1 == RC_DN)
    {
        arm_cmd_enable();
        dm_motor_enable(&hfdcan3, &motor[Motor1]);
        for(int i=1; i<6; i++)
        {
            dm_motor_enable(&hfdcan2, &motor[i]);
        }
    }
    else if (arm_cmd.ctrl_mode == ARM_DISABLE || tmp_data.sw1 == RC_UP)
    {
        arm_cmd_disable();
        dm_motor_disable(&hfdcan3, &motor[Motor1]);
        for(int i=1; i<6; i++)
        {
            dm_motor_disable(&hfdcan2, &motor[i]);
        }
    }

    // 合并键盘和遥控器的输入来决定是否启用或禁用泵
    bool pump_up = (keyboard.x.state == KEY_PRESS_ONCE) || (tmp_data.sw2 == RC_UP);
    bool pump_dn = (keyboard.z.state == KEY_PRESS_ONCE) || (tmp_data.sw2 == RC_DN);

    if (pump_up) {
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_RESET);
    } else if (pump_dn) {
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
    }

}



