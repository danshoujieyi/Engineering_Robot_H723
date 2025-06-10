//
// Created by 刘嘉俊 on 25-4-21.
//

#ifndef CTRBOARD_H7_ALL_ROBOT_H
#define CTRBOARD_H7_ALL_ROBOT_H

#define CPU_FREQUENCY 480     /* CPU主频(mHZ) */

#include "stm32h7xx_hal.h" // 使用的芯片
#include "cmsis_os.h" // 使用的 OS 头文件

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free vPortFree
#else
#define user_malloc malloc
#define user_malloc free
#endif

/* 底盘和云台分别对应的 can 总线 */
#define CAN_CHASSIS    hfdcan1
#define CAN_GIMBAL     hfdcan2
#define CAN_CHASSIS_NAME    "hfdcan1"
#define CAN_GIMBAL_NAME     "hfdcan2"
/* 磁力计所挂载的 i2c 设备名称(软件i2c) */
#define I2C_MAG        "i2c1"    //"Notice: PA8 --> 8; PC9 --> 41"

/* 陀螺仪所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_GYRO       "spi1"
#define SPI_GYRO_CS    16
/* 加速度计所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_ACC        "spi1"
#define SPI_ACC_CS     4

/* 遥控器所挂载的 usart 设备名称 */
#define USART_RC       "uart3"

/* ---------------------------------- 福斯遥控器相关 --------------------------------- */
#define RC_MAX_VALUE      784.0f  /* 遥控器通道最大值 */
/* 遥控器模式下的底盘最大速度比例 */ // TODO：此时表示遥控器输出满值，也只能到达1.0f m/s的速度
/* 底盘前进速度比例(m/s) */
#define CHASSIS_RC_MOVE_RATIO_X 2.0f
/* 底盘平移速度比例(m/s) */
#define CHASSIS_RC_MOVE_RATIO_Y 2.0f
/* 底盘旋转速度比例(rad/s) */
#define CHASSIS_RC_MOVE_RATIO_W 5.0f

/* 鼠标键盘模式下的底盘最大速度比例 */  //TODO：取决于键盘手感
#define CHASSIS_PC_MOVE_RATIO_X 1.0f
/* 底盘平移速度比例(m/s) */
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f
/* 底盘旋转速度比例(rad/s) */
#define CHASSIS_PC_MOVE_RATIO_W 1.0f

/* ---------------------------------- 底盘相关 ---------------------------------- */
/* 底盘轮子半径(m) */
#define WHEEL_RADIUS       0.0762f
/* 底盘轮距(m) 左右轮中心距 */
#define WHEEL_TRACK        0.380f
/* 底盘轴距(m) 前后轮中心距 */
#define WHEEL_BASE         0.456f
/* 底盘轮子安装角度(度) */
#define WHEEL_ANGLE 45.0f   // 轮子安装角度（度，常见45度）
/* 底盘轮子周长(m) */
#define WHEEL_PERIMETER   0.4788f
/* 底盘控制周期（ms） */
#define CHASSIS_PERIOD     1.0f //(ms)
/* 底盘3508电机减速比 */
#define CHASSIS_MOTOR_REDUCTION_RATIO 19.2032f    // 3591/187，输入与输出之比，输入转动19圈输出转动1圈
/* 底盘电机转子转换系数 */
#define WHELL_RPM_RATIO 2406.416f // (60.0f * CHASSIS_MOTOR_REDUCTION_RATIO / WHEEL_PERIMETER)

/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是m/s */
#define MAX_CHASSIS_VX_SPEED 2
#define MAX_CHASSIS_VY_SPEED 2

#define MAX_CHASSIS_VX_SPEED_HIGH 11
#define MAX_CHASSIS_VY_SPEED_HIGH 11

#define MAX_CHASSIS_VX_SPEED_LOW 5
#define MAX_CHASSIS_VY_SPEED_LOW 5

/* 底盘旋转最大速度，单位是rad/s */
#define MAX_CHASSIS_VW_SPEED 5.0f

/* --------------------------------- 底盘单个电机PID参数 -------------------------------- */
/* 电机速度环（实际是C620的电流环）（电流值范围：-16380~0~16380）（-20A~0A~20A） */
// 传入电机转速测量值（转子的，不含减速机），通过预期转子转速，计算对应的预期电流值。
// 相当于期望编码器电机速度与编码器测量得到的电机速度，用PID计算所需PWM值。
#define CHASSIS_KP_V_MOTOR              3.5f
#define CHASSIS_KI_V_MOTOR              0.01
#define CHASSIS_KD_V_MOTOR              0.001
#define CHASSIS_INTEGRAL_V_MOTOR        1500
#define CHASSIS_MAX_V_MOTOR             12000    // 16000

/* 电机角度环 */



// --------------------- 里程计控制参数宏定义 ---------------------
// 目标点定义
#define WAYPOINT_A_X          0.0f    // 点A坐标X（米）
#define WAYPOINT_B_X          2.0f    // 点B坐标X（米）
#define POSITION_TOLERANCE    0.02f   // 到达目标点的位置误差容忍度（米）

// 位置控制PID参数（外环）
#define POSITION_KP           1.5f    // 比例系数
#define POSITION_KI           0.05f   // 积分系数
#define POSITION_KD           0.2f    // 微分系数
#define POSITION_MAX_OUTPUT   (CHASSIS_MAX_V_MOTOR * 0.8f)  // 限制外环输出速度

// 航向角控制PID参数
#define YAW_KP                0.8f    // 航向比例系数
#define YAW_KI                0.0f    // 航向积分系数
#define YAW_KD                0.1f    // 航向微分系数




/* -------------------------------- 基于IMU的底盘数电机PID参数 ------------------------------- */
/* imu速度环 */
#define YAW_KP_V_IMU             5000
#define YAW_KI_V_IMU             200
#define YAW_KD_V_IMU             10
#define YAW_INTEGRAL_V_IMU       1000
#define YAW_MAX_V_IMU            30000
/* imu角度环 */
#define YAW_KP_A_IMU             0.35f
#define YAW_KI_A_IMU             0
#define YAW_KD_A_IMU             0.001f
#define YAW_INTEGRAL_A_IMU       5
#define YAW_MAX_A_IMU            25
/* auto速度环 */
#define YAW_KP_V_AUTO            0
#define YAW_KI_V_AUTO            0
#define YAW_KD_V_AUTO            0
#define YAW_INTEGRAL_V_AUTO      0
#define YAW_MAX_V_AUTO           0
/* auto角度环 */
#define YAW_KP_A_AUTO            0
#define YAW_KI_A_AUTO            0
#define YAW_KD_A_AUTO            0
#define YAW_INTEGRAL_A_AUTO      0
#define YAW_MAX_A_AUTO           0

/* PITCH轴电机PID参数 */
/* imu速度环 */
#define PITCH_KP_V_IMU           4250
#define PITCH_KI_V_IMU           1000
#define PITCH_KD_V_IMU           3
#define PITCH_INTEGRAL_V_IMU     1500
#define PITCH_MAX_V_IMU          20000
/* imu角度环 */
#define PITCH_KP_A_IMU           0.5f
#define PITCH_KI_A_IMU           0.0f
#define PITCH_KD_A_IMU           0.005f
#define PITCH_INTEGRAL_A_IMU     0.2f
#define PITCH_MAX_A_IMU          20
/* auto速度环 */
#define PITCH_KP_V_AUTO          0
#define PITCH_KI_V_AUTO          0
#define PITCH_KD_V_AUTO          0
#define PITCH_INTEGRAL_V_AUTO    0
#define PITCH_MAX_V_AUTO         0
/* auto角度环 */
#define PITCH_KP_A_AUTO          0
#define PITCH_KI_A_AUTO          0
#define PITCH_KD_A_AUTO          0
#define PITCH_INTEGRAL_A_AUTO    0
#define PITCH_MAX_A_AUTO         0


void robot_init(void);

#endif //CTRBOARD_H7_ALL_ROBOT_H
