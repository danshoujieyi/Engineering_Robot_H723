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
#ifndef CTRBOARD_H7_ALL_ROBOT_CONFIG_H
#define CTRBOARD_H7_ALL_ROBOT_CONFIG_H
/******************************************
 * 1. 全局总开关（大模块级别）
 *****************************************/
#define ENABLE_C_STD_MODULE                  1   // C标准库（<stdio.h>等）
#define ENABLE_HAL_DRIVER_MODULE             1   // CubeMX生成的HAL库
#define ENABLE_USER_ALGORITHM_MODULE         1   // 用户算法模块（USER/Algorithm）
#define ENABLE_USER_MODULE_MODULE            1   // 用户外设驱动（USER/Module）
#define ENABLE_USER_ONENET_ESP8266_MODULE    1   // 物联网模块（USER/OneNet_ESP8266）
#define ENABLE_USER_TASK_MODULE              1   // 用户任务模块


/******************************************
 * 2. 子模块开关（仅在大模块启用时生效）
 *****************************************/

/* ----------------- 用户算法模块（USER/Algorithm） ----------------- */
#if ENABLE_USER_ALGORITHM_MODULE
    #define ENABLE_ALGORITHM_CRC             1   // CRC算法
    #define ENABLE_ALGORITHM_DELAY           1   // 延时算法
    #define ENABLE_ALGORITHM_DWT             1   // DWT计时
    #define ENABLE_ALGORITHM_FIFO            1   // FIFO缓冲区
    #define ENABLE_ALGORITHM_FILTER          1   // 常用滤波器
    #define ENABLE_ALGORITHM_KALMAN          1   // 多维卡尔曼滤波
    #define ENABLE_ALGORITHM_KALMAN_ONE      1   // 一维卡尔曼滤波
    #define ENABLE_ALGORITHM_MAHONY_AHRS     1   // Mahony姿态解算
    #define ENABLE_ALGORITHM_MYTYPE          1   // 自定义数据类型
    #define ENABLE_ALGORITHM_PID             1   // PID控制
    #define ENABLE_ALGORITHM_QUATERNION_EKF  1   // 四元数EKF
    #define ENABLE_ALGORITHM_RAMP            1   // 斜坡函数
    #define ENABLE_ALGORITHM_UI_ALGORITHM    1   // UI算法（如界面逻辑）
#endif


/* ----------------- 用户外设驱动（USER/Module） ----------------- */
#if ENABLE_USER_MODULE_MODULE
    #define ENABLE_MODULE_BMI088             1   // BMI088传感器
    #define ENABLE_MODULE_BOARD_BUZZER       1   // 无源蜂鸣器
    #define ENABLE_MODULE_BOARD_WS2812       1   // WS2812彩灯
    #define ENABLE_MODULE_DJ_MOTOR           1   // DJ电机驱动
    #define ENABLE_MODULE_DM_MOTOR_FDCAN     1   // DM电机（FDCAN总线）
    #define ENABLE_MODULE_FDCAN              1   // FDCAN总线驱动
    #define ENABLE_MODULE_KEYBOARD           1   // 键盘输入
    #define ENABLE_MODULE_MSG                1   // 消息协议（如自定义通信）
    #define ENABLE_MODULE_PUMP               1   // 泵控制
    #define ENABLE_MODULE_RC                 1   // 遥控器解析
    #define ENABLE_MODULE_REFEREE            1   // 裁判系统（如比赛协议）
    #define ENABLE_MODULE_UI                 1   // UI显示（如OLED、LCD）
    #define ENABLE_MODULE_VOFA               1   // Vofa+调试工具
#endif


/* ----------------- 物联网模块（OneNet_ESP8266 子模块） ----------------- */
#if ENABLE_USER_ONENET_ESP8266_MODULE
    #define ENABLE_ONENET_ADC_DRV            1   // ADC驱动
    #define ENABLE_ONENET_AHT10              1   // AHT10温湿度传感器
    #define ENABLE_ONENET_BASE64             1   // BASE64编码
    #define ENABLE_ONENET_BHT1750FUI         1   // BHT1750FUI光强传感器
    #define ENABLE_ONENET_ESP8266_CORE       1   // ESP8266核心通信
    #define ENABLE_ONENET_GP2Y1014AU         1   // GP2Y1014AU粉尘传感器
    #define ENABLE_ONENET_HMAC_SHA1          1   // HMAC_SHA1加密
    #define ENABLE_ONENET_MQ3_ALCOHOL        1   // MQ3酒精传感器
    #define ENABLE_ONENET_MQ7_CO             1   // MQ7一氧化碳传感器
    #define ENABLE_ONENET_MQTT               1   // MQTT协议
    #define ENABLE_ONENET_MQTT_SAMPLE        1   // MQTT示例
    #define ENABLE_ONENET_OLED               1   // OLED显示
    #define ENABLE_ONENET_CORE               1   // OneNet平台交互
    #define ENABLE_ONENET_S12SD              1   // S12SD传感器
    #define ENABLE_ONENET_SGP30              1   // SGP30气体传感器
#endif


/* ----------------- 用户任务模块（Task 子模块） ----------------- */
#if ENABLE_USER_TASK_MODULE
    #define ENABLE_TASK_ALGORITHM            1   // 算法任务
    #define ENABLE_TASK_CHASSIS              1   // 底盘控制任务
    #define ENABLE_TASK_CMD                  1   // 指令解析任务
    #define ENABLE_TASK_DJMOTOR              1   // DJmotor电机任务
    #define ENABLE_TASK_DMMOTOR              1   // DMmotor电机任务
    #define ENABLE_TASK_INS                  1   // 惯性导航任务
    #define ENABLE_TASK_ONENET_ESP8266       1   // 物联网任务
    #define ENABLE_TASK_REFEREE              1   // 裁判系统任务
    #define ENABLE_TASK_ROBOT_CONFIG         1   // 机器人配置任务
    #define ENABLE_TASK_TRANSMISSION         1   // 通信传输任务
    #define ENABLE_TASK_USART                1   // USART串口任务
#endif

#endif //CTRBOARD_H7_ALL_ROBOT_CONFIG_H
