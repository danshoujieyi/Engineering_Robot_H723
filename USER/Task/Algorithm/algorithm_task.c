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
#include <stdio.h>
#include "algorithm_task.h"
#include "cmsis_os.h"
#include "KalmanFilterOne.h"
#include "drv_dwt.h"

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
static uint32_t algorithm_task_dwt = 0;   // 毫秒监测
static float algorithm_task_dt = 0;       // 线程实际运行时间dt
static float algorithm_task_delta = 0;    // 监测线程运行时间
static float algorithm_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

static mat_type_t filtered_data[NUM_JOINTS];
static float angles[7] = {0}; // 从队列中读取的角度值（度数）

extern QueueHandle_t xKalmanOneQueue;
extern QueueHandle_t xControlQueue; // 队列句柄


/* -------------------------------- 线程入口 ------------------------------- */
void AlgorithmTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    Init_KalmanFiltersOne(KALMAN_F, KALMAN_H, KALMAN_Q, KALMAN_R);
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();
        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
//        if (xQueueReceive(xKalmanOneQueue, angles, 0) == pdTRUE) {
//            // 对接收的数据进行滤波处理
//            KalmanFilterOne_Data(angles, filtered_data);
//            xQueueSend(xControlQueue, filtered_data, 0);
//        }
//printf("AlgorithmTask_Entry: algorithm_task_dt = %f\n", algorithm_task_dt);
//printf("AlgorithmTask_Entry: algorithm_task_delta = %f\n", algorithm_task_delta);
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(100);
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