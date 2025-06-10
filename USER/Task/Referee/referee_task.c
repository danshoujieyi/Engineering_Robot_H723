/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   �������㷨�����̣߳��������㷨�������������߳��м����������
  ******************************************************************************
  * @attention
  *
  * ��������ѭGPLv3��ԴЭ�飬����ѧϰ����ʹ��
  * δ����ɲ���������ҵ��;
  *
  ******************************************************************************
  */
#include "referee_task.h"
#include "cmsis_os.h"
#include "referee_system.h"
#include "drv_dwt.h"
#include "dj_motor.h"

/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
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
/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
/* -------------------------------- ���Լ���߳���� --------------------------------- */
static uint32_t referee_task_dwt = 0;   // ������
static float referee_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float referee_task_delta = 0;    // ����߳�����ʱ��
static float referee_task_start_dt = 0; // ����߳̿�ʼʱ��
/* -------------------------------- ���Լ���߳���� --------------------------------- */


/* -------------------------------- �߳���� ------------------------------- */
void RefereeTask_Entry(void const * argument)
{
/* -------------------------------- �����ʼ������ ------------------------------- */
    referee_system_init();
/* -------------------------------- �����ʼ������ ------------------------------- */

/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    referee_task_dt = dwt_get_delta(&referee_task_dwt);
    referee_task_start_dt = dwt_get_time_ms();
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    for(;;)
    {
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
        referee_task_delta = dwt_get_time_ms() - referee_task_start_dt;
        referee_task_start_dt = dwt_get_time_ms();
        referee_task_dt = dwt_get_delta(&referee_task_dwt);
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */

/* -------------------------------- �̴߳����д���� ------------------------------- */


/* -------------------------------- �̴߳����д���� ------------------------------- */

/* -------------------------------- �̷߳���Topics��Ϣ ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- �̷߳���Topics��Ϣ ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- �߳̽��� ------------------------------- */

/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
///**
// * @brief chassis �߳������з����߳�ʼ��
// */
//static void chassis_pub_init(void)
//{
//    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
//}
//
///**
// * @brief chassis �߳������ж����߳�ʼ��
// */
//static void chassis_sub_init(void)
//{
//    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
//    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
//    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
//}
//
///**
// * @brief chassis �߳������з��������͸��»���
// */
//static void chassis_pub_push(void)
//{
//    pub_push_msg(pub_chassis,&chassis_fdb);
//}
///**
// * @brief chassis �߳������ж����߻�ȡ���»���
// */
//static void chassis_sub_pull(void)
//{
//    sub_get_msg(sub_cmd, &chassis_cmd);
//    sub_get_msg(sub_trans, &trans_fdb);
//    sub_get_msg(sub_ins, &ins_data);
//}
/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */