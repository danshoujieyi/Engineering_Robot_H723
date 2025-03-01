//
// Created by ���ο� on 25-2-25.
//

#ifndef CTRBOARD_H7_ALL_FIFO_H
#define CTRBOARD_H7_ALL_FIFO_H

#ifdef __cplusplus
"C"
{
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// ���峣�õĺ�
#define NDEBUG
#define USE_DYNAMIC_MEMORY //!< ʹ��ϵͳ�� malloc/free ����

#include "stm32h7xx_hal.h"

#define FIFO_ENTER_CRITICAL __disable_irq  // �����ٽ���
#define FIFO_EXIT_CRITICAL __enable_irq   // �˳��ٽ���
#define FIFO_GET_CPU_SR __get_PRIMASK     // ��ȡ CPU ״̬
#define FIFO_RESTORE_CPU_SR(CPU_SR) __set_PRIMASK(CPU_SR)  // �ָ� CPU ״̬
#define FIFO_CPU_SR_TYPE register unsigned long  // ���� CPU ״̬����

// FIFO ��Ԫ�ڴ�ģ�� (�ֽ�ģʽ)
typedef struct
{
    char *p_start_addr;  //!< FIFO �ڴ����ʼ��ַ
    char *p_end_addr;    //!< FIFO �ڴ�ؽ�����ַ
    int free_num;        //!< FIFO ʣ������
    int used_num;        //!< FIFO �е�Ԫ������
    char *p_read_addr;   //!< FIFO ���ݶ�ȡ����ָ��
    char *p_write_addr;  //!< FIFO ����д������ָ��
} fifo_s_t;

// FIFO �ڴ�ģ��
typedef struct
{
    char *p_start_addr;  //!< FIFO �ڴ����ʼ��ַ
    char *p_end_addr;    //!< FIFO �ڴ�ؽ�����ַ
    int free_num;        //!< FIFO ʣ������
    int used_num;        //!< FIFO �е�Ԫ������
    int unit_size;       //!< FIFO Ԫ�ش�С����λ���ֽڣ�
    char *p_read_addr;   //!< FIFO ���ݶ�ȡ����ָ��
    char *p_write_addr;  //!< FIFO ����д������ָ��
} fifo_t;

#ifdef USE_DYNAMIC_MEMORY

// �����µ� FIFO ʵ������Ԫģʽ��
fifo_s_t *fifo_s_create(int uint_cnt);

// ���� FIFO ʵ������Ԫģʽ��
void fifo_s_destroy(fifo_s_t *p_fifo);

#endif // USE_DYNAMIC_MEMORY

// ��ʼ����̬ FIFO �ṹ����Ԫģʽ��
int fifo_s_init(fifo_s_t *p_fifo, void *p_base_addr, int uint_cnt);

// �� FIFO �в���һ��Ԫ�أ���Ԫģʽ��
int fifo_s_put(fifo_s_t *p_fifo, char element);

// �� FIFO �в�����Ԫ�أ���Ԫģʽ��
int fifo_s_puts(fifo_s_t *p_fifo, char *p_source, int len);

// �� FIFO �л�ȡһ��Ԫ�أ���Ԫģʽ��
char fifo_s_get(fifo_s_t *p_fifo);

// �� FIFO �л�ȡ���Ԫ�أ���Ԫģʽ��
int fifo_s_gets(fifo_s_t *p_fifo, char *p_dest, int len);

// �� FIFO ��Ԥ��ȡһ��Ԫ�أ���Ԫģʽ��
char fifo_s_preread(fifo_s_t *p_fifo, int offset);

// ��� FIFO �Ƿ�Ϊ�գ���Ԫģʽ��
char fifo_s_isempty(fifo_s_t *p_fifo);

// ��� FIFO �Ƿ���������Ԫģʽ��
char fifo_s_isfull(fifo_s_t *p_fifo);

// ��ȡ FIFO �е�Ԫ����������Ԫģʽ��
int fifo_s_used(fifo_s_t *p_fifo);

// ��ȡ FIFO ��ʣ�����������Ԫģʽ��
int fifo_s_free(fifo_s_t *p_fifo);

// ��� FIFO ���ݣ���Ԫģʽ��
void fifo_s_flush(fifo_s_t *p_fifo);

// ���� FIFO �е�Ԫ�أ���Ԫģʽ��
int fifo_s_discard(fifo_s_t *p_fifo, int len);

// �����µ� FIFO ʵ����ͨ��ģʽ��
fifo_t *fifo_create(char unit_size, int unit_cnt);

// ���� FIFO ʵ����ͨ��ģʽ��
void fifo_destory(fifo_t *p_fifo);

// ��ʼ����̬ FIFO �ṹ��ͨ��ģʽ��
int fifo_init(fifo_t *p_fifo, void *p_base_addr, char unit_size, int unit_cnt);

// �� FIFO �в���һ��Ԫ�أ�ͨ��ģʽ��
int fifo_put(fifo_t *p_fifo, void *p_element);

// �� FIFO �в���һ��Ԫ�أ��������жϣ�ͨ��ģʽ��
int fifo_put_noprotect(fifo_t *p_fifo, void *p_element);

// �� FIFO �л�ȡһ��Ԫ�أ�ͨ��ģʽ��
int fifo_get(fifo_t *p_fifo, void *p_element);

// �� FIFO �л�ȡһ��Ԫ�أ��������жϣ�ͨ��ģʽ��
int fifo_get_noprotect(fifo_t *p_fifo, void *p_element);

// Ԥ��ȡ FIFO �е�Ԫ�أ�ͨ��ģʽ��
int fifo_pre_read(fifo_t *p_fifo, char offset, void *p_element);

// ��� FIFO �Ƿ�Ϊ�գ�ͨ��ģʽ��
int fifo_is_empty(fifo_t *p_fifo);

// ��� FIFO �Ƿ�������ͨ��ģʽ��
int fifo_is_full(fifo_t *p_fifo);

// ��ȡ FIFO �е�Ԫ��������ͨ��ģʽ��
int fifo_used(fifo_t *p_fifo);

// ��ȡ FIFO ��ʣ���������ͨ��ģʽ��
int fifo_free(fifo_t *p_fifo);

// ��� FIFO ���ݣ�ͨ��ģʽ��
int fifo_flush(fifo_t *p_fifo);

#ifdef __cplusplus
}
#endif

#endif //CTRBOARD_H7_ALL_FIFO_H
