//
// Created by 刘嘉俊 on 25-2-25.
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

// 定义常用的宏
#define NDEBUG
#define USE_DYNAMIC_MEMORY //!< 使用系统的 malloc/free 函数

#include "stm32h7xx_hal.h"

#define FIFO_ENTER_CRITICAL __disable_irq  // 进入临界区
#define FIFO_EXIT_CRITICAL __enable_irq   // 退出临界区
#define FIFO_GET_CPU_SR __get_PRIMASK     // 获取 CPU 状态
#define FIFO_RESTORE_CPU_SR(CPU_SR) __set_PRIMASK(CPU_SR)  // 恢复 CPU 状态
#define FIFO_CPU_SR_TYPE register unsigned long  // 定义 CPU 状态类型

// FIFO 单元内存模型 (字节模式)
typedef struct
{
    char *p_start_addr;  //!< FIFO 内存池起始地址
    char *p_end_addr;    //!< FIFO 内存池结束地址
    int free_num;        //!< FIFO 剩余容量
    int used_num;        //!< FIFO 中的元素数量
    char *p_read_addr;   //!< FIFO 数据读取索引指针
    char *p_write_addr;  //!< FIFO 数据写入索引指针
} fifo_s_t;

// FIFO 内存模型
typedef struct
{
    char *p_start_addr;  //!< FIFO 内存池起始地址
    char *p_end_addr;    //!< FIFO 内存池结束地址
    int free_num;        //!< FIFO 剩余容量
    int used_num;        //!< FIFO 中的元素数量
    int unit_size;       //!< FIFO 元素大小（单位：字节）
    char *p_read_addr;   //!< FIFO 数据读取索引指针
    char *p_write_addr;  //!< FIFO 数据写入索引指针
} fifo_t;

#ifdef USE_DYNAMIC_MEMORY

// 创建新的 FIFO 实例（单元模式）
fifo_s_t *fifo_s_create(int uint_cnt);

// 销毁 FIFO 实例（单元模式）
void fifo_s_destroy(fifo_s_t *p_fifo);

#endif // USE_DYNAMIC_MEMORY

// 初始化静态 FIFO 结构（单元模式）
int fifo_s_init(fifo_s_t *p_fifo, void *p_base_addr, int uint_cnt);

// 向 FIFO 中插入一个元素（单元模式）
int fifo_s_put(fifo_s_t *p_fifo, char element);

// 向 FIFO 中插入多个元素（单元模式）
int fifo_s_puts(fifo_s_t *p_fifo, char *p_source, int len);

// 从 FIFO 中获取一个元素（单元模式）
char fifo_s_get(fifo_s_t *p_fifo);

// 从 FIFO 中获取多个元素（单元模式）
int fifo_s_gets(fifo_s_t *p_fifo, char *p_dest, int len);

// 从 FIFO 中预读取一个元素（单元模式）
char fifo_s_preread(fifo_s_t *p_fifo, int offset);

// 检查 FIFO 是否为空（单元模式）
char fifo_s_isempty(fifo_s_t *p_fifo);

// 检查 FIFO 是否已满（单元模式）
char fifo_s_isfull(fifo_s_t *p_fifo);

// 获取 FIFO 中的元素数量（单元模式）
int fifo_s_used(fifo_s_t *p_fifo);

// 获取 FIFO 中剩余的容量（单元模式）
int fifo_s_free(fifo_s_t *p_fifo);

// 清空 FIFO 内容（单元模式）
void fifo_s_flush(fifo_s_t *p_fifo);

// 丢弃 FIFO 中的元素（单元模式）
int fifo_s_discard(fifo_s_t *p_fifo, int len);

// 创建新的 FIFO 实例（通用模式）
fifo_t *fifo_create(char unit_size, int unit_cnt);

// 销毁 FIFO 实例（通用模式）
void fifo_destory(fifo_t *p_fifo);

// 初始化静态 FIFO 结构（通用模式）
int fifo_init(fifo_t *p_fifo, void *p_base_addr, char unit_size, int unit_cnt);

// 向 FIFO 中插入一个元素（通用模式）
int fifo_put(fifo_t *p_fifo, void *p_element);

// 向 FIFO 中插入一个元素（不保护中断，通用模式）
int fifo_put_noprotect(fifo_t *p_fifo, void *p_element);

// 从 FIFO 中获取一个元素（通用模式）
int fifo_get(fifo_t *p_fifo, void *p_element);

// 从 FIFO 中获取一个元素（不保护中断，通用模式）
int fifo_get_noprotect(fifo_t *p_fifo, void *p_element);

// 预读取 FIFO 中的元素（通用模式）
int fifo_pre_read(fifo_t *p_fifo, char offset, void *p_element);

// 检查 FIFO 是否为空（通用模式）
int fifo_is_empty(fifo_t *p_fifo);

// 检查 FIFO 是否已满（通用模式）
int fifo_is_full(fifo_t *p_fifo);

// 获取 FIFO 中的元素数量（通用模式）
int fifo_used(fifo_t *p_fifo);

// 获取 FIFO 中剩余的容量（通用模式）
int fifo_free(fifo_t *p_fifo);

// 清空 FIFO 内容（通用模式）
int fifo_flush(fifo_t *p_fifo);

#ifdef __cplusplus
}
#endif

#endif //CTRBOARD_H7_ALL_FIFO_H
