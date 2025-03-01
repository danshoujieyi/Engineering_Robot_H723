#include "fifo.h"

// 定义ASSERT宏，检查表达式是否为真
#ifndef ASSERT
#ifdef NDEBUG
#define ASSERT(x)
#else
#define ASSERT(x)                                     \
  do                                                  \
  {                                                   \
    if (!(x))                                         \
      printf("[assert]: %s, %d", __FILE__, __LINE__); \
    while (!(x))                                      \
      ;                                               \
  } while (0)
#endif
#endif

#ifdef USE_DYNAMIC_MEMORY

// 创建一个新的FIFO实例，并分配内存
fifo_s_t *fifo_s_create(int uint_cnt)
{
  fifo_s_t *p_fifo = NULL;
  char *p_base_addr = NULL;

  ASSERT(uint_cnt);

  // 分配FIFO控制块内存
  p_fifo = (fifo_s_t *)malloc(sizeof(fifo_s_t));
  if (NULL == p_fifo) return NULL;

  // 分配FIFO数据存储区内存
  p_base_addr = malloc(uint_cnt);
  if (NULL == p_base_addr)
  {
    free(p_fifo);
    return NULL;
  }

  // 初始化FIFO模块
  fifo_s_init(p_fifo, p_base_addr, uint_cnt);

  return p_fifo;
}

// 销毁FIFO实例，释放内存
void fifo_s_destroy(fifo_s_t *p_fifo)
{
  ASSERT(p_fifo);
  ASSERT(p_fifo->p_start_addr);

  // 释放FIFO内存
  free(p_fifo->p_start_addr);
  free(p_fifo);

  return;
}

#endif // USE_DYNAMIC_MEMORY

// 初始化静态FIFO结构
int fifo_s_init(fifo_s_t *p_fifo, void *p_base_addr, int uint_cnt)
{
    ASSERT(p_fifo);
    ASSERT(p_base_addr);
    ASSERT(uint_cnt);

    p_fifo->p_start_addr = (char *)p_base_addr;
    p_fifo->p_end_addr = (char *)p_base_addr + uint_cnt - 1;
    p_fifo->free_num = uint_cnt;
    p_fifo->used_num = 0;
    p_fifo->p_read_addr = (char *)p_base_addr;
    p_fifo->p_write_addr = (char *)p_base_addr;

    return 0;
}

// 向FIFO中插入一个元素
int fifo_s_put(fifo_s_t *p_fifo, char element)
{
    FIFO_CPU_SR_TYPE cpu_sr;

    ASSERT(p_fifo);
    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    if (0 == p_fifo->free_num)
    {
        FIFO_RESTORE_CPU_SR(cpu_sr);
        return -1; // FIFO已满
    }

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
        p_fifo->p_write_addr = p_fifo->p_start_addr;

    *(p_fifo->p_write_addr) = element;
    p_fifo->p_write_addr++;
    p_fifo->free_num--;
    p_fifo->used_num++;

    FIFO_RESTORE_CPU_SR(cpu_sr);
    return 0;
}

// 向FIFO中插入多个元素
int fifo_s_puts(fifo_s_t *p_fifo, char *p_source, int len)
{
    FIFO_CPU_SR_TYPE cpu_sr;
    int retval;
    int len_to_end;
    int len_from_start;

    ASSERT(p_fifo);

    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    if (NULL == p_source || 0 == p_fifo->free_num) goto end;

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
        p_fifo->p_write_addr = p_fifo->p_start_addr;

    len = (len < p_fifo->free_num) ? len : p_fifo->free_num;
    len_to_end = p_fifo->p_end_addr - p_fifo->p_write_addr + 1;

    if (len_to_end >= len)
    {
        len_to_end = len;
        memcpy(p_fifo->p_write_addr, p_source, len_to_end);
        p_fifo->p_write_addr += len_to_end;
    }
    else
    {
        len_from_start = len - len_to_end;
        memcpy(p_fifo->p_write_addr, p_source, len_to_end);
        memcpy(p_fifo->p_start_addr, p_source + len_to_end, len_from_start);
        p_fifo->p_write_addr = p_fifo->p_start_addr + len_from_start;
    }

    p_fifo->free_num -= len;
    p_fifo->used_num += len;
    retval = len;

    end:
    FIFO_RESTORE_CPU_SR(cpu_sr);
    return retval;
}

// 从FIFO中获取一个元素
char fifo_s_get(fifo_s_t *p_fifo)
{
    FIFO_CPU_SR_TYPE cpu_sr;
    char retval = 0;

    ASSERT(p_fifo);

    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
        p_fifo->p_read_addr = p_fifo->p_start_addr;

    retval = *p_fifo->p_read_addr;
    p_fifo->p_read_addr++;
    p_fifo->free_num++;
    p_fifo->used_num--;

    FIFO_RESTORE_CPU_SR(cpu_sr);
    return retval;
}

// 从FIFO中获取多个元素
int fifo_s_gets(fifo_s_t *p_fifo, char *p_dest, int len)
{
    FIFO_CPU_SR_TYPE cpu_sr;
    int retval;
    int len_to_end;
    int len_from_start;

    ASSERT(p_fifo);

    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    if (NULL == p_dest || 0 == p_fifo->used_num) goto end;

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
        p_fifo->p_read_addr = p_fifo->p_start_addr;

    len = (len < p_fifo->used_num) ? len : p_fifo->used_num;
    len_to_end = p_fifo->p_end_addr - p_fifo->p_read_addr + 1;

    if (len_to_end >= len)
    {
        len_to_end = len;
        memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
        p_fifo->p_read_addr += len_to_end;
    }
    else
    {
        len_from_start = len - len_to_end;
        memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
        memcpy(p_dest + len_to_end, p_fifo->p_start_addr, len_from_start);
        p_fifo->p_read_addr = p_fifo->p_start_addr + len_from_start;
    }

    p_fifo->free_num += len;
    p_fifo->used_num -= len;
    retval = len;

    end:
    FIFO_RESTORE_CPU_SR(cpu_sr);
    return retval;
}

// 检查FIFO是否为空
char fifo_s_isempty(fifo_s_t *p_fifo)
{
    ASSERT(p_fifo);
    return (p_fifo->used_num ? 0 : 1);
}

// 检查FIFO是否已满
char fifo_s_isfull(fifo_s_t *p_fifo)
{
    ASSERT(p_fifo);
    return (p_fifo->free_num ? 0 : 1);
}

// 获取FIFO中的元素个数
int fifo_s_used(fifo_s_t *p_fifo)
{
    ASSERT(p_fifo);
    return p_fifo->used_num;
}

// 获取FIFO中剩余空间的个数
int fifo_s_free(fifo_s_t *p_fifo)
{
    ASSERT(p_fifo);
    return p_fifo->free_num;
}

// 清空FIFO内容
void fifo_s_flush(fifo_s_t *p_fifo)
{
    FIFO_CPU_SR_TYPE cpu_sr;

    ASSERT(p_fifo);

    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    p_fifo->free_num = p_fifo->p_end_addr - p_fifo->p_start_addr + 1;
    p_fifo->used_num = 0;
    p_fifo->p_read_addr = p_fifo->p_start_addr;
    p_fifo->p_write_addr = p_fifo->p_start_addr;

    FIFO_RESTORE_CPU_SR(cpu_sr);
}

int fifo_s_discard(fifo_s_t *p_fifo, int len)
{
    FIFO_CPU_SR_TYPE cpu_sr;
    char *tmp_index;
    ASSERT(p_fifo);

    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    if (len > p_fifo->used_num)
        len = p_fifo->used_num;

    tmp_index = len + p_fifo->p_read_addr;
    if (tmp_index > p_fifo->p_end_addr)
        tmp_index = tmp_index - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;

    p_fifo->p_read_addr = tmp_index;
    p_fifo->free_num += len;
    p_fifo->used_num -= len;

    FIFO_RESTORE_CPU_SR(cpu_sr);
    return len;
}

#ifdef USE_DYNAMIC_MEMORY

// 创建一个新的FIFO实例，支持动态内存分配
fifo_t *fifo_create(char unit_size, int unit_cnt)
{
  fifo_t *p_fifo = NULL;
  char *p_base_addr = NULL;

  ASSERT(unit_size);
  ASSERT(unit_cnt);

  p_fifo = (fifo_t *)malloc(sizeof(fifo_t));
  if (NULL == p_fifo) return NULL;

  p_base_addr = malloc(unit_size * unit_cnt);
  if (NULL == p_base_addr)
  {
    free(p_fifo);
    return NULL;
  }

  fifo_init(p_fifo, p_base_addr, unit_size, unit_cnt);

  return p_fifo;
}

// 销毁FIFO实例，释放内存
void fifo_destory(fifo_t *p_fifo)
{
  ASSERT(p_fifo);
  ASSERT(p_fifo->p_start_addr);

  free(p_fifo->p_start_addr);
  free(p_fifo);

  return;
}

#endif // USE_DYNAMIC_MEMORY

// 初始化静态FIFO结构
int fifo_init(fifo_t *p_fifo, void *p_base_addr, char unit_size, int unit_cnt)
{
    ASSERT(p_fifo);
    ASSERT(p_base_addr);
    ASSERT(unit_size);
    ASSERT(unit_cnt);

    p_fifo->p_start_addr = (char *)p_base_addr;
    p_fifo->p_end_addr = (char *)p_base_addr + unit_size * unit_cnt - 1;
    p_fifo->free_num = unit_cnt;
    p_fifo->used_num = 0;
    p_fifo->unit_size = unit_size;
    p_fifo->p_read_addr = (char *)p_base_addr;
    p_fifo->p_write_addr = (char *)p_base_addr;

    return 0;
}

// 向FIFO中插入一个元素
int fifo_put(fifo_t *p_fifo, void *p_element)
{
    FIFO_CPU_SR_TYPE cpu_sr;
    ASSERT(p_fifo);
    ASSERT(p_element);

    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    if (0 == p_fifo->free_num)
    {
        FIFO_RESTORE_CPU_SR(cpu_sr);
        return -1; // FIFO已满
    }

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
        p_fifo->p_write_addr = p_fifo->p_start_addr;

    memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
    p_fifo->p_write_addr += p_fifo->unit_size;
    p_fifo->free_num--;
    p_fifo->used_num++;

    FIFO_RESTORE_CPU_SR(cpu_sr);
    return 0;
}

// 向FIFO中插入一个元素，不保护中断
int fifo_put_noprotect(fifo_t *p_fifo, void *p_element)
{
    ASSERT(p_fifo);
    ASSERT(p_element);

    if (0 == p_fifo->free_num) return -1; // FIFO已满

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
        p_fifo->p_write_addr = p_fifo->p_start_addr;

    memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
    p_fifo->p_write_addr += p_fifo->unit_size;
    p_fifo->free_num--;
    p_fifo->used_num++;

    return 0;
}

// 从FIFO中获取一个元素
int fifo_get(fifo_t *p_fifo, void *p_element)
{
    FIFO_CPU_SR_TYPE cpu_sr;
    ASSERT(p_fifo);
    ASSERT(p_element);

    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    if (0 == p_fifo->used_num)
    {
        FIFO_RESTORE_CPU_SR(cpu_sr);
        return -1; // FIFO为空
    }

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
        p_fifo->p_read_addr = p_fifo->p_start_addr;

    memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
    p_fifo->p_read_addr += p_fifo->unit_size;
    p_fifo->free_num++;
    p_fifo->used_num--;

    FIFO_RESTORE_CPU_SR(cpu_sr);
    return 0;
}

// 从FIFO中获取一个元素，不保护中断
int fifo_get_noprotect(fifo_t *p_fifo, void *p_element)
{
    ASSERT(p_fifo);
    ASSERT(p_element);

    if (0 == p_fifo->used_num) return -1; // FIFO为空

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
        p_fifo->p_read_addr = p_fifo->p_start_addr;

    memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
    p_fifo->p_read_addr += p_fifo->unit_size;
    p_fifo->free_num++;
    p_fifo->used_num--;

    return 0;
}

// 预读取FIFO中的元素
int fifo_pre_read(fifo_t *p_fifo, char offset, void *p_element)
{
    char *_pre_red_index;

    ASSERT(p_fifo);
    ASSERT(p_element);

    if (offset >= p_fifo->used_num)
    {
        return -1; // 越界
    }

    _pre_red_index = p_fifo->p_read_addr + p_fifo->unit_size * offset;
    while (_pre_red_index > p_fifo->p_end_addr)
    {
        _pre_red_index = _pre_red_index - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;
    }

    memcpy(p_element, _pre_red_index, p_fifo->unit_size);
    return 0;
}

// 检查FIFO是否为空
int fifo_is_empty(fifo_t *p_fifo)
{
    ASSERT(p_fifo);
    return (0 == p_fifo->used_num);
}

// 检查FIFO是否已满
int fifo_is_full(fifo_t *p_fifo)
{
    ASSERT(p_fifo);
    return (0 == p_fifo->free_num);
}

// 获取FIFO中的元素个数
int fifo_used(fifo_t *p_fifo)
{
    ASSERT(p_fifo);
    return p_fifo->used_num;
}

// 获取FIFO中剩余空间的个数
int fifo_free(fifo_t *p_fifo)
{
    ASSERT(p_fifo);
    return p_fifo->free_num;
}

// 清空FIFO内容
int fifo_flush(fifo_t *p_fifo)
{
    FIFO_CPU_SR_TYPE cpu_sr;

    ASSERT(p_fifo);

    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    p_fifo->free_num = (p_fifo->p_end_addr - p_fifo->p_start_addr) / (p_fifo->unit_size);
    p_fifo->used_num = 0;
    p_fifo->p_read_addr = p_fifo->p_start_addr;
    p_fifo->p_write_addr = p_fifo->p_start_addr;

    FIFO_RESTORE_CPU_SR(cpu_sr);
    return 0;
}
