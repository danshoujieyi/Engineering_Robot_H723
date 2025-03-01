#include "fifo.h"

// ����ASSERT�꣬�����ʽ�Ƿ�Ϊ��
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

// ����һ���µ�FIFOʵ�����������ڴ�
fifo_s_t *fifo_s_create(int uint_cnt)
{
  fifo_s_t *p_fifo = NULL;
  char *p_base_addr = NULL;

  ASSERT(uint_cnt);

  // ����FIFO���ƿ��ڴ�
  p_fifo = (fifo_s_t *)malloc(sizeof(fifo_s_t));
  if (NULL == p_fifo) return NULL;

  // ����FIFO���ݴ洢���ڴ�
  p_base_addr = malloc(uint_cnt);
  if (NULL == p_base_addr)
  {
    free(p_fifo);
    return NULL;
  }

  // ��ʼ��FIFOģ��
  fifo_s_init(p_fifo, p_base_addr, uint_cnt);

  return p_fifo;
}

// ����FIFOʵ�����ͷ��ڴ�
void fifo_s_destroy(fifo_s_t *p_fifo)
{
  ASSERT(p_fifo);
  ASSERT(p_fifo->p_start_addr);

  // �ͷ�FIFO�ڴ�
  free(p_fifo->p_start_addr);
  free(p_fifo);

  return;
}

#endif // USE_DYNAMIC_MEMORY

// ��ʼ����̬FIFO�ṹ
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

// ��FIFO�в���һ��Ԫ��
int fifo_s_put(fifo_s_t *p_fifo, char element)
{
    FIFO_CPU_SR_TYPE cpu_sr;

    ASSERT(p_fifo);
    cpu_sr = FIFO_GET_CPU_SR();
    FIFO_ENTER_CRITICAL();

    if (0 == p_fifo->free_num)
    {
        FIFO_RESTORE_CPU_SR(cpu_sr);
        return -1; // FIFO����
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

// ��FIFO�в�����Ԫ��
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

// ��FIFO�л�ȡһ��Ԫ��
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

// ��FIFO�л�ȡ���Ԫ��
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

// ���FIFO�Ƿ�Ϊ��
char fifo_s_isempty(fifo_s_t *p_fifo)
{
    ASSERT(p_fifo);
    return (p_fifo->used_num ? 0 : 1);
}

// ���FIFO�Ƿ�����
char fifo_s_isfull(fifo_s_t *p_fifo)
{
    ASSERT(p_fifo);
    return (p_fifo->free_num ? 0 : 1);
}

// ��ȡFIFO�е�Ԫ�ظ���
int fifo_s_used(fifo_s_t *p_fifo)
{
    ASSERT(p_fifo);
    return p_fifo->used_num;
}

// ��ȡFIFO��ʣ��ռ�ĸ���
int fifo_s_free(fifo_s_t *p_fifo)
{
    ASSERT(p_fifo);
    return p_fifo->free_num;
}

// ���FIFO����
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

// ����һ���µ�FIFOʵ����֧�ֶ�̬�ڴ����
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

// ����FIFOʵ�����ͷ��ڴ�
void fifo_destory(fifo_t *p_fifo)
{
  ASSERT(p_fifo);
  ASSERT(p_fifo->p_start_addr);

  free(p_fifo->p_start_addr);
  free(p_fifo);

  return;
}

#endif // USE_DYNAMIC_MEMORY

// ��ʼ����̬FIFO�ṹ
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

// ��FIFO�в���һ��Ԫ��
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
        return -1; // FIFO����
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

// ��FIFO�в���һ��Ԫ�أ��������ж�
int fifo_put_noprotect(fifo_t *p_fifo, void *p_element)
{
    ASSERT(p_fifo);
    ASSERT(p_element);

    if (0 == p_fifo->free_num) return -1; // FIFO����

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
        p_fifo->p_write_addr = p_fifo->p_start_addr;

    memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
    p_fifo->p_write_addr += p_fifo->unit_size;
    p_fifo->free_num--;
    p_fifo->used_num++;

    return 0;
}

// ��FIFO�л�ȡһ��Ԫ��
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
        return -1; // FIFOΪ��
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

// ��FIFO�л�ȡһ��Ԫ�أ��������ж�
int fifo_get_noprotect(fifo_t *p_fifo, void *p_element)
{
    ASSERT(p_fifo);
    ASSERT(p_element);

    if (0 == p_fifo->used_num) return -1; // FIFOΪ��

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
        p_fifo->p_read_addr = p_fifo->p_start_addr;

    memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
    p_fifo->p_read_addr += p_fifo->unit_size;
    p_fifo->free_num++;
    p_fifo->used_num--;

    return 0;
}

// Ԥ��ȡFIFO�е�Ԫ��
int fifo_pre_read(fifo_t *p_fifo, char offset, void *p_element)
{
    char *_pre_red_index;

    ASSERT(p_fifo);
    ASSERT(p_element);

    if (offset >= p_fifo->used_num)
    {
        return -1; // Խ��
    }

    _pre_red_index = p_fifo->p_read_addr + p_fifo->unit_size * offset;
    while (_pre_red_index > p_fifo->p_end_addr)
    {
        _pre_red_index = _pre_red_index - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;
    }

    memcpy(p_element, _pre_red_index, p_fifo->unit_size);
    return 0;
}

// ���FIFO�Ƿ�Ϊ��
int fifo_is_empty(fifo_t *p_fifo)
{
    ASSERT(p_fifo);
    return (0 == p_fifo->used_num);
}

// ���FIFO�Ƿ�����
int fifo_is_full(fifo_t *p_fifo)
{
    ASSERT(p_fifo);
    return (0 == p_fifo->free_num);
}

// ��ȡFIFO�е�Ԫ�ظ���
int fifo_used(fifo_t *p_fifo)
{
    ASSERT(p_fifo);
    return p_fifo->used_num;
}

// ��ȡFIFO��ʣ��ռ�ĸ���
int fifo_free(fifo_t *p_fifo)
{
    ASSERT(p_fifo);
    return p_fifo->free_num;
}

// ���FIFO����
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
