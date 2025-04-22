#include "drv_dwt.h"

static dwt_time_t systime;
static uint32_t cpu_freq_hz, cpu_freq_hz_ms, cpu_freq_hz_us; // CPU频率（Hz，动态获取）
static uint32_t CYCCNT_last = 0;       // 上次DWT计数值
static uint64_t CYCCNT_overflow = 0;   // 溢出次数（64位，处理32位回绕）

/**
 * @brief 用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_rount_count
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 */
static void dwt_cnt_update(void)
{
//    // 挂起任务调度，进入临界区
//    vTaskSuspendAll(); // 该临界保护中断不可用

    static volatile uint8_t dwt_locker = 0;
    if (!dwt_locker) // 基本临界保护，避免中断不可用
    {
        dwt_locker = 1;
        volatile uint32_t cnt_now = DWT_CYCCNT;
        if (cnt_now < CYCCNT_last){
            CYCCNT_overflow ++;
        }
        CYCCNT_last = cnt_now;
        dwt_locker = 0;
    }

//    // 恢复任务调度
//    (void)xTaskResumeAll();
}

void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 启用跟踪单元
    DWT->CYCCNT = 0;         // 复位计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 启动周期计数

    // 动态获取系统时钟频率（STM32F407的HCLK=系统时钟，168MHz典型值）
    cpu_freq_hz = HAL_RCC_GetSysClockFreq(); // 自动获取系统时钟频率
    cpu_freq_hz_ms = cpu_freq_hz / 1000;
    cpu_freq_hz_us = cpu_freq_hz / 1000000;

    CYCCNT_overflow = 0;
    dwt_systime_update(); // 初始化时间轴
}

float dwt_get_delta(uint32_t *cnt_last)
{
    dwt_cnt_update();

    volatile uint32_t cnt_now = DWT_CYCCNT;
    volatile uint64_t delta_cycles;
    // 计算真实计数值差（处理回绕：0xFFFFFFFF -> 0）
    if (cnt_now >= *cnt_last) {
        delta_cycles = (uint64_t)cnt_now - *cnt_last;
    } else {
        delta_cycles = ((uint64_t)UINT32_MAX -  *cnt_last) + (uint64_t)cnt_now + 1;  // 溢出时的正确差值
    }

    float dt = ((float)delta_cycles) / ((float)(cpu_freq_hz));
    *cnt_last = cnt_now;

    return dt;
}

double dwt_get_delta_64(uint32_t *cnt_last)
{
    dwt_cnt_update();

    volatile uint32_t cnt_now = DWT_CYCCNT;
    volatile uint64_t delta_cycles;
    // 计算真实计数值差（处理回绕：0xFFFFFFFF -> 0）
    if (cnt_now >= *cnt_last) {
        delta_cycles = (uint64_t)cnt_now - *cnt_last;
    } else {
        delta_cycles = ((uint64_t)UINT32_MAX -  *cnt_last) + (uint64_t)cnt_now + 1;  // 溢出时的正确差值
    }

    double dt = ((double)delta_cycles) / ((double)(cpu_freq_hz));
    *cnt_last = cnt_now;

    return dt;
}

void dwt_systime_update(void)
{
    dwt_cnt_update();

    volatile uint32_t cnt_now = DWT_CYCCNT;
    uint64_t total_cycles = CYCCNT_overflow * ((uint64_t)UINT32_MAX + 1) + (uint64_t)cnt_now;  // 64位总计数，处理溢出

    // 拆分秒、毫秒、微秒（基于CPU频率精确计算）
    systime.s = total_cycles / cpu_freq_hz;
    uint64_t remainder = total_cycles % cpu_freq_hz;
    systime.ms = remainder / cpu_freq_hz_ms;        // 毫秒部分
    systime.us = (remainder % cpu_freq_hz_ms) / cpu_freq_hz_us;  // 微秒部分
}

float dwt_get_time_s(void)
{
    dwt_systime_update();

    float DWT_Timelinef32 = systime.s + systime.ms * 0.001f + systime.us * 0.000001f;

    return DWT_Timelinef32;
}

float dwt_get_time_ms(void)
{
    dwt_systime_update();

    float DWT_Timelinef32 = systime.s * 1000 + systime.ms + systime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t dwt_get_time_us(void)
{
    dwt_systime_update();

    uint64_t DWT_Timelinef32 = systime.s * 1000000 + systime.ms * 1000 + systime.us;

    return DWT_Timelinef32;
}

/**
 * @brief DWT延时函数（修正循环条件，核心错误修复）
 * @note 不建议超过4秒延时（32位计数器最大周期），超长延时需分段
 */
void dwt_delay_s(float delay) {
    uint32_t start = DWT_CYCCNT;
    uint64_t target_cycles = (uint64_t)delay * (uint64_t)cpu_freq_hz + 0.5;  // 四舍五入防误差

    // 循环直到计数值差 >= 目标周期数（处理溢出）
    while (1) {
        //  dwt_systime_update();  // 临界区保护，确保计数值读取原子性
        uint32_t now = DWT_CYCCNT;
        uint64_t delta = (now >= start) ?
                         (uint64_t)now - (uint64_t)start :
                         ((uint64_t)UINT32_MAX - (uint64_t)start) + (uint64_t)now + 1;  // 溢出时的真实差值

        if (delta >= target_cycles) break;
    }
}

/**
 * @brief 获取当前纳秒级时间戳（自初始化后的总时间）
 * @note 1. 采用整数运算避免浮点精度损失
 *       2. 包含FreeRTOS临界区保护，确保多任务安全
 *       3. 实时获取当前计数值，处理32位计数器溢出
 * @return 纳秒级时间戳（无符号64位整数，0表示未初始化）
 */
uint64_t dwt_get_time_ns(void) {
    dwt_cnt_update();

    volatile uint32_t cnt_now = DWT_CYCCNT;
    uint64_t total_cycles = CYCCNT_overflow * ((uint64_t)UINT32_MAX + 1ULL) + (uint64_t)cnt_now;

    // 避免乘法溢出：分段计算
    uint64_t ns = (total_cycles / (uint64_t)cpu_freq_hz) * 1000000000ULL;
    ns += ((total_cycles % (uint64_t)cpu_freq_hz) * 1000000000ULL) / (uint64_t)cpu_freq_hz;

    return ns;
}