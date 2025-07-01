/** ***************************************************************************
 * @File Name: ebtn_custom_hal.h
 * @brief ebtn应用的抽象硬件操作回调函数声明
 * 在首次移植到特定平台后无需再更改
 * @credit : bobwenstudy / easy_button https://github.com/bobwenstudy/easy_button
 * @Author : Sighthesia / easy_button-Application https://github.com/Sighthesia/easy_button-Application/tree/main
 * @Version : 1.0.0
 * @Creat Date : 2025-05-06
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef EBTN_CUSTOM_HAL_H
#define EBTN_CUSTOM_HAL_H

/* ------------------------------ 此处包含单片机平台的头文件 ----------------------------- */
// 包含单片机平台的声明了GPIO和SysTick相关的头文件
// #include "stm32f1xx.h" // 示例：STM32F1系列单片机的头文件

/* -------------------------- 此处修改硬件回调函数自定义声明（如有需求） ------------------------- */

/** ***************************************************************************
 * @brief  自定义回调函数结构体声明
 */
typedef struct
{
    /** ***************************************************************************
     * @brief 读取GPIO电平，在ebtn_app.c中被使用，用于读取按键引脚的电平
     * @param GPIOx 指向GPIO端口的指针
     * @param GPIO_Pin GPIO引脚号
     * @return GPIO引脚的电平状态（0/1）
     */
    uint8_t (*Read_Pin)(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

    /** ***************************************************************************
     * @brief  获取系统滴答计数器值，用于为ebtn_process()函数提供时间基准
     * @note   SysTick时基为1ms
     * @return 系统滴答计数器ms时间值
     */
    uint32_t (*Get_Tick)(void);
} ebtn_custom_hal_t;

/* -------------------------------- 自定义配置部分结束 ------------------------------- */

extern ebtn_custom_hal_t ebtn_custom_hal; // ebtn适配层回调函数结构声明

#endif /* EBTN_CUSTOM_HAL_H */