//
// Created by 刘嘉俊 on 25-6-5.
//
/**
 * 采用 ROHM 原装 BH1750FVI 芯片供电电源:3-5V，光照度范围:0-65535lx
 * 传感器内置 16bitAD 转换器，直接数字输出，省略复杂的计算，省略标定，不区分环境光源接近于视觉灵敏度的分光特性，
 * 可对广泛的亮度进行 1 勒克斯的高精度测定。
 * 标准 NXPICC 通信协议，模块内部包含通信电平转换，可以与 5V 单片机 io 直接连接。
 * 工作电压： 3-5V
 * 工作电流： 200uA
 * 探测范围： 1~65536 lx
 * 模块尺寸： 32.6mm×15.2mm×11.6mm
 * 输出方式: IIC
 * 管脚数量：5 Pin
 */
#include "bh1750fui.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef BH1750_I2C_HANDLE;

/**
 * @brief 初始化 BH1750 传感器
 */
void bh1750_init(void)
{
    uint8_t cmd = BH1750_POWER_ON;
    HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY);

    vTaskDelay(10);
    cmd = BH1750_RESET;
    HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY);

    vTaskDelay(10);
    cmd = BH1750_CONT_H_MODE;
    HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY);
}

/**
 * @brief 读取光照强度（单位：Lux）
 * @param data - BH1750 结构体指针
 * @return 0 成功；1 失败
 */
uint8_t bh1750_read_lux(bh1750_data_t *data)
{
    uint8_t buf[2];
    if (HAL_I2C_Master_Receive(&BH1750_I2C_HANDLE, BH1750_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }

    uint16_t raw = (buf[0] << 8) | buf[1];
    data->lux = raw / 1.2f;  // 数据手册规定除以1.2是转换系数

    return 0;
}