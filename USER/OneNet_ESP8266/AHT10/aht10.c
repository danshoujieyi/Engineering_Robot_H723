//
// Created by 刘嘉俊 on 25-6-5.
//
/**
 * HT10，新一代温湿度传感器在尺寸与智能方面建立了新的标准：它嵌入了适于回流焊的双列扁平无引脚 SMD 封装，底面 4 x 5mm ，高度 1.6mm。
 * 传感器输出经过标定的数字信号，标准 I 2 C 格式。
 * 工作电压： 1.8~3.6V
 * 工作电流： 0.25~23uA
 * 湿度误差： ±2%RH
 * 温度误差： ±0.3℃
 * 输出方式: IIC
 * 管脚数量： 3 Pin
 * TODO：只允许接3.3V，超过即烧毁
 */
#include "aht10.h"
#include "cmsis_os.h"
#include "i2c.h"

#define AHT10_I2C_HANDLE hi2c1
/**
 * @brief AHT10 初始化命令
 */
void aht10_init(void)
{
    uint8_t cmd = AHT10_Init_com;
    HAL_I2C_Master_Transmit(&AHT10_I2C_HANDLE, AHT10_Write_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
    vTaskDelay(10);
}

/**
 * @brief 软复位 AHT10
 */
static void aht10_soft_reset(void)
{
    uint8_t cmd = AHT10_SoftReset_com;
    HAL_I2C_Master_Transmit(&AHT10_I2C_HANDLE, AHT10_Write_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
    vTaskDelay(20);
}

/**
 * @brief 触发测量
 */
static void aht10_trigger_measurement(void)
{
    uint8_t cmd[3] = { AHT10_TrigeMea_com, 0x33, 0x00 };
    HAL_I2C_Master_Transmit(&AHT10_I2C_HANDLE, AHT10_Write_ADDRESS, cmd, 3, HAL_MAX_DELAY);
    vTaskDelay(80);  // 等待测量完成
}

/**
 * @brief 获取温湿度数据（主接口）
 */
aht10_data_t aht10_get_data(void)
{
    aht10_data_t result = {0};
    uint8_t rx_buf[6];
    uint32_t raw_humi = 0, raw_temp = 0;

    aht10_trigger_measurement();

    if (HAL_I2C_Master_Receive(&AHT10_I2C_HANDLE, AHT10_Read_ADDRESS, rx_buf, 6, HAL_MAX_DELAY) != HAL_OK)
        return result; // 读取失败

    uint8_t busy = (rx_buf[0] >> 7) & 0x01;
    uint8_t cal = (rx_buf[0] >> 3) & 0x01;

    if (!cal) {
        aht10_soft_reset();
        return result;
    }

    if (!busy) {
        raw_humi = (rx_buf[1] << 12) | (rx_buf[2] << 4) | (rx_buf[3] >> 4);
        raw_temp = ((rx_buf[3] & 0x0F) << 16) | (rx_buf[4] << 8) | rx_buf[5];

        result.humidity = ((float)raw_humi / 1048576.0f) * 100.0f;
        result.temperature = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;
        result.valid = 1;
    }

    return result;
}