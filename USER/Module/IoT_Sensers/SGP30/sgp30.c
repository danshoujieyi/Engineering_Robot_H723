//
// Created by 刘嘉俊 on 25-6-4.
//
/**
 * SGP30 是一款单一芯片上具有多个传感元件的金属氧化物室内气体传感器，内部集成 4 个气体传感元件，
 * 工作电压： 3.3V
 * 工作电流：40mA
 * 输出方式: IIC
 * 管脚数量：4 Pin
 * 具有完全校准的空气质量输出信号，主要是对空气质量进行检测。可以输出：
 * TVOC（Total Volatile Organic Compounds，总挥发性有机物），量程为 0~60000ppb；
 * CO2 浓度，量程 400~60000ppm。
 */
#include "sgp30.h"
#include "i2c.h"
#include "usart_task.h"
#include "cmsis_os.h"
// sgp30_data_t sgp30_data = {0};

/**
 * @brief	向SGP30发送一条指令(16bit)
 * @param	cmd SGP30指令
 * @retval	成功返回HAL_OK
*/
static uint8_t sgp30_send_cmd(sgp30_cmd_t cmd)
{
    uint8_t cmd_buffer[2];
    cmd_buffer[0] = cmd >> 8;
    cmd_buffer[1] = cmd;
    return HAL_I2C_Master_Transmit(&hi2c1, SGP30_ADDR_WRITE, (uint8_t*) cmd_buffer, 2, 0xFFFF);
}

/**
 * @brief	软复位SGP30
 * @param	none
 * @retval	成功返回HAL_OK
*/
static int sgp30_soft_reset(void)
{
    uint8_t cmd = 0x06;
    return HAL_I2C_Master_Transmit(&hi2c1, 0x00, &cmd, 1, 0xFFFF);
}

/**
 * @brief	初始化SGP30空气质量测量模式
 * @param	none
 * @retval	成功返回0，失败返回-1
*/
int sgp30_init(void)
{
    int status;

    status = sgp30_soft_reset();
    if (status != HAL_OK) {
        return -1;
    }

    vTaskDelay(100);

    status = sgp30_send_cmd(INIT_AIR_QUALITY);

    vTaskDelay(100);

    return status == 0 ? 0 : -1;
}

/**
 * @brief	初始化SGP30空气质量测量模式
 * @param	none
 * @retval	成功返回HAL_OK
*/
static int sgp30_start(void)
{
    return sgp30_send_cmd(MEASURE_AIR_QUALITY);
}

#define CRC8_POLYNOMIAL 0x31

static uint8_t CheckCrc8(uint8_t* const message, uint8_t initial_value)
{
    uint8_t  remainder;	    //余数
    uint8_t  i = 0, j = 0;  //循环变量

    /* 初始化 */
    remainder = initial_value;

    for(j = 0; j < 2;j++)
    {
        remainder ^= message[j];

        /* 从最高位开始依次计算  */
        for (i = 0; i < 8; i++)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1)^CRC8_POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }

    /* 返回计算的CRC码 */
    return remainder;
}

/**
 * @brief	读取一次空气质量数据
 * @param	none
 * @retval	成功返回0，失败返回-1
*/

sgp30_data_t sgp30_get_data(void)
{
    sgp30_data_t result = {0};  // 默认全部清零
    uint8_t recv_buffer[6] = {0};

    // 启动测量命令
    if (sgp30_start() != 0) {
        USART7_DebugPrintf("sgp30 start fail\r\n");
        return result;
    }

    vTaskDelay(100);  // 等待测量完成

    if (HAL_I2C_Master_Receive(&hi2c1, SGP30_ADDR_READ, recv_buffer, 6, 0xFFFF) != HAL_OK) {
        USART7_DebugPrintf("I2C Master Receive fail\r\n");
        return result;
    }

    // 校验 CRC
    if (CheckCrc8(&recv_buffer[0], 0xFF) != recv_buffer[2]) {
        USART7_DebugPrintf("co2 recv data crc check fail\r\n");
        return result;
    }
    if (CheckCrc8(&recv_buffer[3], 0xFF) != recv_buffer[5]) {
        USART7_DebugPrintf("tvoc recv data crc check fail\r\n");
        return result;
    }

    // 转换有效数据
    result.co2  = (recv_buffer[0] << 8) | recv_buffer[1];
    result.tvoc = (recv_buffer[3] << 8) | recv_buffer[4];

    return result;
}
