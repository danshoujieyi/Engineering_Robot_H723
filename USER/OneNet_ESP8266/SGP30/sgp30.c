//
// Created by ���ο� on 25-6-4.
//
/**
 * SGP30 ��һ�һоƬ�Ͼ��ж������Ԫ���Ľ����������������崫�������ڲ����� 4 �����崫��Ԫ����
 * ������ѹ�� 3.3V
 * ����������40mA
 * �����ʽ: IIC
 * �ܽ�������4 Pin
 * ������ȫУ׼�Ŀ�����������źţ���Ҫ�ǶԿ����������м�⡣���������
 * TVOC��Total Volatile Organic Compounds���ܻӷ����л��������Ϊ 0~60000ppb��
 * CO2 Ũ�ȣ����� 400~60000ppm��
 */
#include "sgp30.h"
#include "i2c.h"
#include "usart_task.h"
#include "cmsis_os.h"
// sgp30_data_t sgp30_data = {0};

/**
 * @brief	��SGP30����һ��ָ��(16bit)
 * @param	cmd SGP30ָ��
 * @retval	�ɹ�����HAL_OK
*/
static uint8_t sgp30_send_cmd(sgp30_cmd_t cmd)
{
    uint8_t cmd_buffer[2];
    cmd_buffer[0] = cmd >> 8;
    cmd_buffer[1] = cmd;
    return HAL_I2C_Master_Transmit(&hi2c1, SGP30_ADDR_WRITE, (uint8_t*) cmd_buffer, 2, 0xFFFF);
}

/**
 * @brief	��λSGP30
 * @param	none
 * @retval	�ɹ�����HAL_OK
*/
static int sgp30_soft_reset(void)
{
    uint8_t cmd = 0x06;
    return HAL_I2C_Master_Transmit(&hi2c1, 0x00, &cmd, 1, 0xFFFF);
}

/**
 * @brief	��ʼ��SGP30������������ģʽ
 * @param	none
 * @retval	�ɹ�����0��ʧ�ܷ���-1
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
 * @brief	��ʼ��SGP30������������ģʽ
 * @param	none
 * @retval	�ɹ�����HAL_OK
*/
static int sgp30_start(void)
{
    return sgp30_send_cmd(MEASURE_AIR_QUALITY);
}

#define CRC8_POLYNOMIAL 0x31

static uint8_t CheckCrc8(uint8_t* const message, uint8_t initial_value)
{
    uint8_t  remainder;	    //����
    uint8_t  i = 0, j = 0;  //ѭ������

    /* ��ʼ�� */
    remainder = initial_value;

    for(j = 0; j < 2;j++)
    {
        remainder ^= message[j];

        /* �����λ��ʼ���μ���  */
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

    /* ���ؼ����CRC�� */
    return remainder;
}

/**
 * @brief	��ȡһ�ο�����������
 * @param	none
 * @retval	�ɹ�����0��ʧ�ܷ���-1
*/

sgp30_data_t sgp30_get_data(void)
{
    sgp30_data_t result = {0};  // Ĭ��ȫ������
    uint8_t recv_buffer[6] = {0};

    // ������������
    if (sgp30_start() != 0) {
        USART7_DebugPrintf("sgp30 start fail\r\n");
        return result;
    }

    vTaskDelay(100);  // �ȴ��������

    if (HAL_I2C_Master_Receive(&hi2c1, SGP30_ADDR_READ, recv_buffer, 6, 0xFFFF) != HAL_OK) {
        USART7_DebugPrintf("I2C Master Receive fail\r\n");
        return result;
    }

    // У�� CRC
    if (CheckCrc8(&recv_buffer[0], 0xFF) != recv_buffer[2]) {
        USART7_DebugPrintf("co2 recv data crc check fail\r\n");
        return result;
    }
    if (CheckCrc8(&recv_buffer[3], 0xFF) != recv_buffer[5]) {
        USART7_DebugPrintf("tvoc recv data crc check fail\r\n");
        return result;
    }

    // ת����Ч����
    result.co2  = (recv_buffer[0] << 8) | recv_buffer[1];
    result.tvoc = (recv_buffer[3] << 8) | recv_buffer[4];

    return result;
}
