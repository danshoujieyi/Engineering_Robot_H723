/**
 ******************************************************************************
 * @file    BMI088driver.c
 * @author
 * @version V1.2.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "drv_dwt.h"
#include <math.h>


static const float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
static const float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
static uint8_t res = 0;
static uint8_t write_reg_num = 0;
static uint8_t error = BMI088_NO_ERROR;

//---------BMI088校准参数变量---------//
float gyroDiff[3], gNormDiff;  // 陀螺仪加速度计最大最小值之差
uint8_t caliOffset = 1;  // 决定是否读取BMI088校准参数
int16_t caliCount = 0;  // BMI088校准轮次计数，重新校准次数
int16_t Count = 0;  // 完成一次校准采集次数
//---------BMI088校准参数变量---------//

ImuDataTypeDef BMI088;

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
        {
                {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
                {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
                {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
                {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
                {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
                {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

        };

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
        {
                {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
                {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
                {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
                {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
                {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
                {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

        };

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

// static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//     BMI088_read_write_byte( reg );
//     while( len != 0 )
//     {
//
//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }
//}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

//TODO:陀螺仪零漂校准函数
void Calibrate_MPU_Offset(ImuDataTypeDef *bmi088)
{
    static float startTime;
    static uint16_t CaliTimes = 5000;   // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    float gyroMax[3], gyroMin[3];

    float gNormTemp, gNormMax, gNormMin;  // 加速度计极值

#ifndef BSP_BMI088_CALI  // 更换开发板或定期校准，以便快速启动
    static uint8_t cali_dt_max = 0;   // 对于不同开发板都需要定期校准，此处，已经校准故cali_dt_max = 0
#elif
    static uint8_t cali_dt_max = 20;
#endif /* BSP_BMI088_CALI */

    startTime = dwt_get_time_s();
    do
    {
        if (dwt_get_time_s() - startTime > cali_dt_max)
        {
            // 校准超时，使用预设偏移值（如上次校准结果）
            bmi088->gyro_offset[0] = GxOFFSET;   // 采用提前校准好的值，避免重复校准
            bmi088->gyro_offset[1] = GyOFFSET;
            bmi088->gyro_offset[2] = GzOFFSET;
            bmi088->g_norm = gNORM;
            bmi088->temp_when_cali = 40;
            break;
        }

        dwt_delay_ms(5);
        bmi088->g_norm = 0;
        bmi088->gyro_offset[0] = 0;
        bmi088->gyro_offset[1] = 0;
        bmi088->gyro_offset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            // 1. 读取加速度计数据（X/Y/Z轴）
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

            // 2. 计算加速度合值（用于判断是否静止）
            gNormTemp = sqrtf(bmi088->accel[0] * bmi088->accel[0] +
                              bmi088->accel[1] * bmi088->accel[1] +
                              bmi088->accel[2] * bmi088->accel[2]);
            bmi088->g_norm += gNormTemp;

            // 3. 读取陀螺仪数据（X/Y/Z轴）
            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->gyro_offset[0] += bmi088->gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->gyro_offset[1] += bmi088->gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->gyro_offset[2] += bmi088->gyro[2];
            }

            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = bmi088->gyro[j];
                    gyroMin[j] = bmi088->gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (bmi088->gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088->gyro[j];
                    if (bmi088->gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088->gyro[j];
                }
            }

            // 数据差异过大认为收到外界干扰，需重新校准
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.3f ||
                gyroDiff[0] > 0.1f || // 若加速度合值波动过大（>0.3m/s?）或陀螺仪单轴波动超过 0.1°/s，认为设备未静止，放弃当前轮次数据，重新校准。
                gyroDiff[1] > 0.1f ||
                gyroDiff[2] > 0.1f)
                break;
            dwt_delay_ms(1);; // 微小延时（避免采样过于密集）

            Count++; // 采集次数计数（外部变量，需自行定义）
        }

        // 取平均值得到标定结果
        bmi088->g_norm /= (float)CaliTimes; // 平均加速度合值（理想值≈9.81m/s?）
        for (uint8_t i = 0; i < 3; i++)
            bmi088->gyro_offset[i] /= (float)CaliTimes; // 陀螺仪零偏（平均值即零漂）,因为理论陀螺仪值应该为0，

        // 记录标定时的温度（影响传感器漂移）
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088->temp_when_cali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++; // 校准轮次计数（外部变量，需自行定义）
    } while (gNormDiff > 0.3f ||  // 加速度波动超限
             fabsf(bmi088->g_norm - 9.81f) > 0.5f ||  // 平均合值偏离重力加速度超过0.5m/s?
             gyroDiff[0] > 0.1f ||    // 陀螺仪波动超限
             gyroDiff[1] > 0.1f ||
             gyroDiff[2] > 0.1f ||
             fabsf(bmi088->gyro_offset[0]) > 0.01f ||   // 零偏绝对值超限（0.01°/s）
             fabsf(bmi088->gyro_offset[1]) > 0.01f ||
             fabsf(bmi088->gyro_offset[2]) > 0.01f);   // 不满足校准条件则重复循环
    //TODO:
    // 根据标定结果校准加速度计标度因数
    bmi088->accel_scale = 9.81f / bmi088->g_norm;  // 9.7865f取决于海南省标准，其他地区可能不同
}


uint8_t BMI088_init(SPI_HandleTypeDef *bmi088_SPI)
{
    BMI088_SPI = bmi088_SPI;
    error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    Calibrate_MPU_Offset(&BMI088);

    return error;
}


uint8_t bmi088_accel_init(void)
{
    // check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    // do
    // {
    //     BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    // } while (res != BMI088_ACC_CHIP_ID_VALUE);

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            // write_reg_num--;
            // return write_BMI088_accel_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    // check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);
    // check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            write_reg_num--;
            // return write_BMI088_gyro_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

void BMI088_Read(ImuDataTypeDef *bmi088)
{
    static uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    static int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    bmi088->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    bmi088->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    bmi088->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->accel_scale;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->gyro_offset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    bmi088->temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}




