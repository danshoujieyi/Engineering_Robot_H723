#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "spi.h"

#define BMI088_USE_SPI
/*
#define CS1_ACCEL_GPIO_Port ACCEL_NSS_GPIO_Port
#define CS1_ACCEL_Pin ACCEL_NSS_Pin
#define CS1_GYRO_GPIO_Port GYRO_NSS_GPIO_Port
#define CS1_GYRO_Pin GYRO_NSS_Pin
*/

// 方便其他文件调用
extern SPI_HandleTypeDef *BMI088_SPI;

#if defined(BMI088_USE_SPI)
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
uint8_t BMI088_read_write_byte(uint8_t reg);
#endif

#endif
