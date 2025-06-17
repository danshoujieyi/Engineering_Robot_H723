// Tip: 遥控器接收模块
#include "rc_sbus.h"
#include "robot.h"
#include <stm32h723xx.h>
#include <string.h>
#include "cmsis_os.h"
#include "stdlib.h"

/* 数据有效性检查 */
#define VALID_CHANNEL(val) (abs(val) <= RC_MAX_VALUE)

// 声明互斥锁句柄
static SemaphoreHandle_t  sbus_mutex;

static sbus_data_t sbus_data;
sbus_data_t sbus_data_fdb;

void sbus_data_init()
{
    memset(&sbus_data, 0, sizeof(sbus_data_t));
    memset(&sbus_data_fdb, 0, sizeof(sbus_data_t));
    sbus_mutex = xSemaphoreCreateMutex();  // 初始化互斥锁
}

/**
 * @brief 遥控器sbus数据解析
 *
 * @param rc_obj 指向sbus_rc实例的指针
 */
void sbus_data_unpack(uint8_t *data, uint16_t len){
    if ((data[0] != SBUS_HEAD) || (data[24] != SBUS_END))
    {
        sbus_data.online = 0;
        return;
    }

    sbus_data.online = (data[23] & 0x0C) ? 0 : 1;

    /* 下面是正常遥控器数据的处理 */
    sbus_data.ch1 = ((data[1] | data[2] << 8) & 0x07FF) - 1024;
    sbus_data.ch2 = ((data[2] >> 3 | data[3] << 5) & 0x07FF) - 1024;
    sbus_data.ch3 = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF) - 1024;
    sbus_data.ch4 = ((data[5] >> 1 | data[6] << 7) & 0x07FF) - 1024;

    /* 旋钮值获取 */
    sbus_data.ch5 = ((data[6] >> 4 | data[7] << 4) & 0x07FF) - 1024;
    sbus_data.ch6 = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF) - 1024;
    /* 防止遥控器零点有偏差 */
    if(sbus_data.ch1 <= 10 && sbus_data.ch1 >= -10)
        sbus_data.ch1 = 0;
    if(sbus_data.ch2 <= 10 && sbus_data.ch2 >= -10)
        sbus_data.ch2 = 0;
    if(sbus_data.ch3 <= 10 && sbus_data.ch3 >= -10)
        sbus_data.ch3 = 0;
    if(sbus_data.ch4 <= 10 && sbus_data.ch4 >= -10)
        sbus_data.ch4 = 0;
    if(sbus_data.ch5 <= 10 && sbus_data.ch5 >= -10)
        sbus_data.ch5 = 0;
    if(sbus_data.ch6 <= 10 && sbus_data.ch6 >= -10)
        sbus_data.ch6 = 0;

    /* 拨杆值获取 */
    sbus_data.sw1 = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
    sbus_data.sw2 = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
    sbus_data.sw3 = ((data[12] | data[13] << 8) & 0x07FF);
    sbus_data.sw4 =((data[13] >> 3 | data[14] << 5) & 0x07FF);

    /* 数据有效性检查 */
    if (VALID_CHANNEL(sbus_data.ch1) &&
        VALID_CHANNEL(sbus_data.ch2) &&
        VALID_CHANNEL(sbus_data.ch3) &&
        VALID_CHANNEL(sbus_data.ch4) &&
        VALID_CHANNEL(sbus_data.ch5) &&
        VALID_CHANNEL(sbus_data.ch6))
    {
        xSemaphoreTake(sbus_mutex, portMAX_DELAY);
        memcpy(&sbus_data_fdb, &sbus_data, sizeof(sbus_data_t));
        xSemaphoreGive(sbus_mutex);
    }
}
