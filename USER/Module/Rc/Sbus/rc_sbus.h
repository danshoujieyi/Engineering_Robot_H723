 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-24      ChuShicheng     first version
 *                 ZhengWanshun
 *                 Yangshuo
 *                 ChenSihan
 */
#ifndef _RC_SBUS_H
#define _RC_SBUS_H

#include "cmsis_os.h"

#define SBUS_RX_BUF_NUM 25  //36u
#define SBUS_FRAME_SIZE 25u


/**
  * @brief 遥控器拨杆值
  */
enum {
    RC_UP = 240,
    RC_MI = 0,
    RC_DN = 15,
};

typedef struct
{
    uint16_t online;
    /* 摇杆最终值为：-784~783 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧上下
    int16_t ch4;   //左侧左右
    /* FS-i6x旋钮为线性，左右最终值为：-784~783 */
    int16_t ch5;   //左侧线性旋钮
    int16_t ch6;   //右侧线性旋钮
    /* 遥控器的拨杆数据，上(中)下最终值分别为：240、（0）、15 */
    uint8_t sw1;   //SWA，二档
    uint8_t sw2;   //SWB，二档
    uint8_t sw3;   //SWC，三档
    uint8_t sw4;   //SWD，二档

    /* PC 鼠标数据 */
    struct
    {
        /* 鼠标移动相关 */
        int16_t x;   //鼠标平移
        int16_t y;   //鼠标上下
        /* 鼠标按键相关，1为按下，0为松开 */
        uint8_t l;   //左侧按键
        uint8_t r;   //右侧按键
    }mouse;

    /* PC 键盘按键数据 */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        }bit;
    }kb;

    /* 遥控器左侧拨轮数据数值范围:（左）660 ~ -660(右) */
    int16_t wheel;
} rc_obj_t;

/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
 rc_obj_t *sbus_rc_init(void);

 void sbus_rc_decode(uint8_t *buff);

 extern uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
 extern rc_obj_t rc_obj[2];

#endif /* _RC_SBUS_H */
