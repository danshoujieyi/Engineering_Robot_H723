//
// Created by 刘嘉俊 on 25-3-8.
//

#ifndef CTRBOARD_H7_ALL_KEYBOARD_H
#define CTRBOARD_H7_ALL_KEYBOARD_H
#include "FreeRTOS.h"
#include "referee_system.h"
/**
  * @brief     底盘运动速度快慢模式
  */
typedef enum
{
    NORMAL_MODE = 0,    //正常模式
    FAST_MODE,          //快速模式
    SLOW_MODE,          //慢速模式
} key_move_e;

/**
  * @brief     鼠标按键状态类型枚举
  */
typedef enum
{
    KEY_RELEASE = 0,    //没有按键按下
    KEY_WAIT_EFFECTIVE, //等待按键按下有效，防抖
    KEY_PRESS_ONCE,     //按键按下一次的状态
    KEY_PRESS_DOWN,     //按键已经被按下
    KEY_PRESS_LONG,     //按键长按状态
} key_state_e;

/**
  * @brief     键盘鼠标数据结构体
  */
typedef struct
{
    /* 键盘模式使能标志 */
    uint8_t keyboard_enable;

    /* 鼠标键盘控制模式下的底盘移动速度目标值 */
    float vx;          //底盘前进后退目标速度
    float vy;          //底盘左右平移目标速度
    float vw;          //底盘旋转速度
    float max_spd;     //运动最大速度

    /* 键盘按键状态 */
    key_state_e e_state; //E键按键状态
    key_state_e f_state; //F键按键状态
    key_state_e shift_state; //SHIFT键按键状态
    key_state_e v_state; //V键按键状态
    key_state_e g_state; //V键按键状态

    /* 运动模式，键盘控制底盘运动快慢 */
    key_move_e move_mode;

} keyboard_control_t;

typedef struct {
    uint8_t mouse_enable;

    /* 左右按键状态 */
    key_state_e lk_state; //左侧按键状态
    key_state_e mk_state; //中间按键状态
    key_state_e rk_state; //右侧按键状态

    uint16_t lk_cnt;
    uint16_t mk_cnt;
    uint16_t rk_cnt;

} mouse_control_t;

/**
  * @brief 解析后的遥控器数据结构体
  */
typedef struct
{
    /* PC 鼠标数据 */
    struct
    {
        int16_t x;   // 鼠标水平平移,负值标识向左移动
        int16_t y;   // 鼠标垂直移动,负值标识向下移动
        int16_t z;   // 鼠标滚轮滚动,负值标识向后滚动
        uint8_t l;   // 鼠标左键，1为按下，0为松开
        uint8_t r;   // 鼠标右键，1为按下，0为松开
    } mouse;

    /* PC 键盘按键数据 */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W     :1;
            uint16_t S     :1;
            uint16_t A     :1;
            uint16_t D     :1;
            uint16_t SHIFT :1;
            uint16_t CTRL  :1;
            uint16_t Q     :1;
            uint16_t E     :1;
            uint16_t R     :1;
            uint16_t F     :1;
            uint16_t G     :1;
            uint16_t Z     :1;
            uint16_t X     :1;
            uint16_t C     :1;
            uint16_t V     :1;
            uint16_t B     :1;
        } bit;
    } keyboard;

} pc_control_t;

void key_state_machine(key_state_e *state, uint8_t key);

pc_control_t convert_remote_to_pc(const remote_control_t *remote);

void PC_keyboard_mouse(const pc_control_t *pc_control);

#endif //CTRBOARD_H7_ALL_KEYBOARD_H
