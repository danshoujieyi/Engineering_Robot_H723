//
// Created by 刘嘉俊 on 25-3-8.
//

#include "keyboard.h"
#include "FreeRTOS.h"
#include "rm_config.h"
#include "referee_system.h"
#include "user_lib.h"
#include "ramp.h"

/* mouse button long press time */
#define LONG_PRESS_TIME  800   //ms
/* key acceleration time */
#define KEY_ACC_TIME     1700  //ms

extern struct referee_fdb_msg referee_fdb;
extern ramp_obj_t *km_vx_ramp;//x轴控制斜坡
extern ramp_obj_t *km_vy_ramp;//y周控制斜坡

int16_t delta_spd = MAX_CHASSIS_VX_SPEED*1.0f/KEY_ACC_TIME*GIMBAL_PERIOD;

keyboard_control_t keyboard = {0};
mouse_control_t mouse = {0};
/**
  * @brief     鼠标按键状态机
  * @param[in] state: 按键状态指针
  * @param[in] key: 按键键值
  */
static void key_state_machine(key_state_e state, uint8_t key)
{
    switch (state)
    {
        case KEY_RELEASE:
        {
            if (key)
                state = KEY_WAIT_EFFECTIVE;
            else
                state = KEY_RELEASE;
        }break;

        case KEY_WAIT_EFFECTIVE:
        {
            if (key)
                state = KEY_PRESS_ONCE;
            else
                state = KEY_RELEASE;
        }break;


        case KEY_PRESS_ONCE:
        {
            if (key)
            {
                state = KEY_PRESS_DOWN;
                if (state == mouse.lk_state)
                    mouse.lk_cnt = 0;
                else
                    mouse.rk_cnt = 0;
            }
            else
                state = KEY_RELEASE;
        }break;

        case KEY_PRESS_DOWN:
        {
            if (key)
            {
                if (state == mouse.lk_state)
                {
                    if (mouse.lk_cnt++ > LONG_PRESS_TIME/GIMBAL_PERIOD)
                        state = KEY_PRESS_LONG;
                }
                else
                {
                    if (mouse.rk_cnt++ > LONG_PRESS_TIME/GIMBAL_PERIOD)
                        state = KEY_PRESS_LONG;
                }
            }
            else
                state = KEY_RELEASE;
        }break;

        case KEY_PRESS_LONG:
        {
            if (!key)
            {
               state = KEY_RELEASE;
            }
        }break;

        default:
            break;

    }
}

/*
* @brief 将裁判系统解析后的键盘鼠标数据转换成方便使用的遥控器数据结构体
* @param remote 指向原始裁判系统数据的指针
* @return 转换后的遥控器数据结构体
*/
pc_control_t convert_remote_to_pc(const remote_control_t *remote)
{
    pc_control_t pc;

    if(remote == NULL)
    {
        // 如果需要，可以在此处理错误情况
        pc.mouse.x = 0;
        pc.mouse.y = 0;
        pc.mouse.z = 0;
        pc.mouse.l = 0;
        pc.mouse.r = 0;
        pc.keyboard.key_code = 0;
        return pc;
    }

    pc.mouse.x = remote->mouse_x;
    pc.mouse.y = remote->mouse_y;
    pc.mouse.z = remote->mouse_z;
    pc.mouse.l = (uint8_t)remote->left_button_down;
    pc.mouse.r = (uint8_t)remote->right_button_down;
    pc.keyboard.key_code = remote->keyboard_value;

    return pc;
}


/**
  * @brief     PC 处理键盘鼠标数据函数
  */
void PC_keyboard_mouse(pc_control_t pc_control)
{
    // 假设referee_fdb.remote_control已经填充好数据
    pc_control = convert_remote_to_pc(&referee_fdb.remote_control);

    if (pc_control.keyboard.bit.SHIFT)
    {
        keyboard.move_mode = FAST_MODE;
        keyboard.max_spd = 3500;
    }
    else if (pc_control.keyboard.bit.CTRL)
    {
        keyboard.move_mode = SLOW_MODE;
        keyboard.max_spd = 2500;
    }
    else
    {
        keyboard.move_mode = NORMAL_MODE;
        keyboard.max_spd = 3000;
    }

    //add ramp
    if (pc_control.keyboard.bit.W)
        keyboard.vy += (float)delta_spd;
    else if (pc_control.keyboard.bit.S)
        keyboard.vy -= (float)delta_spd;
    else
    {
        keyboard.vy =(float)keyboard.vy* ( 1 - km_vy_ramp->calc(km_vy_ramp));
    }

    if (pc_control.keyboard.bit.A)
        keyboard.vx -= (float)delta_spd;
    else if (pc_control.keyboard.bit.D)
        keyboard.vx += (float)delta_spd;
    else
    {
        keyboard.vx = (float) keyboard.vx* ( 1 - km_vx_ramp->calc(km_vx_ramp));
    }

    VAL_LIMIT(keyboard.vx, -keyboard.max_spd, keyboard.max_spd);
    VAL_LIMIT(keyboard.vy, -keyboard.max_spd, keyboard.max_spd);

    VAL_LIMIT(keyboard.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
    VAL_LIMIT(keyboard.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);

    key_state_machine(mouse.lk_state, pc_control.mouse.l);
    key_state_machine(mouse.rk_state, pc_control.mouse.r);
    key_state_machine(keyboard.e_state, pc_control.keyboard.bit.E);
    key_state_machine(keyboard.f_state, pc_control.keyboard.bit.F);
    key_state_machine(keyboard.shift_state, pc_control.keyboard.bit.SHIFT);
    key_state_machine(keyboard.v_state, pc_control.keyboard.bit.V);
}
