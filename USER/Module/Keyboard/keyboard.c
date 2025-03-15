//
// Created by ���ο� on 25-3-8.
//

#include "keyboard.h"
#include "FreeRTOS.h"
#include "rm_config.h"
#include "referee_system.h"
#include "user_lib.h"
#include "ramp.h"
#include "chassis_task.h"
#include "rm_task.h"
#include "pump.h"

/* mouse button long press time */
#define LONG_PRESS_TIME  800   //ms
/* key acceleration time */
#define KEY_ACC_TIME     1700  //ms

extern struct referee_fdb_msg referee_fdb;
extern struct chassis_cmd_msg chassis_cmd;
extern ramp_obj_t *km_vx_ramp;//x�����б��
extern ramp_obj_t *km_vy_ramp;//y�ܿ���б��
extern ramp_obj_t *km_vw_ramp; // ��ת����б�£������ⲿ����

static float delta_spd = MAX_CHASSIS_VX_SPEED*1.0f/KEY_ACC_TIME*GIMBAL_PERIOD;
static float delta_spd_w = MAX_CHASSIS_VW_SPEED*1.0f/KEY_ACC_TIME*GIMBAL_PERIOD;

keyboard_control_t keyboard = {0};
mouse_control_t mouse = {0};
/**
  * @brief     ��갴��״̬��
  * @param[in] state: ����״ָ̬��
  * @param[in] key: ������ֵ
  */
void key_state_machine(key_state_e *state, uint8_t key)
{
    switch (*state)
    {
        case KEY_RELEASE:
        {
            if (key)
                *state = KEY_WAIT_EFFECTIVE;
            else
                *state = KEY_RELEASE;
        } break;

        case KEY_WAIT_EFFECTIVE:
        {
            if (key)
                *state = KEY_PRESS_ONCE;
            else
                *state = KEY_RELEASE;
        } break;

        case KEY_PRESS_ONCE:
        {
            if (key)
            {
                *state = KEY_PRESS_DOWN;
                // ���ݾ������ѡ����������Ҽ���������
                if (*state == mouse.lk_state)
                    mouse.lk_cnt = 0;
                else
                    mouse.rk_cnt = 0;
            }
            else
                *state = KEY_RELEASE;
        } break;

        case KEY_PRESS_DOWN:
        {
            if (key)
            {
                if (*state == mouse.lk_state)
                {
                    if (mouse.lk_cnt++ > LONG_PRESS_TIME / GIMBAL_PERIOD)
                        *state = KEY_PRESS_LONG;
                }
                else
                {
                    if (mouse.rk_cnt++ > LONG_PRESS_TIME / GIMBAL_PERIOD)
                        *state = KEY_PRESS_LONG;
                }
            }
            else
                *state = KEY_RELEASE;
        } break;

        case KEY_PRESS_LONG:
        {
            if (!key)
            {
                *state = KEY_RELEASE;
            }
        } break;

        default:
            break;
    }
}

/*
* @brief ������ϵͳ������ļ����������ת���ɷ���ʹ�õ�ң�������ݽṹ��
* @param remote ָ��ԭʼ����ϵͳ���ݵ�ָ��
* @return ת�����ң�������ݽṹ��
*/
pc_control_t convert_remote_to_pc(const remote_control_t *remote)
{
    pc_control_t pc;

    if(remote == NULL)
    {
        // �����Ҫ�������ڴ˴���������
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



void PC_keyboard_mouse(const pc_control_t *pc_control)
{

    key_state_machine(&keyboard.g_state,pc_control->keyboard.bit.G);
    if (keyboard.g_state == KEY_PRESS_ONCE)
    {
        chassis_cmd.ctrl_mode=CHASSIS_RELAX;
    }

    key_state_machine(&keyboard.f_state,pc_control->keyboard.bit.F);
    if (keyboard.f_state == KEY_PRESS_ONCE)
    {
        chassis_cmd.ctrl_mode=CHASSIS_ENABLE;
    }

    if (pc_control->keyboard.bit.SHIFT )
    {
        keyboard.move_mode = FAST_MODE;
        keyboard.max_spd = 3500;
    }
    if (pc_control->keyboard.bit.CTRL)
    {
        keyboard.move_mode = SLOW_MODE;
        keyboard.max_spd = 2000;
    }
    else
    {
        keyboard.move_mode = NORMAL_MODE;
        keyboard.max_spd = 3000;
    }

    // ǰ������W/S��
    if (pc_control->keyboard.bit.W) {
        keyboard.vy += delta_spd;
    } else if (pc_control->keyboard.bit.S) {
        keyboard.vy -= delta_spd;
    } else {
        keyboard.vy *= (1 - km_vy_ramp->calc(km_vy_ramp));
    }

    // ���ҷ�����A/D��
    if (pc_control->keyboard.bit.A) {
        keyboard.vx -= delta_spd;
    } else if (pc_control->keyboard.bit.D) {
        keyboard.vx += delta_spd;
    } else {
        keyboard.vx *= (1 - km_vx_ramp->calc(km_vx_ramp)); // ������ʱ�ٶ�˥��
    }

    /* ��ת���ƣ�Q/E��*/
    if (pc_control->keyboard.bit.Q) {
        keyboard.vw -= delta_spd_w; // ����
    } else if (pc_control->keyboard.bit.E) {
        keyboard.vw += delta_spd_w; // ����
    } else {
        keyboard.vw *= (1 - km_vw_ramp->calc(km_vw_ramp)); // ʹ����תб��
    }


    VAL_LIMIT(keyboard.vx, -keyboard.max_spd, keyboard.max_spd);
    VAL_LIMIT(keyboard.vy, -keyboard.max_spd, keyboard.max_spd);
    VAL_LIMIT(keyboard.vw, -keyboard.max_spd, keyboard.max_spd);

    VAL_LIMIT(keyboard.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
    VAL_LIMIT(keyboard.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
    VAL_LIMIT(keyboard.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);

    key_state_machine(&keyboard.v_state, pc_control->keyboard.bit.V);
    set_pump(keyboard.v_state == KEY_PRESS_LONG);

}
