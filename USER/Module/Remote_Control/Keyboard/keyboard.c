//
// Created by ���ο� on 25-3-8.
//

#include "keyboard.h"
#include "FreeRTOS.h"
#include "robot.h"
#include "referee_system.h"
#include "user_lib.h"
#include "ramp.h"
#include "chassis_task.h"
#include "robot_task.h"
#include "pump.h"
#include "DMmotor_task.h"

/* key acceleration time */
#define KEY_ACC_TIME     1700  //ms

extern struct referee_fdb_msg referee_fdb;
extern struct cmd_chassis_msg cmd_chassis;
extern ramp_obj_t *km_vx_ramp;//x�����б��
extern ramp_obj_t *km_vy_ramp;//y�ܿ���б��
extern ramp_obj_t *km_vw_ramp; // ��ת����б�£������ⲿ����

static float base_delta = 500.0f  / KEY_ACC_TIME;
static float base_delta_w = MAX_CHASSIS_VW_SPEED  / KEY_ACC_TIME;

/* ʱ������궨�� */
#define LONG_PRESS_DEFAULT_MS   800   // Ĭ�ϳ���ʱ��
#define SHIFT_LONG_PRESS_MS     500   // SHIFT����ʱ��
#define FUNCTION_KEY_PRESS_MS   300   // ���ܼ�����ʱ��
/* ���Ʋ������� ------------------------------------------------------------*/
#define LONG_PRESS_TIME       600     // �����ж�ʱ��(ms)
#define DEBOUNCE_TIME         10      // ����ʱ��(ms)
#define MICRO_SENSITIVITY     0.4f    // CTRL΢��������ϵ��
#define BOOST_FACTOR          1.2f    // SHIFT���ٱ���
#define NORMAL_DECAY          0.85f   // ����˥��ϵ��
#define MICRO_DECAY           0.95f   // ΢��ģʽ˥��ϵ��
#define DEAD_ZONE             5.0f    // �ٶ�����(mm/s)

pump_mode_e pump_mode = PUMP_INIT;

// ȫ�ּ��̿��ƶ�����
keyboard_control_t keyboard = {
        .vx = 0, .vy = 0, .vw = 0,
        .max_spd = 3000,
        .move_mode = NORMAL_MODE,
        .shift = {KEY_RELEASE, 0, 500, 0},   // SHIFT����500ms
        .ctrl  = {KEY_RELEASE, 0, 500, 0},   // CTRL����800ms
        .v     = {KEY_RELEASE, 0, 800, 0},   // V���̰�500ms
        .b     = {KEY_RELEASE, 0, 800, 0},   // V���̰�500ms
        .g     = {KEY_RELEASE, 0, 800, 0},   // G��������Ӧ
        .f     = {KEY_RELEASE, 0, 800, 0},    // F��������Ӧ
        .x     = {KEY_RELEASE, 0, 800, 0},    // F��������Ӧ
        .z     = {KEY_RELEASE, 0, 800, 0},
        .r     = {KEY_RELEASE, 0, 800, 0}    // F��������Ӧ
};

mouse_control_t mouse = {0} ;

void key_state_machine(key_status_t *key, uint8_t key_input)
{
    switch (key->state)
    {
        case KEY_RELEASE:
        {
            if (key_input)
                key->state = KEY_WAIT_EFFECTIVE;
            else
                key->state = KEY_RELEASE;
        } break;

        case KEY_WAIT_EFFECTIVE:
        {
            if (key_input)
                key->state = KEY_PRESS_DOWN;
            else
                key->state = KEY_RELEASE;
        } break;

        case KEY_PRESS_DOWN:
        {
            if (key_input)
            {
                key->state = KEY_PRESS_ONCE;
                // ���ݾ������ѡ����������Ҽ���������
                if (key->state == mouse.lk_state)
                    mouse.lk_cnt = 0;
                else
                    mouse.rk_cnt = 0;
            }
            else
                key->state = KEY_RELEASE;
        } break;

        case KEY_PRESS_ONCE:
        {
            if (key_input)
            {
                if (key->state == mouse.lk_state)
                {
                    if (mouse.lk_cnt++ > LONG_PRESS_TIME )
                        key->state = KEY_PRESS_LONG;
                }
                else
                {
                    if (mouse.rk_cnt++ > LONG_PRESS_TIME )
                        key->state = KEY_PRESS_LONG;
                }
            }
            else
                key->state = KEY_RELEASE;
        } break;

        case KEY_PRESS_LONG:
        {
            if (!key_input)
            {
                key->state = KEY_RELEASE;
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



extern struct arm_cmd_msg arm_cmd;

void PC_keyboard_mouse(const pc_control_t *pc_control)
{

    key_state_machine(&keyboard.x, pc_control->keyboard.bit.X);
    key_state_machine(&keyboard.z, pc_control->keyboard.bit.Z);
//    pump_control(keyboard.x);
    // X�������ڴ�����
    if(keyboard.x.state == KEY_PRESS_ONCE) {
        pump_mode = PUMP_OPEN;
    }
    // Z�������ڹر�����
    if(keyboard.z.state == KEY_PRESS_ONCE) {
        pump_mode = PUMP_CLOSE;
    }


    key_state_machine(&keyboard.g,pc_control->keyboard.bit.G);
    if (keyboard.g.state == KEY_PRESS_ONCE)
    {
        cmd_chassis.last_mode= cmd_chassis.ctrl_mode;
        cmd_chassis.ctrl_mode =CHASSIS_RELAX;
    }

    key_state_machine(&keyboard.f,pc_control->keyboard.bit.F);
    if (keyboard.f.state == KEY_PRESS_ONCE)
    {
        cmd_chassis.last_mode= cmd_chassis.ctrl_mode;
        cmd_chassis.ctrl_mode=CHASSIS_ENABLE;
    }

    key_state_machine(&keyboard.b,pc_control->keyboard.bit.B);
    if (keyboard.b.state == KEY_PRESS_ONCE)
    {
        arm_cmd.last_mode = arm_cmd.ctrl_mode;
        arm_cmd.ctrl_mode = ARM_DISABLE;
    }

    key_state_machine(&keyboard.v,pc_control->keyboard.bit.V);
    if (keyboard.v.state == KEY_PRESS_ONCE)
    {
        arm_cmd.last_mode = arm_cmd.ctrl_mode;
        arm_cmd.ctrl_mode = ARM_ENABLE;
    }

    /* ģʽ���ȼ����� */
    keyboard.move_mode = NORMAL_MODE;
    keyboard.max_spd = 2500;
    // �ȴ���CTRL
    key_state_machine(&keyboard.ctrl, pc_control->keyboard.bit.CTRL);
    if(keyboard.ctrl.state == KEY_PRESS_LONG) {
        keyboard.move_mode = SLOW_MODE;
        keyboard.max_spd = 1500;
    }
    // ����SHIFT���������ȼ���
    key_state_machine(&keyboard.shift, pc_control->keyboard.bit.SHIFT);
    if(keyboard.shift.state == KEY_PRESS_LONG) {
        keyboard.move_mode = FAST_MODE;
        keyboard.max_spd = 3500;
    }

    /* ���㶯̬���� */
    float delta = (keyboard.move_mode == FAST_MODE) ?
                  (base_delta * BOOST_FACTOR) :
                  (keyboard.move_mode == SLOW_MODE) ?
                  (base_delta * MICRO_SENSITIVITY) :
                  base_delta;

    float decay = (keyboard.move_mode == SLOW_MODE) ?
                  MICRO_DECAY : NORMAL_DECAY;


    // ǰ����W/S -> vy��
    if(pc_control->keyboard.bit.W) {
        keyboard.vx += delta;
    } else if(pc_control->keyboard.bit.S) {
        keyboard.vx -= delta;
    } else {
        keyboard.vx *= (1 - km_vy_ramp->calc(km_vy_ramp) * decay);
    }

    // ���ҷ���A/D -> vx��
    if(pc_control->keyboard.bit.A) {
        keyboard.vy -= delta;
    } else if(pc_control->keyboard.bit.D) {
        keyboard.vy += delta;
    } else {     //TODO: ����б�º�����ת����ɼ���б�º��������ٽ׶β�ʹ��б�º���
        keyboard.vy *= (1 - km_vy_ramp->calc(km_vy_ramp) * decay);
    }

    // ��ת���ƣ�Q/E��
    if(pc_control->keyboard.bit.Q) {
        keyboard.vw -= base_delta_w * 1.0f;  // ��ת������ϵ��
    } else if(pc_control->keyboard.bit.E) {
        keyboard.vw += base_delta_w * 1.0f;
    } else {
        keyboard.vw *= (1 - km_vw_ramp->calc(km_vw_ramp) * decay);
    }

//    // ��������
//    if(fabs(keyboard.vx) < DEAD_ZONE) keyboard.vx = 0;
//    if(fabs(keyboard.vy) < DEAD_ZONE) keyboard.vy = 0;
//    if(fabs(keyboard.vw) < DEAD_ZONE) keyboard.vw = 0;


    VAL_LIMIT(keyboard.vx, -keyboard.max_spd, keyboard.max_spd);
    VAL_LIMIT(keyboard.vy, -keyboard.max_spd, keyboard.max_spd);
    VAL_LIMIT(keyboard.vw, -keyboard.max_spd, keyboard.max_spd);

    VAL_LIMIT(keyboard.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
    VAL_LIMIT(keyboard.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
    VAL_LIMIT(keyboard.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);



//    key_state_machine(&keyboard.r,pc_control->keyboard.bit.R);
//    if (keyboard.r.state == KEY_PRESS_ONCE)
//    {
//        arm_cmd.last_mode = arm_cmd.ctrl_mode;
//        arm_cmd.ctrl_mode = ARM_INIT;
//    }
}

