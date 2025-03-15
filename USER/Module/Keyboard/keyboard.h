//
// Created by ���ο� on 25-3-8.
//

#ifndef CTRBOARD_H7_ALL_KEYBOARD_H
#define CTRBOARD_H7_ALL_KEYBOARD_H
#include "FreeRTOS.h"
#include "referee_system.h"
/**
  * @brief     �����˶��ٶȿ���ģʽ
  */
typedef enum
{
    NORMAL_MODE = 0,    //����ģʽ
    FAST_MODE,          //����ģʽ
    SLOW_MODE,          //����ģʽ
} key_move_e;

/**
  * @brief     ��갴��״̬����ö��
  */
typedef enum
{
    KEY_RELEASE = 0,    //û�а�������
    KEY_WAIT_EFFECTIVE, //�ȴ�����������Ч������
    KEY_PRESS_ONCE,     //��������һ�ε�״̬
    KEY_PRESS_DOWN,     //�����Ѿ�������
    KEY_PRESS_LONG,     //��������״̬
} key_state_e;

/**
  * @brief     ����������ݽṹ��
  */
typedef struct
{
    /* ����ģʽʹ�ܱ�־ */
    uint8_t keyboard_enable;

    /* �����̿���ģʽ�µĵ����ƶ��ٶ�Ŀ��ֵ */
    float vx;          //����ǰ������Ŀ���ٶ�
    float vy;          //��������ƽ��Ŀ���ٶ�
    float vw;          //������ת�ٶ�
    float max_spd;     //�˶�����ٶ�

    /* ���̰���״̬ */
    key_state_e e_state; //E������״̬
    key_state_e f_state; //F������״̬
    key_state_e shift_state; //SHIFT������״̬
    key_state_e v_state; //V������״̬
    key_state_e g_state; //V������״̬

    /* �˶�ģʽ�����̿��Ƶ����˶����� */
    key_move_e move_mode;

} keyboard_control_t;

typedef struct {
    uint8_t mouse_enable;

    /* ���Ұ���״̬ */
    key_state_e lk_state; //��ఴ��״̬
    key_state_e mk_state; //�м䰴��״̬
    key_state_e rk_state; //�Ҳఴ��״̬

    uint16_t lk_cnt;
    uint16_t mk_cnt;
    uint16_t rk_cnt;

} mouse_control_t;

/**
  * @brief �������ң�������ݽṹ��
  */
typedef struct
{
    /* PC ������� */
    struct
    {
        int16_t x;   // ���ˮƽƽ��,��ֵ��ʶ�����ƶ�
        int16_t y;   // ��괹ֱ�ƶ�,��ֵ��ʶ�����ƶ�
        int16_t z;   // �����ֹ���,��ֵ��ʶ������
        uint8_t l;   // ��������1Ϊ���£�0Ϊ�ɿ�
        uint8_t r;   // ����Ҽ���1Ϊ���£�0Ϊ�ɿ�
    } mouse;

    /* PC ���̰������� */
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
