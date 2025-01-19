/*
* Change Logs:
* Date            Author          Notes
* 2023-08-23      ChuShicheng     first version
*/
#include <string.h>
#include "dj_motor.h"
#include <cmsis_os.h>
#include <stm32h7xx_hal.h>
#include <malloc.h>
#include "motor_def.h"
#include "rm_config.h"
#include "bsp_fdcan.h"

#define DJI_MOTOR_CNT 14             // 默认波特率下，实测挂载电机极限数量

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static dji_motor_object_t *dji_motor_obj[DJI_MOTOR_CNT] = {NULL};

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

// TODO: 0x2ff容易发送失败

// dji 电机同一组id的can控制帧
struct dji_msg
{
    uint32_t id;
    uint8_t data[8];
};



/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在dji_motor_control()使用
 *        flag的初始化在 motor_send_grouping()中进行,012为底盘can,345为云台can
 */
static uint8_t sender_enable_flag[6] = {0};

/**
 * @brief 根据返回的can_object对反馈报文进行解析
 *
 * @param object 收到数据的object,通过遍历与所有电机进行对比以选择正确的实例
 */
static void decode_dji_motor(dji_motor_object_t *motor, uint8_t *data)
{
    uint8_t *rxbuff = data;
    dji_motor_measure_t *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

//    rt_timer_start(motor->timer);  // 判定电机未离线，重置电机定时器

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_rpm = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_rpm + SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
int dji_motot_rx_callback(uint32_t id, uint8_t *data){
    // fdcanx_receive(&hfdcan1, &id, data);
    // 找到对应的实例后再调用decode_dji_motor进行解析
    for (size_t i = 0; i < idx; ++i)
    {
        if (dji_motor_obj[i]->rx_id == id)
        {
            decode_dji_motor(dji_motor_obj[i], data);
            return 0;
        }
    }
    return -1; // 未找到对应的电机实例
}

void dji_motor_relax(dji_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void dji_motor_enable(dji_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

// 运算所有电机实例的控制器,发送控制报文
void dji_motor_control()
{
    dji_motor_object_t *motor;
    dji_motor_measure_t measure;
    int16_t set; // 电机控制器计算得到的控制参数
    uint8_t id;  // 1~4 用于装填多电机CAN控制报文
    uint8_t data_buf[8];  // 用于多电机模式下合并一帧CAN报文

    // 遍历所有电机实例,运行控制算法并填入报文
    // 遍历所有电机实例,运行控制算法并填入报文
    for (size_t i = 0; i < idx; ++i)
    {
        motor = dji_motor_obj[i];
        id = motor->rx_id - 0x201;     // 对应多电机模式下的ID转换规则
        measure = motor->measure;
        set = motor->control(measure); // 调用对接的电机控制器计算
        //LIMIT_MIN_MAX(set,  -2000,  2000);

        // 合并报文
        if (motor->stop_flag == MOTOR_STOP)
        {
            data_buf[id*2] = 0;
            data_buf[id*2 + 1] = 0;
        }
        else
        {
            data_buf[id*2] = (uint8_t) (set >> 8);
            data_buf[id*2 + 1] = (uint8_t) (set & 0x00ff);
        }
        // 发送报文
        if(i == idx - 1)
            CAN_send(&hfdcan1, 0x200, data_buf);
    }
}

/**
 * @brief 电机初始化,返回一个电机实例
 * @param config 电机配置
 * @return dji_motor_object_t* 电机实例指针
 */
dji_motor_object_t *dji_motor_register(motor_config_t *config, void *control)
{
    dji_motor_object_t *object = (dji_motor_object_t *)user_malloc(sizeof(dji_motor_object_t));
    memset(object, 0, sizeof(dji_motor_object_t));

    // 对接用户配置的 motor_config
    object->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    object->rx_id = config->rx_id;
    object->tx_id = config->tx_id; // 电机接收报文的ID
    object->control = control;                                       // 电机控制器执行

    // 电机挂载CAN总线
    object->can_id = config->can_id;
    switch (config->can_id)
    {
        case 1:
            object->can = &hfdcan1;
            break;
        case 2:
            object->can = &hfdcan2;
            break;
        default:
            break;
    }

    dji_motor_enable(object);
    dji_motor_obj[idx++] = object;
    return object;
}