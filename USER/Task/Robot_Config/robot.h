//
// Created by ���ο� on 25-4-21.
//

#ifndef CTRBOARD_H7_ALL_ROBOT_H
#define CTRBOARD_H7_ALL_ROBOT_H

#define CPU_FREQUENCY 480     /* CPU��Ƶ(mHZ) */

#include "stm32h7xx_hal.h" // ʹ�õ�оƬ
#include "cmsis_os.h" // ʹ�õ� OS ͷ�ļ�

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free vPortFree
#else
#define user_malloc malloc
#define user_malloc free
#endif

/* ���̺���̨�ֱ��Ӧ�� can ���� */
#define CAN_CHASSIS    hfdcan1
#define CAN_GIMBAL     hfdcan2
#define CAN_CHASSIS_NAME    "hfdcan1"
#define CAN_GIMBAL_NAME     "hfdcan2"
/* �����������ص� i2c �豸����(���i2c) */
#define I2C_MAG        "i2c1"    //"Notice: PA8 --> 8; PC9 --> 41"

/* �����������ص� SPI �豸���Ƽ� CS ���� */
#define SPI_GYRO       "spi1"
#define SPI_GYRO_CS    16
/* ���ٶȼ������ص� SPI �豸���Ƽ� CS ���� */
#define SPI_ACC        "spi1"
#define SPI_ACC_CS     4

/* ң���������ص� usart �豸���� */
#define USART_RC       "uart3"

/* ---------------------------------- ��˹ң������� --------------------------------- */
#define RC_MAX_VALUE      784.0f  /* ң����ͨ�����ֵ */
/* ң����ģʽ�µĵ�������ٶȱ��� */ // TODO����ʱ��ʾң���������ֵ��Ҳֻ�ܵ���1.0f m/s���ٶ�
/* ����ǰ���ٶȱ���(m/s) */
#define CHASSIS_RC_MOVE_RATIO_X 2.0f
/* ����ƽ���ٶȱ���(m/s) */
#define CHASSIS_RC_MOVE_RATIO_Y 2.0f
/* ������ת�ٶȱ���(rad/s) */
#define CHASSIS_RC_MOVE_RATIO_W 5.0f

/* ������ģʽ�µĵ�������ٶȱ��� */  //TODO��ȡ���ڼ����ָ�
#define CHASSIS_PC_MOVE_RATIO_X 1.0f
/* ����ƽ���ٶȱ���(m/s) */
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f
/* ������ת�ٶȱ���(rad/s) */
#define CHASSIS_PC_MOVE_RATIO_W 1.0f

/* ---------------------------------- ������� ---------------------------------- */
/* �������Ӱ뾶(m) */
#define WHEEL_RADIUS       0.0762f
/* �����־�(m) ���������ľ� */
#define WHEEL_TRACK        0.380f
/* �������(m) ǰ�������ľ� */
#define WHEEL_BASE         0.456f
/* �������Ӱ�װ�Ƕ�(��) */
#define WHEEL_ANGLE 45.0f   // ���Ӱ�װ�Ƕȣ��ȣ�����45�ȣ�
/* ���������ܳ�(m) */
#define WHEEL_PERIMETER   0.4788f
/* ���̿������ڣ�ms�� */
#define CHASSIS_PERIOD     1.0f //(ms)
/* ����3508������ٱ� */
#define CHASSIS_MOTOR_REDUCTION_RATIO 19.2032f    // 3591/187�����������֮�ȣ�����ת��19Ȧ���ת��1Ȧ
/* ���̵��ת��ת��ϵ�� */
#define WHELL_RPM_RATIO 2406.416f // (60.0f * CHASSIS_MOTOR_REDUCTION_RATIO / WHEEL_PERIMETER)

/******** ��������ٶ����� *******/
/* �����ƶ�����ٶȣ���λ��m/s */
#define MAX_CHASSIS_VX_SPEED 2
#define MAX_CHASSIS_VY_SPEED 2

#define MAX_CHASSIS_VX_SPEED_HIGH 11
#define MAX_CHASSIS_VY_SPEED_HIGH 11

#define MAX_CHASSIS_VX_SPEED_LOW 5
#define MAX_CHASSIS_VY_SPEED_LOW 5

/* ������ת����ٶȣ���λ��rad/s */
#define MAX_CHASSIS_VW_SPEED 5.0f

/* --------------------------------- ���̵������PID���� -------------------------------- */
/* ����ٶȻ���ʵ����C620�ĵ�������������ֵ��Χ��-16380~0~16380����-20A~0A~20A�� */
// ������ת�ٲ���ֵ��ת�ӵģ��������ٻ�����ͨ��Ԥ��ת��ת�٣������Ӧ��Ԥ�ڵ���ֵ��
// �൱����������������ٶ�������������õ��ĵ���ٶȣ���PID��������PWMֵ��
#define CHASSIS_KP_V_MOTOR              3.5f
#define CHASSIS_KI_V_MOTOR              0.01
#define CHASSIS_KD_V_MOTOR              0.001
#define CHASSIS_INTEGRAL_V_MOTOR        1500
#define CHASSIS_MAX_V_MOTOR             12000    // 16000

/* ����ǶȻ� */



// --------------------- ��̼ƿ��Ʋ����궨�� ---------------------
// Ŀ��㶨��
#define WAYPOINT_A_X          0.0f    // ��A����X���ף�
#define WAYPOINT_B_X          2.0f    // ��B����X���ף�
#define POSITION_TOLERANCE    0.02f   // ����Ŀ����λ��������̶ȣ��ף�

// λ�ÿ���PID�������⻷��
#define POSITION_KP           1.5f    // ����ϵ��
#define POSITION_KI           0.05f   // ����ϵ��
#define POSITION_KD           0.2f    // ΢��ϵ��
#define POSITION_MAX_OUTPUT   (CHASSIS_MAX_V_MOTOR * 0.8f)  // �����⻷����ٶ�

// ����ǿ���PID����
#define YAW_KP                0.8f    // �������ϵ��
#define YAW_KI                0.0f    // �������ϵ��
#define YAW_KD                0.1f    // ����΢��ϵ��




/* -------------------------------- ����IMU�ĵ��������PID���� ------------------------------- */
/* imu�ٶȻ� */
#define YAW_KP_V_IMU             5000
#define YAW_KI_V_IMU             200
#define YAW_KD_V_IMU             10
#define YAW_INTEGRAL_V_IMU       1000
#define YAW_MAX_V_IMU            30000
/* imu�ǶȻ� */
#define YAW_KP_A_IMU             0.35f
#define YAW_KI_A_IMU             0
#define YAW_KD_A_IMU             0.001f
#define YAW_INTEGRAL_A_IMU       5
#define YAW_MAX_A_IMU            25
/* auto�ٶȻ� */
#define YAW_KP_V_AUTO            0
#define YAW_KI_V_AUTO            0
#define YAW_KD_V_AUTO            0
#define YAW_INTEGRAL_V_AUTO      0
#define YAW_MAX_V_AUTO           0
/* auto�ǶȻ� */
#define YAW_KP_A_AUTO            0
#define YAW_KI_A_AUTO            0
#define YAW_KD_A_AUTO            0
#define YAW_INTEGRAL_A_AUTO      0
#define YAW_MAX_A_AUTO           0

/* PITCH����PID���� */
/* imu�ٶȻ� */
#define PITCH_KP_V_IMU           4250
#define PITCH_KI_V_IMU           1000
#define PITCH_KD_V_IMU           3
#define PITCH_INTEGRAL_V_IMU     1500
#define PITCH_MAX_V_IMU          20000
/* imu�ǶȻ� */
#define PITCH_KP_A_IMU           0.5f
#define PITCH_KI_A_IMU           0.0f
#define PITCH_KD_A_IMU           0.005f
#define PITCH_INTEGRAL_A_IMU     0.2f
#define PITCH_MAX_A_IMU          20
/* auto�ٶȻ� */
#define PITCH_KP_V_AUTO          0
#define PITCH_KI_V_AUTO          0
#define PITCH_KD_V_AUTO          0
#define PITCH_INTEGRAL_V_AUTO    0
#define PITCH_MAX_V_AUTO         0
/* auto�ǶȻ� */
#define PITCH_KP_A_AUTO          0
#define PITCH_KI_A_AUTO          0
#define PITCH_KD_A_AUTO          0
#define PITCH_INTEGRAL_A_AUTO    0
#define PITCH_MAX_A_AUTO         0


void robot_init(void);

#endif //CTRBOARD_H7_ALL_ROBOT_H
