/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */
#include "ins_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "main.h"
#include "pid.h"
#include "BMI088driver.h"
#include "tim.h"
#include "drv_dwt.h"
#include "user_lib.h"
#include "robot_task.h"
#include "QuaternionEKF.h"
#include "msg_freertos.h"
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static struct ins_msg ins_publish_data;

static publisher_t *ins_topic_publish;

static void ins_topic_publish_init(void);
static void ins_topic_subscribe_init(void);
static void ins_topic_publish_push(void);
static void ins_topic_subscribe_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t ins_task_dwt = 0;   // 毫秒监测
static float ins_task_dt = 0;       // 线程实际运行时间dt
static float ins_task_delta = 0;    // 监测线程运行时间
static float ins_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

static void ins_init(void);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
static void IMU_Param_Correction(imu_param_t *param, float gyro[3], float accel[3]);
static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
static void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
static void InitQuaternion(float *init_q4);

/* ----------------------------- IMU_TEMPRETURE ----------------------------- */
#define IMU_TARGET_TEMP           32            /* imu期望恒温温度 */  // 依据当天环境温度调整，同时需要调整PID的参数

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;

    switch (Channel)
    {
        case TIM_CHANNEL_1:
            tim_pwmHandle->Instance->CCR1 = value;
            break;
        case TIM_CHANNEL_2:
            tim_pwmHandle->Instance->CCR2 = value;
            break;
        case TIM_CHANNEL_3:
            tim_pwmHandle->Instance->CCR3 = value;
            break;
        case TIM_CHANNEL_4:
            tim_pwmHandle->Instance->CCR4 = value;
            break;
    }
}

static uint32_t period = 250000;
static uint32_t pulse = 0;
static pid_obj_t *imu_temp_pid;
static pid_config_t imu_temp_config = INIT_PID_CONFIG(200, 3, 0, 10, 500, PID_Integral_Limit);

/* ----------------------------- IMU_TEMPRETURE ----------------------------- */

/* ---------------------------- Attitude_Solving ---------------------------- */
#define X 0
#define Y 1
#define Z 2
static ins_t ins;
static imu_param_t imu_param;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

extern ImuDataTypeDef BMI088;


/* -------------------------------- 线程入口 ------------------------------- */
void InsTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};// 取决于海口地区g值

    imu_temp_pid = pid_register(&imu_temp_config);   /* 注册 PID 实例 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    BMI088_Read(&BMI088);
    ins_init();

    uint32_t ins_wake_time = osKernelSysTick();  // 获取该ins_tak开始的第一次时间
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
    ins_topic_publish_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    ins_task_dt = dwt_get_delta(&ins_task_dwt);
    ins_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        ins_task_delta = dwt_get_time_ms() - ins_task_start_dt;
        ins_task_start_dt = dwt_get_time_ms();
        ins_task_dt = dwt_get_delta(&ins_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        BMI088_Read(&BMI088);
        ins.accel[0] = BMI088.accel[0];
        ins.accel[1] = BMI088.accel[1];
        ins.accel[2] = BMI088.accel[2];
        ins.gyro[0] = BMI088.gyro[0];
        ins.gyro[1] = BMI088.gyro[1];
        ins.gyro[2] = BMI088.gyro[2];
        IMU_Param_Correction(&imu_param, ins.gyro, ins.accel);
        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(ins.gyro[X], ins.gyro[Y], ins.gyro[Z], ins.accel[X], ins.accel[Y], ins.accel[Z], ins_task_dt);

        memcpy(ins.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, ins.xn, ins.q);
        BodyFrameToEarthFrame(yb, ins.yn, ins.q);
        BodyFrameToEarthFrame(zb, ins.zn, ins.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, ins.q);
        for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
        {
            ins.motion_accel_b[i] = (ins.accel[i] - gravity_b[i]) * ins_task_dt / (ins.accel_lpf + ins_task_dt) + ins.motion_accel_b[i] * ins.accel_lpf / (ins.accel_lpf + ins_task_dt);
        }
        BodyFrameToEarthFrame(ins.motion_accel_b, ins.motion_accel_n, ins.q); // 转换回导航系n

        ins.yaw = QEKF_INS.Yaw;
        ins.pitch = QEKF_INS.Pitch;
        ins.roll = QEKF_INS.Roll;
        ins.yaw_total_angle = QEKF_INS.YawTotalAngle;

        {/* publish msg */
            // FIXME:/* 根据陀螺仪安装情况进行调整 */
            // TODO:右手系，逆时针旋转为正
            ins_publish_data.yaw = ins.yaw;
            ins_publish_data.roll = -ins.roll;
            ins_publish_data.yaw_total_angle = ins.yaw_total_angle;
            ins_publish_data.pitch = ins.pitch;
            ins_publish_data.gyro[0] = ins.gyro[0];
            ins_publish_data.gyro[1] = ins.gyro[1];
            ins_publish_data.gyro[2] = ins.gyro[2];
            ins_publish_data.accel[0] = ins.accel[0];
            ins_publish_data.accel[1] = ins.accel[1];
            ins_publish_data.accel[2] = ins.accel[2];
            ins_publish_data.motion_accel_b[0] = ins.motion_accel_b[0];
            ins_publish_data.motion_accel_b[1] = ins.motion_accel_b[1];
            ins_publish_data.motion_accel_b[2] = ins.motion_accel_b[2];
            ins_topic_publish_push();
        }

        // temperature control
        if ((count % 2) == 0)
        {
            // 1khz采样率，每2ms更新一次temperature
            pulse = pid_calculate(imu_temp_pid, BMI088.temperature, IMU_TARGET_TEMP);
            TIM_Set_PWM(&htim3, TIM_CHANNEL_4, pulse);
        }

        count++;
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelayUntil(&ins_wake_time, 1);// 使用ins_wake_time执行严格的1ms周期延时，不允许无节制运行，只能1ms周期运行
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static void ins_topic_publish_init(void)
{
    ins_topic_publish = pub_register("ins_pub",sizeof(struct ins_msg));
}

static void ins_topic_subscribe_init(void)
{

}

static void ins_topic_publish_push(void)
{
    pub_push_msg(ins_topic_publish,&ins_publish_data);
}

static void ins_topic_subscribe_pull(void)
{

}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

/**
 * @brief 初始化 ins 解算系统
 *
 */
static void ins_init(void)
{
    imu_param.scale[X] = 1;
    imu_param.scale[Y] = 1;
    imu_param.scale[Z] = 1;
    imu_param.yaw = 0;
    imu_param.pitch = 0;
    imu_param.roll = 0;
    imu_param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);

    //TODO：需要调节的BMI088参数
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);

    // noise of accel is relatively big and of high freq,thus lpf is used
    ins.accel_lpf = 0.0085;
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        acc_init[0] += BMI088.accel[0];
        acc_init[1] += BMI088.accel[1];
        acc_init[2] += BMI088.accel[2];
        dwt_delay_ms(1);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(imu_param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->yaw - lastYawOffset) > 0.001f ||
        fabsf(param->pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->yaw;
    lastPitchOffset = param->pitch;
    lastRollOffset = param->roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief 四元数更新函数,即实现dq/dt=0.5Ωq
 *
 * @param q  四元数
 * @param gx
 * @param gy
 * @param gz
 * @param dt 距离上次调用的时间间隔
 */
static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
static void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
