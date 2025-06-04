//
// Created by 14685 on 2023/2/5.
//

#include "ins_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "main.h"
#include "pid.h"
#include "BMI088driver.h"
#include "tim.h"
#include "drv_dwt.h"
#include "user_lib.h"
#include "rm_task.h"
#include "QuaternionEKF.h"

static void ins_init(void);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
static void IMU_Param_Correction(imu_param_t *param, float gyro[3], float accel[3]);
static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
static void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
static void InitQuaternion(float *init_q4);

/* ----------------------------- IMU_TEMPRETURE ----------------------------- */
#define IMU_TARGET_TEMP           35            /* imu���������¶� */  // ���ݵ��컷���¶ȵ�����ͬʱ��Ҫ����PID�Ĳ���

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
static struct ins_msg ins_data;  // ���ͳ�ȥ������

static imu_param_t imu_param;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

extern ImuDataTypeDef BMI088;

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
static uint32_t ins_task_dwt = 0;   // ������
static float ins_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float ins_task_delta = 0;    // ����߳�����ʱ��
static float ins_task_start_dt = 0; // ����߳̿�ʼʱ��
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

void InsTask_Entry(void const * argument)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};// ȡ���ں��ڵ���gֵ

    imu_temp_pid = pid_register(&imu_temp_config);   /* ע�� PID ʵ�� */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    BMI088_Read(&BMI088);
    ins_init();

    uint32_t ins_wake_time = osKernelSysTick();  // ��ȡ��ins_tak��ʼ�ĵ�һ��ʱ��
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    ins_task_dt = dwt_get_delta(&ins_task_dwt);
    ins_task_start_dt = dwt_get_time_ms();
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    for(;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        ins_task_delta = dwt_get_time_ms() - ins_task_start_dt;
        ins_task_start_dt = dwt_get_time_ms();

        ins_task_dt = dwt_get_delta(&ins_task_dwt);
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */


        BMI088_Read(&BMI088);
        ins.accel[0] = BMI088.accel[0];
        ins.accel[1] = BMI088.accel[1];
        ins.accel[2] = BMI088.accel[2];
        ins.gyro[0] = BMI088.gyro[0];
        ins.gyro[1] = BMI088.gyro[1];
        ins.gyro[2] = BMI088.gyro[2];
        IMU_Param_Correction(&imu_param, ins.gyro, ins.accel);
        // ���ĺ���,EKF������Ԫ��
        IMU_QuaternionEKF_Update(ins.gyro[X], ins.gyro[Y], ins.gyro[Z], ins.accel[X], ins.accel[Y], ins.accel[Z], ins_task_dt);

        memcpy(ins.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // ����ϵ������ת������������ϵ������ѡȡ����ϵΪ����ϵ
        BodyFrameToEarthFrame(xb, ins.xn, ins.q);
        BodyFrameToEarthFrame(yb, ins.yn, ins.q);
        BodyFrameToEarthFrame(zb, ins.zn, ins.q);

        // �������ӵ�������ϵnת��������ϵb,�����ݼ��ٶȼ����ݼ����˶����ٶ�
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, ins.q);
        for (uint8_t i = 0; i < 3; ++i) // ͬ����һ����ͨ�˲�
        {
            ins.motion_accel_b[i] = (ins.accel[i] - gravity_b[i]) * ins_task_dt / (ins.accel_lpf + ins_task_dt) + ins.motion_accel_b[i] * ins.accel_lpf / (ins.accel_lpf + ins_task_dt);
        }
        BodyFrameToEarthFrame(ins.motion_accel_b, ins.motion_accel_n, ins.q); // ת���ص���ϵn

        ins.yaw = QEKF_INS.Yaw;
        ins.pitch = QEKF_INS.Pitch;
        ins.roll = QEKF_INS.Roll;
        ins.yaw_total_angle = QEKF_INS.YawTotalAngle;

        {/* publish msg */
            // FIXME:/* ���������ǰ�װ������е��� */
            // TODO:����ϵ����ʱ����תΪ��
            ins_data.yaw = ins.yaw;
            ins_data.roll = -ins.roll;
            ins_data.yaw_total_angle = ins.yaw_total_angle;
            ins_data.pitch = ins.pitch;
            ins_data.gyro[0] = ins.gyro[0];
            ins_data.gyro[1] = ins.gyro[1];
            ins_data.gyro[2] = ins.gyro[2];
            ins_data.accel[0] = ins.accel[0];
            ins_data.accel[1] = ins.accel[1];
            ins_data.accel[2] = ins.accel[2];
            ins_data.motion_accel_b[0] = ins.motion_accel_b[0];
            ins_data.motion_accel_b[1] = ins.motion_accel_b[1];
            ins_data.motion_accel_b[2] = ins.motion_accel_b[2];
        }

        // temperature control
        if ((count % 2) == 0)
        {
            // 1khz�����ʣ�ÿ2ms����һ��temperature
            pulse = pid_calculate(imu_temp_pid, BMI088.temperature, IMU_TARGET_TEMP);
            TIM_Set_PWM(&htim3, TIM_CHANNEL_4, pulse);
        }

        count++;
        vTaskDelayUntil(&ins_wake_time, 1);// ʹ��ins_wake_timeִ���ϸ��1ms������ʱ���������޽������У�ֻ��1ms��������
    }
    /* USER CODE END InsTask */
}

/**
 * @brief ��ʼ�� ins ����ϵͳ
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

    //TODO����Ҫ���ڵ�BMI088����
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);

    // noise of accel is relatively big and of high freq,thus lpf is used
    ins.accel_lpf = 0.0085;
}

// ʹ�ü��ٶȼƵ����ݳ�ʼ��Roll��Pitch,��Yaw��0,�������Ա����ڳ�ʼʱ�����̬�������
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // ����ϵ�������ٶ�ʸ��,��һ����Ϊ(0,0,1)
    float axis_rot[3] = {0};           // ��ת��
    // ��ȡ100�μ��ٶȼ�����,ȡƽ��ֵ��Ϊ��ʼֵ
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        acc_init[0] = BMI088.accel[0];
        acc_init[1] = BMI088.accel[1];
        acc_init[3] = BMI088.accel[2];
        dwt_delay_ms(1);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // ����ԭʼ���ٶ�ʸ���͵���ϵ�������ٶ�ʸ���ļн�
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // ��ǹ�ʽ,������Ϊ0(û��z�����)
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
 * @brief reserved.��������IMU��װ��������������,�������������̨��İ�װƫ��
 *
 *
 * @param param IMU����
 * @param gyro  ���ٶ�
 * @param accel ���ٶ�
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
 * @brief ��Ԫ�����º���,��ʵ��dq/dt=0.5��q
 *
 * @param q  ��Ԫ��
 * @param gx
 * @param gy
 * @param gz
 * @param dt �����ϴε��õ�ʱ����
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
