//
// Created by ���ο� on 25-4-11.
//

#ifndef CTRBOARD_H7_ALL_QUATERNIONEKF_H
#define CTRBOARD_H7_ALL_QUATERNIONEKF_H
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/**
 * @struct QEKF_INS_t
 * @brief ��չ�������˲�����EKF����̬������Ľṹ��
 *        �洢���������ݡ��˲���״̬���㷨�������м������
 */
typedef struct
{
    uint8_t Initialized;                  /**< ��ʼ����ɱ�־��1=�ѳ�ʼ����0=δ��ʼ�� */

    KalmanFilter_t IMU_QuaternionEKF;     /**< �������˲���ʵ����
                                             *   - ״̬����ά�ȣ�6��ǰ4ά��Ԫ������2ά��������ƫ��
                                             *   - ��������ά�ȣ�3�����ٶȼƵ�λ���������������
                                             */

    uint8_t ConvergeFlag;                 /**< �˲���������־��1=��������������ͨ������0=δ���� */
    uint8_t StableFlag;                   /**< �����˶��ȶ��Ա�־��1=�ȶ����ͽ��ٶ�+�������ٶȣ���0=�����˶� */

    uint64_t ErrorCount;                   /**< ��������ʧЧ���������ڼ���˲����Ƿ�ɢ��������ֵʱ��������״̬�� */
    uint64_t UpdateCount;                  /**< ��̬�������ڼ��������ڳ�ʼ����ͨ�˲������״����� */

    float q[4];                           /**< ��Ԫ����̬����ֵ����λ��Ԫ������˳��[q_w, q_x, q_y, q_z]����ʾ��������ڵ�������ϵ����ת */
    float GyroBias[3];                    /**< ��������ƫ����ֵ����λ��rad/s����
                                             *   - GyroBias[0] = x����ƫ��GyroBias[1] = y����ƫ��z����ƫ�̶�Ϊ0����yaw���ɹ۲⣩
                                             */

    float Gyro[3];                        /**< ��ƫ����������������ݣ���λ��rad/s����[gx - GyroBias[0], gy - GyroBias[1], gz] */
    float Accel[3];                       /**< ��ͨ�˲���ļ��ٶȼ����ݣ���λ��m/s?��������������µ�ƽ�������� */

    float OrientationCosine[3];           /**< �������������������������ֵ��0~1����
                                             *   - ��������Ӧ������������Ƹ���/��ת�ӽ�90��ʱ���쳣����
                                             */

    float accLPFcoef;                     /**< ���ٶȼƵ�ͨ�˲�ϵ����
                                             *   - �˲���ʽ��y = (accLPFcoef/(dt + accLPFcoef)) * y_prev + (dt/(dt + accLPFcoef)) * x
                                             *   - 0��ʾ���˲���ֱ��ʹ��ԭʼ����
                                             */
    float gyro_norm;                      /**< ���������ݷ�������λ��1/rad/s������(gx? + gy? + gz?)�������жϽ��ٶȴ�С */
    float accl_norm;                      /**< ���ٶȼ����ݷ�������λ��m/s?������(ax? + ay? + az?)�������ж��Ƿ�ӽ��������ٶ� */
    float AdaptiveGainScale;              /**< ����Ӧ�����������ӣ����ݿ����������������������棬��Χ0~1 */

    float Roll;                           /**< �����ǣ�Roll������x����ת�Ƕȣ���λ���㣩������Ԫ������õ� */
    float Pitch;                          /**< �����ǣ�Pitch������y����ת�Ƕȣ���λ���㣩������Ԫ������õ� */
    float Yaw;                            /**< ƫ���ǣ�Yaw������z����ת�Ƕȣ���λ���㣩������Ԫ������õ���-180��~180�㣩 */

    float YawTotalAngle;                  /**< �ۼ�ƫ���ǣ���λ���㣩��������360���ѭ�����⣬����������ת��������YawTotalAngle = 360��*YawRoundCount + Yaw�� */

    float Q1;                             /**< ��Ԫ�����¹�������������
                                             *   - ��ģ�����ǰ���������̬��Ӱ�죨������������Q�ĶԽ���ǰ4ά��
                                             *   - ȡֵԽ���˲����������Ƕ������������̶�Խ��
                                             */
    float Q2;                             /**< ��������ƫ��������������
                                             *   - ��ģ��ƫ����ʱ�����ԣ��������ģ�ͣ�������������Q�ĶԽ��ߺ�2ά��
                                             *   - ȡֵԽС����ƫ����Խƽ��
                                             */
    float R;                              /**< ���ٶȼ���������������
                                             *   - ������������R�ĶԽ���Ԫ�أ�ȡֵԽ���˲����Լ��ٶȼ����ݵ����ζ�Խ��
                                             *   - �˶������������Ա�������˶�ʱ�������
                                             */

    float dt;                             /**< ��̬�������ڣ���λ��s�������θ���֮���ʱ������������ɢ��״̬ת�ƾ������������ */

    mat ChiSquare;                        /**< ��������������1x1�����洢ͳ������? = (z - h(x'))^T * S?? * (z - h(x')) */
    float ChiSquare_Data[1];              /**< �����������ݴ洢���飺���ھ�������ӿڼ��� */
    float ChiSquareTestThreshold;         /**< ����������ֵ������?������ֵ����Ϊ���ⲻ���ţ��������»����÷�ɢ���� */
    float lambda;                         /**< �������ӣ��������ӣ���
                                             *   - ȡֵ��Χ��0 < lambda �� 1������ƫЭ����������ָ��˥����P_b /= lambda��
                                             *   - �ӽ�1ʱ�ʺ�ƽ�ȳ������ӽ�0ʱ�ʺϷ�ƽ�ȶ�̬����
                                             */

    int16_t YawRoundCount;                /**< ƫ����ѭ����������¼Yaw������180��Ĵ��������ڼ����ۼ�ƫ����YawTotalAngle */

    float YawAngleLast;                   /**< ��һ����ƫ���ǣ���λ���㣩�����ڼ��Yaw�Ƿ��Խ��180��߽磬����YawRoundCount */

} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;

extern float chiSquare;
extern float ChiSquareTestThreshold;

void IMU_QuaternionEKF_Init(
        float* init_quaternion,    // ��ʼ��Ԫ��
        float process_noise1,      // ��Ԫ���������� Q1
        float process_noise2,      // ��������ƫ�������� Q2
        float measure_noise,       // ���ٶȼ��������� R
        float lambda,              // ��������
        float lpf                  // ���ٶȼƵ�ͨ�˲�ϵ��
);
void IMU_QuaternionEKF_Update(
        float gx, float gy, float gz,  // ������ԭʼ���ݣ�rad/s��
        float ax, float ay, float az,  // ���ٶȼ�ԭʼ���ݣ�m/s?��
        float dt                       // �������ڣ�s��
);
#endif //CTRBOARD_H7_ALL_QUATERNIONEKF_H
