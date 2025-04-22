/**
 ******************************************************************************
 * @file    kalman filter.h
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   �������˲���ͷ�ļ�
 ******************************************************************************
 * @attention
 * ���ļ������������˲����������ݽṹ����ͽӿں�������
 ******************************************************************************
 */

#ifndef CTRBOARD_H7_ALL_KALMAN_FILTER_H
#define CTRBOARD_H7_ALL_KALMAN_FILTER_H

// ʹ��ARM Cortex-M4 DSP��ѧ��
/*
#define __CC_ARM    // Keil
#define ARM_MATH_CM4
#define ARM_MATH_MATRIX_CHECK
#define ARM_MATH_ROUNDING
#define ARM_MATH_DSP    // ��arm_math.h�ж���
*/

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "stdint.h"
#include "stdlib.h"

// �ڴ���������ã�����CMSIS-RTOS��
#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

// ������������ã�ʹ�ø������㣩
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32      // �����ʼ��
#define Matrix_Add arm_mat_add_f32        // ����ӷ�
#define Matrix_Subtract arm_mat_sub_f32   // �������
#define Matrix_Multiply arm_mat_mult_f32  // ����˷�
#define Matrix_Transpose arm_mat_trans_f32// ����ת��
#define Matrix_Inverse arm_mat_inverse_f32// ��������

/**
 * @brief �������˲����������ݽṹ
 * @note �����˲�������״̬���������м������
 */
typedef struct kf_t
{
    // ��������ָ��
    float *FilteredValue;    // �˲����״̬����ֵ���
    float *MeasuredVector;   // �����������뻺����
    float *ControlVector;    // �����������뻺����

    // ά�Ȳ���
    uint8_t xhatSize;        // ״̬����ά��
    uint8_t uSize;           // ��������ά��
    uint8_t zSize;           // �����������ά��

    // �Զ�������ز���
    uint8_t UseAutoAdjustment;   // �Զ��������ܿ���
    uint8_t MeasurementValidNum; // ��ǰ��Ч��������

    // �������ò���
    uint8_t *MeasurementMap;      // ����-״̬ӳ���ָ��ÿ�������Ӧ��״̬��ţ�
    float *MeasurementDegree;     // ����ϵ����H�����ӦԪ��ֵ��
    float *MatR_DiagonalElements; // �����������R����Խ���Ԫ�أ�
    float *StateMinVariance;      // ״̬��С�������ƣ���ֹЭ�������������
    uint8_t *temp;                // ��ʱ�洢��Ч��������

    // ���㲽����Ʊ�־������û����庯��ʹ��,��Ϊ��־λ�����ж��Ƿ�Ҫ������׼KF����������е�����һ��
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5; // ����������־

    // �����壨ARM��ѧ�����ʵ����
    mat xhat;      // ����״̬���� x(k|k)
    mat xhatminus; // ����״̬���� x(k|k-1)
    mat u;         // ��������
    mat z;         // ��������
    mat P;         // ����Э������� P(k|k)
    mat Pminus;    // ����Э������� P(k|k-1)
    mat F, FT;     // ״̬ת�ƾ�����ת��
    mat B;         // �����������
    mat H, HT;     // ���������ת��
    mat Q;         // ��������Э�������
    mat R;         // ��������Э�������
    mat K;         // �������������
    mat S;         // �м����������
    mat temp_matrix, temp_matrix1; // ��ʱ�������
    mat temp_vector, temp_vector1; // ��ʱ��������

    int8_t MatStatus; // ��������״̬��

    // �û��Զ��庯�����ӣ�������չ��׼�������˲���
    void (*User_Func0_f)(struct kf_t *kf); // ����Ԥ������
    void (*User_Func1_f)(struct kf_t *kf); // ״̬Ԥ�������
    void (*User_Func2_f)(struct kf_t *kf); // Э����Ԥ�������
    void (*User_Func3_f)(struct kf_t *kf); // ������������������
    void (*User_Func4_f)(struct kf_t *kf); // ״̬���º�����
    void (*User_Func5_f)(struct kf_t *kf); // Э������º�����
    void (*User_Func6_f)(struct kf_t *kf); // ���ս��������

    // �������ݴ洢ָ��
    float *xhat_data, *xhatminus_data; // ״̬�����洢
    float *u_data;          // ���������洢
    float *z_data;          // ���������洢
    float *P_data, *Pminus_data; // Э�������洢
    float *F_data, *FT_data;     // ״̬ת�ƾ���洢
    float *B_data;          // ���ƾ���洢
    float *H_data, *HT_data;// �������洢
    float *Q_data;          // ������������洢
    float *R_data;          // ������������洢
    float *K_data;          // ����������洢
    float *S_data;          // �м�������洢
    float *temp_matrix_data, *temp_matrix_data1; // ��ʱ����洢
    float *temp_vector_data, *temp_vector_data1; // ��ʱ�����洢
} KalmanFilter_t;

// ȫ�ֱ������������ڶ�̬�ڴ���䣩
extern uint16_t sizeof_float, sizeof_double;

/* �������� ----------------------------------------------------------------*/

/**
 * @brief �������˲�����ʼ������
 * @param kf        �˲�������ָ��
 * @param xhatSize  ״̬����ά��
 * @param uSize     ��������ά��
 * @param zSize     �����������ά��
 * @note ����ڴ���䡢�����ʼ����Ĭ�ϲ������õ�׼������
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);

/**
 * @brief ��������Ԥ������
 * @param kf �˲�������ָ��
 * @note ִ��������Ч�Լ�顢�Զ�����H/R����ά�ȡ����ݸ�ʽת��
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf);

/**
 * @brief ����״̬���Ƹ��£�Ԥ�ⷽ�̣�
 * @param kf �˲�������ָ��
 * @note ����x(k|k-1) = F��x(k-1|k-1) + B��u(k-1)
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);

/**
 * @brief ����Э���������£�Ԥ�ⷽ�̣�
 * @param kf �˲�������ָ��
 * @note ����P(k|k-1) = F��P(k-1|k-1)��F^T + Q
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);

/**
 * @brief �������������
 * @param kf �˲�������ָ��
 * @note ����K(k) = P(k|k-1)��H^T��(H��P(k|k-1)��H^T + R)^-1
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf);

/**
 * @brief ����״̬���Ƹ��£�������£�
 * @param kf �˲�������ָ��
 * @note ����x(k|k) = x(k|k-1) + K(k)��(z(k) - H��x(k|k-1))
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief ����Э���������£�������£�
 * @param kf �˲�������ָ��
 * @note ����P(k|k) = (I - K(k)��H)��P(k|k-1)
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf);

/**
 * @brief �������˲��������º���
 * @param kf �˲�������ָ��
 * @return float* ���ظ��º��״̬����ֵָ��
 * @note ��˳��ִ��Ԥ��͸��²��裬���������Ŀ������˲���ʽ
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif //CTRBOARD_H7_ALL_KALMAN_FILTER_H