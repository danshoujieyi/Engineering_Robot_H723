/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   �������˲�����C����ʵ��
 ******************************************************************************
 * @attention
 * ���������˲��������ڴ���������Ƶ�ʲ�ͬ������£���̬��������H��R��K��ά������ֵ��
 * ͨ���Զ��������ƣ��ܹ����ݴ�����������Ч�Զ�̬��������ϵͳ���̡�
 *
 * ����H��R�ĳ�ʼ����ʽ�����P��F��Q��ͬ����ʼ����������zʱ����Ҫָ��ÿ������������
 * ��Ӧ��״̬������ź����ⷽʽ��ֱ�Ӳ������Ӳ�������������ο�ʾ������
 *
 * ������Ҫ��̬������������z���ɽ��ṹ���е�Use_Auto_Adjustment��Ϊ0�������ճ��淽ʽ
 * ��ʼ��z��H��R����
 *
 * ��������z���������u���ڴ������ص������и��¡�z�����е�0ֵ��ʾ��������Ч�����ϴ�
 * �˲����º��������ݣ���ÿ���˲����º�z��u�����ᱻ���㡣
 *
 * ��ʵ��ͨ������Э�������P����Сֵ����ֹ�˲��������������������ƫ��������Ʒ���
 * ��ο�ʾ������
 *
 * @ʾ��:
 * ״̬�������壺
 * x = |   �߶�    |
 *     |   �ٶ�    |
 *     | ���ٶ�    |
 *
 * ��ʼ�����̣�
 * - ���ó�ʼЭ�������P��״̬ת�ƾ���F��������������Q
 * - �����Զ���������
 * - ָ���������������Ӧ��״̬����������ѹ�Ʋ�߶�Ӧ�߶�״̬��
 * - ����������������R�ĶԽ���Ԫ��
 * - ����״̬��������С��������
 *
 * ���ݸ��£�
 * - ���������ݵ���ʱ����MeasuredVector��ӦԪ��
 * - ���ڵ���Kalman_Filter_Update�����˲�����
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // ������С����
 *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // �����Զ�����
 *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // ��ѹ��ø߶� GPS��ø߶� ���ٶȼƲ��z���˶����ٶ�
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // ����measurement_reference��measurement_degree����H�������£��ڵ�ǰ����ȫ������������Ч����£�
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //����mat_R_diagonal_elements����R�������£��ڵ�ǰ����ȫ������������Ч����£�
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // ���þ���ֵ
 *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // ѭ������
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // �������ݸ���Ӧ����������ʽ ����MeasuredVector��ֵ
 * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */

#include "kalman_filter.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif


uint16_t sizeof_float, sizeof_double;  // ���ڶ�̬�ڴ����ʱȷ���������ʹ�С

/**
 * @brief �����������H����������R�Ϳ���������K
 * @param kf �������˲�������ָ��
 * @note ���ݵ�ǰ��Ч�������ݶ�̬�������ⷽ��
 */
static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
 * @brief ��ʼ���������˲�������������ڴ�
 * @param kf        �������˲�������ָ��
 * @param xhatSize  ״̬����ά��
 * @param uSize     ���Ʊ���ά��
 * @param zSize     �۲������ά��
 * @note ������¹�����
 * 1. ��ʼ��������ά����Ϣ
 * 2. Ϊ���о�������ڴ�ռ�
 * 3. ��ʼ������Ϊȫ��
 * 4. ����Ĭ��������־
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    // ��ȡ�������ʹ�С�����ڴ����
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    // ��ʼ��ά����Ϣ
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;
    kf->MeasurementValidNum = 0;  // ��Ч��������ʼΪ0

    /* ���䲢��ʼ��������ز����洢�ռ� ----------------------------*/
    // ����ӳ�����¼ÿ�������Ӧ��״̬�������
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);

    // ����ϵ�������ⷽ���ж�Ӧ״̬��ϵ��
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);

    // ������������R�ĶԽ���Ԫ�أ��Զ�����ʱ��̬����R����
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);

    // ״̬��������С�������ƣ���ֹЭ����������������
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);

    // ��ʱ�洢��Ч�������������ھ��������
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    /* ���䲢��ʼ���˲������ݴ洢�ռ� ------------------------------*/
    // �˲������������չ���״̬��
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);

    // ԭʼ�����������뻺�������������ص��������£�
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);

    // ���������뻺����
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    /* ��ʼ��״̬�������� ----------------------------------------*/
    // �������״̬ x(k|k)
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, xhatSize, 1, kf->xhat_data);

    // �������״̬ x(k|k-1)
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, xhatSize, 1, kf->xhatminus_data);

    // ���������󣨴��ڿ�����ʱ��ʼ����
    if (uSize != 0) {
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, uSize, 1, kf->u_data);
    }

    // �������� z
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, zSize, 1, kf->z_data);

    /* ��ʼ��Э������� ------------------------------------------*/
    // �������Э���� P(k|k)
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, xhatSize, xhatSize, kf->P_data);

    // �������Э���� P(k|k-1)
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, xhatSize, xhatSize, kf->Pminus_data);

    /* ��ʼ��ϵͳģ�;��� ----------------------------------------*/
    // ״̬ת�ƾ��� F ����ת�� FT
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, xhatSize, xhatSize, kf->F_data);
    Matrix_Init(&kf->FT, xhatSize, xhatSize, kf->FT_data);

    // ���ƾ��� B�����ڿ�����ʱ��ʼ����
    if (uSize != 0) {
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, xhatSize, uSize, kf->B_data);
    }

    // ������� H ����ת�� HT
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, zSize, xhatSize, kf->H_data);
    Matrix_Init(&kf->HT, xhatSize, zSize, kf->HT_data);

    // ��������Э������� Q
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, xhatSize, xhatSize, kf->Q_data);

    // ��������Э������� R
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, zSize, zSize, kf->R_data);

    // ������������� K
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, xhatSize, zSize, kf->K_data);

    /* ��ʼ����ʱ������� ----------------------------------------*/
    // �����м�������ʱ���������
    kf->S_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * xhatSize);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * xhatSize);
    Matrix_Init(&kf->S, xhatSize, xhatSize, kf->S_data);
    Matrix_Init(&kf->temp_matrix, xhatSize, xhatSize, kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, xhatSize, xhatSize, kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, xhatSize, 1, kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, xhatSize, 1, kf->temp_vector_data1);

    // ��ʼ������������־��Ĭ�ϲ������κμ��㲽�裩
    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

// ��������������ע���Ѱ�����ԭ����ӣ�����ƪ���˴�ʡ��...
void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // ����H K R������������Զ�����
    // matrix H K R auto adjustment
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H��P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H��P'(k)��HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H��P'(k)��HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)��HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H��xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)��(z(k) - H��xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)��H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)��H��P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
 * @brief ִ�п������˲��ƽ���ʽ,�ṩ���û����庯��,�����������е�����һ������,����������չΪEKF/UKF/ESKF/AUKF��
 *
 * @param kf kf���Ͷ���
 * @return float* �����˲�ֵ
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. ��ȡ������Ϣ
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // �������
    // 1. xhat'(k)= A��xhat(k-1) + B��u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // Ԥ�����
    // 2. P'(k) = A��P(k-1)��AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // �������
        // 3. K(k) = P'(k)��HT / (H��P'(k)��HT + R)
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // �ں�
        // 4. xhat(k) = xhat'(k) + K(k)��(z(k) - H��xhat'(k))
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // ��������
        // 5. P(k) = (1-K(k)��H)��P'(k) ==> P(k) = P'(k)-K(k)��H��P'(k)
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // ����Ч����,��Ԥ��
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // �Զ��庯��,�����ṩ�����
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // �����˲�����������
    // suppress filter excessive convergence
    for (uint8_t i = 0; i < kf->xhatSize; ++i)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // ʶ������������Ч�Բ���������H R K
    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; ++i)
    {
        if (kf->z_data[i] != 0)
        {
            // �ع�����z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // �ع�����H
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; ++i)
    {
        // �ع�����R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // ��������ά��
    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
