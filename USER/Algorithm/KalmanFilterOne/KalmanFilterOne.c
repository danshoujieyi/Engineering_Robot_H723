// Created by 刘嘉俊 on 25-4-11.
//
/* 卡尔曼滤波算法公式说明：
   预测阶段：
   1. 状态先验估计：x??? = F? * x????
   2. 协方差先验估计：P?? = F? * P??? * F?? + Q?

   更新阶段：
   3. 残差计算：y? = z? - H? * x???
   4. 残差协方差：S? = H? * P?? * H?? + R?
   5. 卡尔曼增益：K? = P?? * H?? * S???
   6. 状态后验估计：x?? = x??? + K? * y?
   7. 协方差更新：P? = (I - K? * H?) * P??
*/

#include "KalmanFilterOne.h"

// 在应用程序中声明滤波器数组
static KalmanFilter joint_filters[NUM_JOINTS];
const mat_type_t first_measurements[NUM_JOINTS] = {
        FIRST_MEASUREMENT_1,
        FIRST_MEASUREMENT_2,
        FIRST_MEASUREMENT_3,
        FIRST_MEASUREMENT_4,
        FIRST_MEASUREMENT_5,
        FIRST_MEASUREMENT_6
};

// 初始化每个通道（可配置不同参数）
void Init_KalmanFiltersOne(mat_type_t F, mat_type_t H, mat_type_t Q, mat_type_t R) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        mat_type_t x0 = first_measurements[i];  // 使用第一次测量值作为初始状态
        KalmanFilterOne_Init(&joint_filters[i], F, H, Q, R, x0);
    }
}


void KalmanFilterOne_Init(KalmanFilter *kf,
                          mat_type_t F,
                          mat_type_t H,
                          mat_type_t Q,
                          mat_type_t R,
                          mat_type_t x0) {
    // 初始化系统模型矩阵
    kf->F = F;
    kf->H = H;

    // 初始化噪声协方差矩阵
    kf->Q = Q;
    kf->R = R;

    // 初始化状态估计
    kf->x = x0;

    // 初始化误差协方差矩阵
    kf->P = 1.0f;
}

void KalmanFilterOne_Predict(KalmanFilter *kf) {
    // 预测状态：x??? = F * x????
    kf->x = kf->F * kf->x;

    // 预测协方差：P?? = F * P??? * F? + Q
    kf->P = kf->F * kf->P * kf->F + kf->Q;
}

mat_type_t KalmanFilterOne_Update(KalmanFilter *kf, mat_type_t z) {
    // 计算残差 y = z - Hx
    mat_type_t y = z - kf->H * kf->x;

    // 计算残差协方差 S = HPH? + R
    mat_type_t S = kf->H * kf->P * kf->H + kf->R;

    // 计算卡尔曼增益 K = PH?S??
    kf->K = kf->P * kf->H / S;

    // 更新状态估计 x = x + Ky
    kf->x = kf->x + kf->K * y;

    // 更新协方差矩阵 P = (I - KH)P
    kf->P = (1 - kf->K * kf->H) * kf->P;

    return kf->x;
}

// 接收数据 float data[6]
void KalmanFilterOne_Data(float *measurements, mat_type_t *filtered_data) {
    for (int ch = 0; ch < NUM_JOINTS; ch++) {
        KalmanFilterOne_Predict(&joint_filters[ch]);
        filtered_data[ch] = KalmanFilterOne_Update(&joint_filters[ch], measurements[ch]);
    }
}





