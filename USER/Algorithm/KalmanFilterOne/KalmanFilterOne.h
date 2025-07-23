// Created by 刘嘉俊 on 25-4-11.
//

#ifndef CTRBOARD_H7_ALL_KALMANFILTERONE_H
#define CTRBOARD_H7_ALL_KALMANFILTERONE_H

#include <stdint.h>

/* 配置区 -------------------------------------------------*/
#define NUM_JOINTS 6  // 新增通道数定义,表示6个浮点数

// 取消注释以下行使用双精度
// #define USE_DOUBLE_PRECISION

#ifdef USE_DOUBLE_PRECISION
typedef double mat_type_t;
#else
typedef float mat_type_t;
#endif
/* -------------------------------------------------------*/
#define KALMAN_F  1.0f     // 状态转移矩阵（一维时为标量）
#define KALMAN_H  1.0f     // 观测矩阵（一维时为标量）
#define KALMAN_Q  0.08f
#define KALMAN_R  0.3f

//Q（过程噪声）：反映预测模型的不确定性。
//Q调大：系统认为预测模型不可靠，更依赖测量值（响应快，但噪声敏感）。
//Q调小：信任预测模型，滤波结果更平滑（响应慢，抗噪性好）。
//R（观测噪声）：反映传感器的测量噪声。
//R调大：认为传感器噪声大，更依赖预测值（结果平滑，可能滞后）。
//R调小：信任传感器，滤波结果紧跟测量值（响应快，但波动大）。
//Q/R比值：增大 Q/R 使响应更快，减小 Q/R 使结果更平滑。

// 定义宏来表示串口第一次接收的数据
// 避免x0初始状态每次都从0.0f开始，直接从自定义控制器第一次传回来的数据开始收敛，加快收敛速度，需要根据自定义控制器实时修改
#define FIRST_MEASUREMENT_1 183.51561f
#define FIRST_MEASUREMENT_2 232.055664f
#define FIRST_MEASUREMENT_3 109.68734f
#define FIRST_MEASUREMENT_4 173.320312f
#define FIRST_MEASUREMENT_5 250.751953f
#define FIRST_MEASUREMENT_6 138.779297f

typedef struct {
    // 系统模型矩阵
    mat_type_t F;  // 状态转移矩阵
    mat_type_t H;  // 观测矩阵

    // 噪声协方差矩阵
    mat_type_t Q;  // 过程噪声
    mat_type_t R;  // 观测噪声

    // 状态变量
    mat_type_t x;  // 状态估计
    mat_type_t P;  // 估计误差协方差

    // 卡尔曼增益
    mat_type_t K;
} KalmanFilter;

void Init_KalmanFiltersOne(mat_type_t F, mat_type_t H, mat_type_t Q, mat_type_t R);
void KalmanFilterOne_Init(KalmanFilter *kf,
                          mat_type_t F,
                          mat_type_t H,
                          mat_type_t Q,
                          mat_type_t R,
                          mat_type_t x0);

void KalmanFilterOne_Predict(KalmanFilter *kf);
mat_type_t KalmanFilterOne_Update(KalmanFilter *kf, mat_type_t z);
void KalmanFilterOne_Data(float *measurements, mat_type_t *filtered_data);

#endif //CTRBOARD_H7_ALL_KALMANFILTERONE_H



//
//// Created by 刘嘉俊 on 25-4-11.
////
//#include "arm_math.h"
//#include <stdint.h>
//
///* 配置区 -------------------------------------------------*/
//#define NUM_JOINTS          6       // 关节数量
//#define STATE_DIM           2       // 状态维度（位置+速度）
//#define MEASURE_DIM         1       // 观测维度
//#define PREDICT_TIME        0.3f    // 超前预测时间（秒）
//
//// 取消注释以下行使用双精度
//// #define USE_DOUBLE_PRECISION
//#ifdef USE_DOUBLE_PRECISION
//typedef double mat_type_t;
//#define ARM_MAT_INIT  arm_mat_init_f64
//#define ARM_MAT_MULT  arm_mat_mult_f64
//#define ARM_MAT_TRANS arm_mat_trans_f64
//#define ARM_MAT_ADD   arm_mat_add_f64
//#define ARM_MAT_SUB   arm_mat_sub_f64
//#define ARM_MAT_SCALE arm_mat_scale_f64
//#define ARM_MAT_INV   arm_mat_inverse_f64
//#define ARM_MATRIX_INSTANCE arm_matrix_instance_f64
//#else
//typedef float mat_type_t;
//#define ARM_MAT_INIT  arm_mat_init_f32
//#define ARM_MAT_MULT  arm_mat_mult_f32
//#define ARM_MAT_TRANS arm_mat_trans_f32
//#define ARM_MAT_ADD   arm_mat_add_f32
//#define ARM_MAT_SUB   arm_mat_sub_f32
//#define ARM_MAT_SCALE arm_mat_scale_f32
//#define ARM_MAT_INV   arm_mat_inverse_f32
//#define ARM_MATRIX_INSTANCE arm_matrix_instance_f32
//#endif

