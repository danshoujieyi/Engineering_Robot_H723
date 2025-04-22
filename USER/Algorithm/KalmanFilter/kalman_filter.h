/**
 ******************************************************************************
 * @file    kalman filter.h
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   卡尔曼滤波器头文件
 ******************************************************************************
 * @attention
 * 该文件包含卡尔曼滤波器核心数据结构定义和接口函数声明
 ******************************************************************************
 */

#ifndef CTRBOARD_H7_ALL_KALMAN_FILTER_H
#define CTRBOARD_H7_ALL_KALMAN_FILTER_H

// 使用ARM Cortex-M4 DSP数学库
/*
#define __CC_ARM    // Keil
#define ARM_MATH_CM4
#define ARM_MATH_MATRIX_CHECK
#define ARM_MATH_ROUNDING
#define ARM_MATH_DSP    // 在arm_math.h中定义
*/

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "stdint.h"
#include "stdlib.h"

// 内存分配器配置（兼容CMSIS-RTOS）
#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

// 矩阵运算库配置（使用浮点运算）
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32      // 矩阵初始化
#define Matrix_Add arm_mat_add_f32        // 矩阵加法
#define Matrix_Subtract arm_mat_sub_f32   // 矩阵减法
#define Matrix_Multiply arm_mat_mult_f32  // 矩阵乘法
#define Matrix_Transpose arm_mat_trans_f32// 矩阵转置
#define Matrix_Inverse arm_mat_inverse_f32// 矩阵求逆

/**
 * @brief 卡尔曼滤波器核心数据结构
 * @note 包含滤波器所有状态、参数和中间计算结果
 */
typedef struct kf_t
{
    // 基本数据指针
    float *FilteredValue;    // 滤波后的状态估计值输出
    float *MeasuredVector;   // 量测向量输入缓冲区
    float *ControlVector;    // 控制向量输入缓冲区

    // 维度参数
    uint8_t xhatSize;        // 状态向量维度
    uint8_t uSize;           // 控制向量维度
    uint8_t zSize;           // 量测向量最大维度

    // 自动调整相关参数
    uint8_t UseAutoAdjustment;   // 自动调整功能开关
    uint8_t MeasurementValidNum; // 当前有效量测数量

    // 量测配置参数
    uint8_t *MeasurementMap;      // 量测-状态映射表（指定每个量测对应的状态序号）
    float *MeasurementDegree;     // 量测系数（H矩阵对应元素值）
    float *MatR_DiagonalElements; // 量测噪声方差（R矩阵对角线元素）
    float *StateMinVariance;      // 状态最小方差限制（防止协方差过度收敛）
    uint8_t *temp;                // 临时存储有效量测索引

    // 计算步骤控制标志。配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5; // 方程跳步标志

    // 矩阵定义（ARM数学库矩阵实例）
    mat xhat;      // 后验状态估计 x(k|k)
    mat xhatminus; // 先验状态估计 x(k|k-1)
    mat u;         // 控制向量
    mat z;         // 量测向量
    mat P;         // 后验协方差矩阵 P(k|k)
    mat Pminus;    // 先验协方差矩阵 P(k|k-1)
    mat F, FT;     // 状态转移矩阵及其转置
    mat B;         // 控制输入矩阵
    mat H, HT;     // 量测矩阵及其转置
    mat Q;         // 过程噪声协方差矩阵
    mat R;         // 量测噪声协方差矩阵
    mat K;         // 卡尔曼增益矩阵
    mat S;         // 中间计算结果矩阵
    mat temp_matrix, temp_matrix1; // 临时计算矩阵
    mat temp_vector, temp_vector1; // 临时计算向量

    int8_t MatStatus; // 矩阵运算状态码

    // 用户自定义函数钩子（用于扩展标准卡尔曼滤波）
    void (*User_Func0_f)(struct kf_t *kf); // 量测预处理钩子
    void (*User_Func1_f)(struct kf_t *kf); // 状态预测后处理钩子
    void (*User_Func2_f)(struct kf_t *kf); // 协方差预测后处理钩子
    void (*User_Func3_f)(struct kf_t *kf); // 卡尔曼增益计算后处理钩子
    void (*User_Func4_f)(struct kf_t *kf); // 状态更新后处理钩子
    void (*User_Func5_f)(struct kf_t *kf); // 协方差更新后处理钩子
    void (*User_Func6_f)(struct kf_t *kf); // 最终结果后处理钩子

    // 矩阵数据存储指针
    float *xhat_data, *xhatminus_data; // 状态向量存储
    float *u_data;          // 控制向量存储
    float *z_data;          // 量测向量存储
    float *P_data, *Pminus_data; // 协方差矩阵存储
    float *F_data, *FT_data;     // 状态转移矩阵存储
    float *B_data;          // 控制矩阵存储
    float *H_data, *HT_data;// 量测矩阵存储
    float *Q_data;          // 过程噪声矩阵存储
    float *R_data;          // 量测噪声矩阵存储
    float *K_data;          // 卡尔曼增益存储
    float *S_data;          // 中间计算结果存储
    float *temp_matrix_data, *temp_matrix_data1; // 临时矩阵存储
    float *temp_vector_data, *temp_vector_data1; // 临时向量存储
} KalmanFilter_t;

// 全局变量声明（用于动态内存分配）
extern uint16_t sizeof_float, sizeof_double;

/* 函数声明 ----------------------------------------------------------------*/

/**
 * @brief 卡尔曼滤波器初始化函数
 * @param kf        滤波器对象指针
 * @param xhatSize  状态向量维度
 * @param uSize     控制向量维度
 * @param zSize     量测向量最大维度
 * @note 完成内存分配、矩阵初始化、默认参数设置等准备工作
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);

/**
 * @brief 量测数据预处理函数
 * @param kf 滤波器对象指针
 * @note 执行量测有效性检查、自动调整H/R矩阵维度、数据格式转换
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf);

/**
 * @brief 先验状态估计更新（预测方程）
 * @param kf 滤波器对象指针
 * @note 计算x(k|k-1) = F·x(k-1|k-1) + B·u(k-1)
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);

/**
 * @brief 先验协方差矩阵更新（预测方程）
 * @param kf 滤波器对象指针
 * @note 计算P(k|k-1) = F·P(k-1|k-1)·F^T + Q
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);

/**
 * @brief 卡尔曼增益计算
 * @param kf 滤波器对象指针
 * @note 计算K(k) = P(k|k-1)·H^T·(H·P(k|k-1)·H^T + R)^-1
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf);

/**
 * @brief 后验状态估计更新（量测更新）
 * @param kf 滤波器对象指针
 * @note 计算x(k|k) = x(k|k-1) + K(k)·(z(k) - H·x(k|k-1))
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief 后验协方差矩阵更新（量测更新）
 * @param kf 滤波器对象指针
 * @note 计算P(k|k) = (I - K(k)·H)·P(k|k-1)
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf);

/**
 * @brief 卡尔曼滤波器主更新函数
 * @param kf 滤波器对象指针
 * @return float* 返回更新后的状态估计值指针
 * @note 按顺序执行预测和更新步骤，包含完整的卡尔曼滤波五式
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif //CTRBOARD_H7_ALL_KALMAN_FILTER_H