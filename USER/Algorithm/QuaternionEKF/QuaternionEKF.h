//
// Created by 刘嘉俊 on 25-4-11.
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
 * @brief 扩展卡尔曼滤波器（EKF）姿态解算核心结构体
 *        存储传感器数据、滤波器状态、算法参数及中间计算结果
 */
typedef struct
{
    uint8_t Initialized;                  /**< 初始化完成标志：1=已初始化，0=未初始化 */

    KalmanFilter_t IMU_QuaternionEKF;     /**< 卡尔曼滤波器实例：
                                             *   - 状态向量维度：6（前4维四元数，后2维陀螺仪零偏）
                                             *   - 量测向量维度：3（加速度计单位化后的重力向量）
                                             */

    uint8_t ConvergeFlag;                 /**< 滤波器收敛标志：1=收敛（卡方检验通过），0=未收敛 */
    uint8_t StableFlag;                   /**< 载体运动稳定性标志：1=稳定（低角速度+正常加速度），0=剧烈运动 */

    uint64_t ErrorCount;                   /**< 连续量测失效计数：用于检测滤波器是否发散（超过阈值时重置收敛状态） */
    uint64_t UpdateCount;                  /**< 姿态更新周期计数：用于初始化低通滤波器的首次输入 */

    float q[4];                           /**< 四元数姿态估计值（单位四元数），顺序：[q_w, q_x, q_y, q_z]，表示载体相对于导航坐标系的旋转 */
    float GyroBias[3];                    /**< 陀螺仪零偏估计值（单位：rad/s）：
                                             *   - GyroBias[0] = x轴零偏，GyroBias[1] = y轴零偏（z轴零偏固定为0，因yaw不可观测）
                                             */

    float Gyro[3];                        /**< 零偏补偿后的陀螺仪数据（单位：rad/s）：[gx - GyroBias[0], gy - GyroBias[1], gz] */
    float Accel[3];                       /**< 低通滤波后的加速度计数据（单位：m/s?）：用于量测更新的平滑后数据 */

    float OrientationCosine[3];           /**< 重力向量与载体坐标轴的余弦值（0~1）：
                                             *   - 用于自适应增益调整，抑制俯仰/滚转接近90°时的异常修正
                                             */

    float accLPFcoef;                     /**< 加速度计低通滤波系数：
                                             *   - 滤波公式：y = (accLPFcoef/(dt + accLPFcoef)) * y_prev + (dt/(dt + accLPFcoef)) * x
                                             *   - 0表示不滤波，直接使用原始数据
                                             */
    float gyro_norm;                      /**< 陀螺仪数据范数（单位：1/rad/s）：√(gx? + gy? + gz?)，用于判断角速度大小 */
    float accl_norm;                      /**< 加速度计数据范数（单位：m/s?）：√(ax? + ay? + az?)，用于判断是否接近重力加速度 */
    float AdaptiveGainScale;              /**< 自适应增益缩放因子：根据卡方检验结果调整卡尔曼增益，范围0~1 */

    float Roll;                           /**< 翻滚角（Roll）：绕x轴旋转角度（单位：°），由四元数反解得到 */
    float Pitch;                          /**< 俯仰角（Pitch）：绕y轴旋转角度（单位：°），由四元数反解得到 */
    float Yaw;                            /**< 偏航角（Yaw）：绕z轴旋转角度（单位：°），由四元数反解得到（-180°~180°） */

    float YawTotalAngle;                  /**< 累计偏航角（单位：°）：处理超过360°的循环问题，用于连续旋转场景（如YawTotalAngle = 360°*YawRoundCount + Yaw） */

    float Q1;                             /**< 四元数更新过程噪声参数：
                                             *   - 建模陀螺仪白噪声对姿态的影响（过程噪声矩阵Q的对角线前4维）
                                             *   - 取值越大，滤波器对陀螺仪短期噪声的容忍度越高
                                             */
    float Q2;                             /**< 陀螺仪零偏过程噪声参数：
                                             *   - 建模零偏的慢时变特性（随机游走模型，过程噪声矩阵Q的对角线后2维）
                                             *   - 取值越小，零偏估计越平滑
                                             */
    float R;                              /**< 加速度计量测噪声参数：
                                             *   - 量测噪声矩阵R的对角线元素，取值越大，滤波器对加速度计数据的信任度越低
                                             *   - 运动场景需增大以避免剧烈运动时的误更新
                                             */

    float dt;                             /**< 姿态更新周期（单位：s）：两次更新之间的时间间隔，用于离散化状态转移矩阵和噪声矩阵 */

    mat ChiSquare;                        /**< 卡方检验结果矩阵（1x1）：存储统计量χ? = (z - h(x'))^T * S?? * (z - h(x')) */
    float ChiSquare_Data[1];              /**< 卡方检验数据存储数组：用于矩阵运算接口兼容 */
    float ChiSquareTestThreshold;         /**< 卡方检验阈值：若χ?超过此值，认为量测不可信，跳过更新或启用发散保护 */
    float lambda;                         /**< 渐消因子（遗忘因子）：
                                             *   - 取值范围：0 < lambda ≤ 1，对零偏协方差矩阵进行指数衰减（P_b /= lambda）
                                             *   - 接近1时适合平稳场景，接近0时适合非平稳动态场景
                                             */

    int16_t YawRoundCount;                /**< 偏航角循环计数：记录Yaw超过±180°的次数，用于计算累计偏航角YawTotalAngle */

    float YawAngleLast;                   /**< 上一周期偏航角（单位：°）：用于检测Yaw是否跨越±180°边界，更新YawRoundCount */

} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;

extern float chiSquare;
extern float ChiSquareTestThreshold;

void IMU_QuaternionEKF_Init(
        float* init_quaternion,    // 初始四元数
        float process_noise1,      // 四元数过程噪声 Q1
        float process_noise2,      // 陀螺仪零偏过程噪声 Q2
        float measure_noise,       // 加速度计量测噪声 R
        float lambda,              // 渐消因子
        float lpf                  // 加速度计低通滤波系数
);
void IMU_QuaternionEKF_Update(
        float gx, float gy, float gz,  // 陀螺仪原始数据（rad/s）
        float ax, float ay, float az,  // 加速度计原始数据（m/s?）
        float dt                       // 更新周期（s）
);
#endif //CTRBOARD_H7_ALL_QUATERNIONEKF_H
