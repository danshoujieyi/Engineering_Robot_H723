//
// Created by 刘嘉俊 on 25-4-29.
//

#ifndef F407VGT6_CARARM_MG310_H
#define F407VGT6_CARARM_MG310_H

/* 霍尔传感器参数 */
#define HALL_SENSOR_PPR          13      // 霍尔传感器每转脉冲数 (13线)
#define HALL_QUADRATURE_FACTOR   4       // 4倍频解码
#define HALL_EFFECTIVE_PPR       (HALL_SENSOR_PPR * HALL_QUADRATURE_FACTOR) // 实际每转脉冲数 = 52

/* 减速箱参数 */
#define GEAR_RATIO_NUMERATOR     20409   // 减速比分子 (1:20.409 = 1:20409/1000)
#define GEAR_RATIO_DENOMINATOR   1000    // 减速比分母
#define GEAR_RATIO               ((float)GEAR_RATIO_NUMERATOR / GEAR_RATIO_DENOMINATOR) // 20.409

/* 电机性能参数 */
#define NO_LOAD_RPM              500     // 空载转速 (转/分钟)
#define RATED_RPM                400     // 额定转速 (转/分钟)
#define OUTPUT_SHAFT_PPR         (HALL_EFFECTIVE_PPR * GEAR_RATIO) // 输出轴每转脉冲数 ≈ 1061.268

/* 转换常数 */
#define RPM_TO_HZ(rpm)           ((rpm) * OUTPUT_SHAFT_PPR / 60.0f) // 转速→脉冲频率转换
#define PULSE_TO_ANGLE(pulse)    ((pulse) * 360.0f / OUTPUT_SHAFT_PPR) // 脉冲数→角度转换

#endif //F407VGT6_CARARM_MG310_H
