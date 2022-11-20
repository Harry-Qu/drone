/** 
 *  @brief	    姿态计算相关算法
 *  @details    提供了MadgWick和Mahony两种姿态解算算法
 *              提供了改进版MadgWick算法
 *              提供了四元数转换角度的方法
 *  @author     Harry-Qu
 *  @date       2022/10/26
 *  @version    1.0
 *  @par        日志
*/

#ifndef GY86_AHRS_H
#define GY86_AHRS_H

#include "main.h"
#include "sdk_ano.h"

#define MADGWICK_BETA 0.066f
#define AHRS_DELTA_TIME 0.03f

extern float twoKp;
extern float twoKi;
extern float stepSize;
extern float stepMax, betaMax, betaK;

/**
 * madgwick算法
 * @param q 四元数（原始数据及结果数据）
 * @param a 加速度
 * @param g 角速度
 * @param m 磁力计数据
 */
void AHRS_MadgWick(quat_t *q, vector3f_t a, vector3f_t g, vector3f_t m);

/**
 * madgwick算法测试版（公式中梯度部分根据梯度大小动态求得参数β，而不是使用定值）
 * @param q 四元数（原始数据及结果数据）
 * @param a 加速度
 * @param g 角速度
 * @param m 磁力计数据
 */
void AHRS_MadgWickTest(quat_t *q, vector3f_t a, vector3f_t g, vector3f_t m);

/**
 * Mahony算法
 * @param q 四元数（原始数据及结果数据）
 * @param a 加速度
 * @param g 角速度
 * @param m 磁力计数据
 */
void AHRS_Mahony(quat_t *q, vector3f_t a, vector3f_t g, vector3f_t m);

/**
 * 将四元数转换为欧拉角
 * @details 按照ZYX顺序转换，绕着坐标轴逆时针旋转为角度的负方向，即右手螺旋方向为角度负方向
 * @param q 四元数
 * @param angle 角度（结果）
 */
void AHRS_ConvertQuatToDegree(quat_t *q, vector3f_t *angle);

#endif //GY86_AHRS_H
