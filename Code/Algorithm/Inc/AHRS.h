/** 
 *  @brief	    姿态计算相关算法
 *  @details    提供了MadgWick和Mahony两种姿态解算算法
 *              提供了改进版MadgWick算法
 *              提供了四元数转换角度的方法
 *  @author     Harry-Qu
 *  @date       2022/10/26
 *  @version    1.1.1
 *  @par        日志
 *              1.0     |       实现姿态解算基本功能
 *              1.1     |       新增四元数初始化功能
 *              1.1.1   |       修改四元数转欧拉角代码，角度符合右手螺旋方向
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
 * @details 按照ZYX顺序转换，绕着坐标轴逆时针旋转为角度的正方向，即右手螺旋方向为角度正方向
 * @param q 四元数
 * @param angle 角度（结果）
 */
void AHRS_ConvertQuatToDegree(quat_t *q, vector3f_t *angle);

/**
 * 初始化四元数
 * @param q 四元数
 * @param m 磁力计数据
 */
void AHRS_InitQuat_MAG(quat_t *q, vector3f_t m);

/**
 * 初始化四元数
 * @param q 四元数
 * @param a 线性加速度
 * @param m 磁力计数据
 */
void AHRS_InitQuat_IMUAndAXIS(quat_t *q, vector3f_t a, vector3f_t m);

#endif //GY86_AHRS_H
