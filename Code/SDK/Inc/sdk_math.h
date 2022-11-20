/** 
 *  @brief	    数学运算
 *  @details    实现更快的三角函数运算，开根号，字符串转小数功能
 *  @author     Harry-Qu
 *  @date       2022/10/20
 *  @version    1.1
 *  @par        日志
 *              1.0     |       实现三角函数，开根号，字符串转小数功能
 *              1.1     |       支持在无DSP库环境（注释arm_math.h）下使用本函数的部分功能
*/

#ifndef GY86_SDK_MATH_H
#define GY86_SDK_MATH_H

#include "main.h"
#include "arm_math.h"

#define SQR(x) ((x)*(x))
#define ABS(x) (((x)>0)?(x):(-(x)))

#define DEGREE_TO_RAD 0.017453292519943f
#define RAD_TO_DEGREE 57.295779513082321f

#ifdef _ARM_MATH_H

#define sinf arm_sin_f32
#define cosf arm_cos_f32

#endif

#define absf fabsf


#define LIMIT(x, minVal, maxVal) x=(x>(maxVal))?(maxVal):((x<(minVal))?(minVal):x)
#define RECORD_EXTREME_VALUE(x, minVal, maxVal) {\
                                                minVal = ((x)<minVal)?(x):minVal;\
                                                maxVal = ((x)>maxVal)?(x):maxVal;\
                                                }

/**
 * 求1/sqrt(x)
 * @param x
 * @return
 */
float invSqrt(float x);

float sqrtf(float x);

/**
 * 将字符串转浮点数，功能和atof一样，效率比他高
 * @param strs 字符串
 * @return 浮点数
 */
float convertString2Float(char *strs);

#endif //GY86_SDK_MATH_H
