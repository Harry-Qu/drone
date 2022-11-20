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

#include "sdk_math.h"

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float sqrtf(float x) {
    float result;
    if (x >= 0.0f) {
#if defined   (  __GNUC__  ) && defined(__ARMVFP__)
        __ASM("VSQRT.F32 %0,%1" : "=t"(result) : "t"(x));
#else
        result = sqrtf(x);
#endif

        return result;
    } else {
        return 0.0f;
    }
}


float convertString2Float(char *strs) {
    int isNegative = ((*strs) == '-');
    float p = 1;
    float value = 0;

    if (isNegative) {
        strs++;
    }
    while ((*strs) != '.' && (*strs) != '\0') {
        if ((*strs) < '0' || (*strs) > '9') {
            return NAN;
        }

        value = value * 10 + (*strs) - '0';
        strs++;
    }
    if ((*strs) == '\0') {
        if (isNegative) value = -value;
        return value;
    }
    strs++;
    while ((*strs) != '\0') {
        if ((*strs) < '0' || (*strs) > '9') {
            return NAN;
        }
        p *= 0.1f;
        value = value + ((*strs) - '0') * p;
        strs++;
    }
    if (isNegative) value = -value;
    return value;
}