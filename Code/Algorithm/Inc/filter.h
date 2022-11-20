/** 
 *  @brief	    滤波算法
 *  @details    提供了低通滤波算法
 *  @author     Harry-Qu
 *  @date       2022/11/18
 *  @version    1.0
 *  @par        日志
*/

#ifndef FILTER_H
#define FILTER_H

typedef struct {
    float trust;
}lowPass_t;

float lowPassFilter(lowPass_t lowPass, float lastNum, float nowNum);

#endif //FILTER_H
