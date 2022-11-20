/** 
 *  @brief	    滤波算法
 *  @details    提供了低通滤波算法
 *  @author     Harry-Qu
 *  @date       2022/11/18
 *  @version    1.0
 *  @par        日志
*/

#include "filter.h"


float lowPassFilter(lowPass_t lowPass, float lastNum, float nowNum) {
    return (nowNum * lowPass.trust) + lastNum * (1 - lowPass.trust);
}
