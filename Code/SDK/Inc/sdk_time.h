/** 
 *  @brief	    时间相关SDK
 *  @details    实现延迟毫秒，延迟微秒，获取微秒时间戳功能
 *  @author     Harry-Qu
 *  @date       2022/7/19
 *  @version    1.0
 *  @par        日志
*/

#ifndef GY86_SDK_TIME_H
#define GY86_SDK_TIME_H

#include "main.h"

/**
 * 延时指定时间（纳秒）
 * @param nanosecond 需要延时的纳秒时长
 */
void delay_us(uint32_t nanosecond);

/**
 * 延时指定时间（微秒）
 * @param microsecond 需要延时的微秒时长
 */
void delay_ms(uint32_t microsecond);

uint32_t sdk_time_GetMicroSecondsTick(void);

#endif //GY86_SDK_TIME_H
