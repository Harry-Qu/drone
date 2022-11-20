/** 
 *  @brief	    时间相关SDK
 *  @details    实现延迟毫秒，延迟微秒，获取微秒时间戳功能
 *  @author     Harry-Qu
 *  @date       2022/7/19
 *  @version    1.0
 *  @par        日志
*/

#include "sdk_time.h"

void delay_us(uint32_t nanosecond) {

    uint32_t microseconds = nanosecond / 1000;
    nanosecond = nanosecond % 1000;

    uint32_t tickStart = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    uint32_t tickTimesStart = HAL_GetTick();
    uint32_t tickLoad = SysTick->LOAD & SysTick_LOAD_RELOAD_Msk;

    uint32_t tickFreq = 1000 / HAL_GetTickFreq();
    uint32_t tickDelta = (tickLoad + 1) / 10 * nanosecond / tickFreq * 10;   //1kHz

    uint32_t waitTick, waitTimes;

    if (tickDelta < tickStart) {
        waitTick = tickStart - tickDelta;
        waitTimes = tickTimesStart + microseconds;
    } else {
        waitTick = tickLoad - tickDelta + tickStart;
        waitTimes = tickTimesStart + 1 + microseconds;
    }

    while (SysTick->VAL > waitTick || HAL_GetTick() < waitTimes);
}

void delay_ms(uint32_t microsecond) {
    HAL_Delay(microsecond);
}

uint32_t sdk_time_GetMicroSecondsTick(void) {
    uint32_t secondTick = HAL_GetTick();
    uint32_t microSecond = 1000 - SysTick->VAL * 1000 / SysTick->LOAD;
    return secondTick * 1000 + microSecond;
}
