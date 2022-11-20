/**
 *  @brief      超声波驱动
 *  @author     Harry-Qu
 *  @date       2022.04
 *  @version    1.1
 *  @par        Copyright (c):  四轴小组
 *  @par 	    具体使用方法见Readme.md
 *  @par    版本变更:
				1.0		|		构建文件
				1.1     |       实现基本功能
*/


#include "driver_ultrasound.h"

#if DEVICE_ULTRASOUND_EN > 0

float ultrasoundCalDistance = 0, ultrasoundDistance=0;
OS_EVENT *sem_ultrasound_measure;

static void DelayUs(uint16_t us);

void DelayUs(uint16_t us) {
    uint32_t tickStart = SysTick->VAL;
    uint32_t tickNum = 0, curTick = 0;
    uint32_t tickMax = SysTick->LOAD;
    uint32_t delayTick = (tickMax / 1000) * us;
    while (tickNum < delayTick) {
        curTick = SysTick->VAL;
        if (curTick > tickStart) {
            tickNum = tickStart + (tickMax - curTick);
        } else {
            tickNum = tickStart - curTick;
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        uint32_t value = htim2.Instance->CCR2;
        ultrasoundDistance = (float) value /100 * 1.7;
//        OSSemPost(sem_ultrasound_measure);
        HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);
    }
}

void driver_ultrasound_init(void) {

}

float driver_ultrasound_getValue(void) {
    HAL_GPIO_WritePin(ULTRASOUND_GPIO, ULTRASOUND_PIN, GPIO_PIN_SET);
    DelayUs(10);
    HAL_GPIO_WritePin(ULTRASOUND_GPIO, ULTRASOUND_PIN, GPIO_PIN_RESET);
//    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    sem_ultrasound_measure = OSSemCreate(0);
    uint8_t err;
//    OSSemPend(sem_ultrasound_measure, 10, &err);
//    if (err != OS_ERR_NONE) {
//        return 0;
//    }
//    ultrasoundDistance=ultrasoundCalDistance;
    return ultrasoundDistance;
}

#endif
