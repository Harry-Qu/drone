/**
 *  @brief      电机驱动
 *  @details    控制电机转速
 *  @author     Harry-Qu
 *  @date       2021.09
 *  @version    1.0
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
				1.0		|		实现电机控制的基本功能
*/

#include "driver_motor.h"
#include "app_cfg.h"
#include "sdk_math.h"


void driver_motor_Init(void) {
    HAL_TIM_Base_Start(&MOTOR_TIMER);
    HAL_TIM_PWM_Start(&MOTOR_TIMER, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&MOTOR_TIMER, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&MOTOR_TIMER, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&MOTOR_TIMER, TIM_CHANNEL_4);
}

void driver_motor_set_speed(float *speed) {

    //将速度限制在[0,MOTOR_MAX_SPEED]中
    LIMIT(speed[0], 0, 0.6f * MOTOR_MAX_SPEED);
    LIMIT(speed[1], 0, 0.6f * MOTOR_MAX_SPEED);
    LIMIT(speed[2], 0, 0.6f * MOTOR_MAX_SPEED);
    LIMIT(speed[3], 0, 0.6f * MOTOR_MAX_SPEED);

    //设置对应的PWM占空比
    MOTOR_TIMER.Instance->CCR1 = MIN_DUTY + (uint16_t) ((MAX_DUTY - MIN_DUTY) * (speed[0] * 1.0f / MOTOR_MAX_SPEED));
    MOTOR_TIMER.Instance->CCR2 = MIN_DUTY + (uint16_t) ((MAX_DUTY - MIN_DUTY) * (speed[1] * 1.0f / MOTOR_MAX_SPEED));
    MOTOR_TIMER.Instance->CCR3 = MIN_DUTY + (uint16_t) ((MAX_DUTY - MIN_DUTY) * (speed[2] * 1.0f / MOTOR_MAX_SPEED));
    MOTOR_TIMER.Instance->CCR4 = MIN_DUTY + (uint16_t) ((MAX_DUTY - MIN_DUTY) * (speed[3] * 1.0f / MOTOR_MAX_SPEED));
}
