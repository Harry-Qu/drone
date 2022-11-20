/**
 *  @brief      电机驱动
 *  @details    控制电机转速
 *  @author     Harry-Qu
 *  @date       2021.09
 *  @version    1.0.1
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
 *  			1.0.0	|		实现电机控制的基本功能
 *  			1.0.1   |       修改宏定义MOTOR_MAX_SPEED为MAX_MOTOR_SPEED
*/

#ifndef __DRIVER_MOTOR_H
#define __DRIVER_MOTOR_H

#include "app_cfg.h"


#include "tim.h"

#define MOTOR_TIMER htim3

#define MIN_DUTY 1000
#define MAX_DUTY 2000
#define MAX_MOTOR_SPEED 10000

/**
 * @brief 电机初始化
 */
void driver_motor_Init(void);

/**
 * @brief 设置四个电机的转速
 * @param speed 四个电机的转速数组，每个值的范围为0-MAX_MOTOR_SPEED
 */
void driver_motor_set_speed(float *speed);

#endif