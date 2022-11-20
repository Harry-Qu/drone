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


#ifndef _DRIVER_ULTRASOUND_H
#define _DRIVER_ULTRASOUND_H

#include "app_cfg.h"

#if DEVICE_ULTRASOUND_EN
#include "tim.h"

#define ULTRASOUND_GPIO GPIOA
#define ULTRASOUND_PIN GPIO_PIN_1

/**
 * @brief 超声波初始化
 */
void driver_ultrasound_init(void);

/**
 * @brief 获取超声波数据
 * @return 距离值（厘米）
 */
float driver_ultrasound_getValue(void);

extern float ultrasoundDistance;

#endif

#endif

