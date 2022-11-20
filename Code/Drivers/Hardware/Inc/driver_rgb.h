/**
 *  @brief      RGB灯驱动
 *  @details    控制机头机尾的RGB指示灯状态
 *  @author     Harry-Qu
 *  @date       2021.11.25
 *  @version    1.0
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
				1.0		|		构建文件
				1.1     |       修改为仅支持单RGB灯，添加定时器功能，支持控制每秒闪烁次数
*/


#ifndef __DRIVER_RGB_H
#define __DRIVER_RGB_H

#include "app_cfg.h"

enum RGB_COLOR {
    //RGB
    WHITE = 0,  //白色
    YELLOW = 1, //黄色
    PINK = 2,   //粉色
    RED = 3,    //红色
    CYAN = 4,   //青色
    GREEN = 5,  //绿色
    BLUE = 6,   //蓝色
    OFF = 7     //关闭
};

#if DEVICE_RGB_EN > 0

#include "main.h"
#include "gpio.h"
#include "tim.h"


#define R_PIN GPIO_PIN_2
#define R_GPIO_Port GPIOC
#define G_PIN GPIO_PIN_1
#define G_GPIO_Port GPIOC
#define B_PIN GPIO_PIN_0
#define B_GPIO_Port GPIOC

//yellow=red+green
//pink=blue+red
//bluegrass=blue+green


void driver_rgb_setColor(enum RGB_COLOR color, uint8_t lightTimes);

void driver_rgb_init(void);

#else   //DEVICE_RGB_EN

#define driver_rgb_setColor
#define driver_rgb_init

#endif  //DEVICE_RGB_EN

#endif
