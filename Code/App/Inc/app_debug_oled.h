/**
 *  @brief      调试任务-OLED显示部分
 *  @details    提供OLED调试功能函数，如显示指定数据等
 *  @author     Harry-Qu
 *  @date       2021.10
 *  @version    1.3.1
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
				1.0		|		实现OLED显示gy86原始数据、遥控器数据，页面切换功能
				1.1     |       更改为定时器触发方式，增加marker跟踪功能
				1.1.1   |       修改定时器回调函数的参数类型和回调函数名
				1.2     |       新增超声波数值显示功能
				1.3     |       新增浮点数显示测试功能，修改字体大小变量名为fontSize
				1.3.1   |       修改IMU，磁力计数据的相关格式。
*/

#ifndef __APP_OLED_H
#define __APP_OLED_H

#include "app_cfg.h"

#if DEBUG_OLED_EN > 0

#include "driver_oled_4pin.h"


#define MAX_OLED_FUNCTION_NUM 3 //定义OLED显示功能
enum OLED_FUNCTION {
    GY86 = 0,
    RC,
    MOTOR
};


/**
 * @brief OLED展示数据函数
 */
void app_oled_Show_Data(void);

/**
 * @brief OLED展示数据模版的函数
 */
void app_oled_Show_Template(void);

/**
 * @brief OLED应用初始化函数
 */
void app_oled_Init(void);

/**
 * @brief OLED切换显示页面函数
 */
void app_oled_Switch_Function(void);

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif  //DEBUG_OLED_EN
#endif  //__APP_OLED_H