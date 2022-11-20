/**
 *  @brief      姿态解算应用
 *  @details    负责GY86数据采集，姿态解算任务
 *  @author     Harry-Qu
 *  @date       2022.04
 *  @version    1.1
 *  @par        Copyright (c):  四轴小组
 *
*/

#ifndef _APP_GY86_H
#define _APP_GY86_H

#include "app_cfg.h"
#include "main.h"


#include "driver_gy86.h"

/**
 * @brief GY86初始化
 */
void app_attitude_init(void);

/**
 * GY86获取数据任务
 * @param args 传入参数
 * @return
 */
void app_attitude_task(void *args);

#endif

