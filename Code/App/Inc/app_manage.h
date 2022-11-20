/**
 *  @brief      应用管理
 *  @details    负责应用初始化，创建任务等功能
 *  @author     Harry-Qu
 *  @date       2022.04
 *  @version    1.5
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
				1.0		|		实现基本功能
				1.1     |       支持dbus数据解析为信号量模式时新建任务
				1.1.1   |       完善dbus任务宏定义
				1.2     |       新增超声波任务
				1.3     |       新增LED和浮点运算测试任务，支持自定义LED测试任务数量
				1.4     |       为每个任务栈设置8字节对齐，添加多浮点数计算任务测试选项
				1.5     |       重新划分各任务
*/

#ifndef _APP_MANAGE_H
#define _APP_MANAGE_H

#include "app_cfg.h"
#include "main.h"
#include "ucos_ii.h"

#include "app_control.h"
#include "app_attitude.h"
#include "driver_rgb.h"

#if TASK_DEBUG_EN > 0

#include "app_debug.h"

#endif


#if DEVICE_BEEP_EN > 0
#include "driver_beep.h"
#endif

/**
 * @brief 应用初始化函数
 * @details 初始化所需要的全部外设
 */
void app_init(void);

/**
 * @brief 创建任务
 * @details 创建本项目所需的全部应用
 */
void app_create_task(void);

#endif //_APP_MANAGE_H
