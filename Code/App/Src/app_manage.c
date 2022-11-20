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


#include <stdio.h>
#include "app_manage.h"

#define TYPICAL_STK_SIZE    1024
#define LARGE_STK_SIZE    3072

ALIGNED8 OS_STK TaskAttitudeStk[LARGE_STK_SIZE];    //姿态解算任务
#define TASK_ATTITUDE_PRIO 10

ALIGNED8 OS_STK TaskControlStk[LARGE_STK_SIZE];     //控制任务
#define TASK_CONTROL_PRIO 8

#if TASK_DEBUG_EN > 0
ALIGNED8 OS_STK TaskDebugStk[LARGE_STK_SIZE];       //调试相关任务
#define TASK_DEBUG_PRIO 30
#endif


HAL_StatusTypeDef app_status_check(void) {
    if (driver_gy86_Get_Status() != GY86_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}


void app_init(void) {

#if DEVICE_RGB_EN > 0
    driver_rgb_init();
    driver_rgb_setColor(YELLOW, 1);    //系统初始化-黄灯1闪
#endif

    app_attitude_init();
    app_control_init();


#if TASK_DEBUG_EN > 0

    app_debug_init();

#endif

    if (app_status_check() == HAL_OK) {
        app_control_toggleFlyState(READY);
        driver_rgb_setColor(GREEN, 1);    //系统准备就绪-绿灯1闪
    } else {
        app_control_toggleFlyState(SENSOR_ERROR);
        driver_rgb_setColor(RED, 2);    //系统初始化错误-红灯双闪
    }

}

void app_create_task(void) {
    uint8_t err;

    OSTaskCreateExt(app_attitude_task,
                    (void *) 0,
                    TaskAttitudeStk + LARGE_STK_SIZE - 1,
                    TASK_ATTITUDE_PRIO,
                    TASK_ATTITUDE_PRIO,
                    TaskAttitudeStk,
                    LARGE_STK_SIZE,
                    (void *) 0,
                    OS_TASK_OPT_SAVE_FP);
    OSTaskNameSet(TASK_ATTITUDE_PRIO, (INT8U *) "TaskAttitude", &err);


    OSTaskCreateExt(app_control_task,
                    (void *) 0,
                    TaskControlStk + TYPICAL_STK_SIZE - 1,
                    TASK_CONTROL_PRIO,
                    TASK_CONTROL_PRIO,
                    TaskControlStk,
                    TYPICAL_STK_SIZE,
                    (void *) 0,
                    OS_TASK_OPT_SAVE_FP);
    OSTaskNameSet(TASK_CONTROL_PRIO, (INT8U *) "TaskControl", &err);


#if TASK_DEBUG_EN > 0
    OSTaskCreateExt(app_debug_task,
                    (void *) 0,
                    TaskDebugStk + LARGE_STK_SIZE - 1,
                    TASK_DEBUG_PRIO,
                    TASK_DEBUG_PRIO,
                    TaskDebugStk,
                    LARGE_STK_SIZE,
                    (void *) 0,
                    0);
    OSTaskNameSet(TASK_DEBUG_PRIO, (INT8U *) "TaskDebug", &err);
#endif
}
