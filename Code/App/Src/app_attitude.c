/**
 *  @brief      姿态解算应用
 *  @details    负责GY86数据采集，姿态解算任务
 *  @author     Harry-Qu
 *  @date       2022.04
 *  @version    1.1.1
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
 *  			1.0.0   |		实现基本功能
 *  			1.1.0   |       支持OSTimeDly延时方式，修复初始化时延时的bug
 *  			1.1.1   |       将任务修改为定时器调用方式，周期控制更加精准
 *
 *  @note
 *              姿态解算任务初始化：app_attitude_init();
 *              姿态解算任务：app_attitude_task
 *
*/

#include "app_attitude.h"
#include "AHRS.h"

OS_EVENT *sem_attitude_task;

void app_attitude_init(void) {
    driver_gy86_Init(GY86_IMU | GY86_MAG);  //初始化GY86各芯片

//    driver_gy86_MAG_MeasureError_8shape();

    driver_gy86_Attitude_InitQuat_MAG();    //计算一个初始的四元数
}

void app_attitude_tmr_callback(void *ptmr, void *parg) {
    (void) ptmr;
    (void) parg;
    OSSemPost(sem_attitude_task);
}

void app_attitude_task(void *args) {
    (void) args;
    uint8_t err;
    OS_TMR *tmr_attitude_task;
    sem_attitude_task = OSSemCreate(0);
    tmr_attitude_task = OSTmrCreate(0, OS_TICKS(1), OS_TMR_OPT_PERIODIC, app_attitude_tmr_callback, (void *) 0,
                                    (INT8U *) "attitude_TASK_Tmr", &err);//75Hz
    OSTmrStart(tmr_attitude_task, &err);

    while (1) {
        OSSemPend(sem_attitude_task, 0, &err);
        OS_TRACE_MARKER_START(2);
        driver_gy86_IMU_RefreshData();
        driver_gy86_MAG_RefreshData();
//        driver_gy86_Attitude_Update_Madgwick();
        AHRS_MadgWickTest(&attitude, imu_calibrated_data.acc, imu_calibrated_data.gyro, mag_calibrated_data);
//        driver_gy86_Attitude_Update_Mahony();
        OS_TRACE_MARKER_STOP(2);
    }

}

/**
 * 按键中断回调函数，调试用于比较各姿态解算方法优劣
 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    if (GPIO_Pin == B1_Pin && !(B1_GPIO_Port->IDR & B1_Pin)) {
//        attitudeTest = attitude;
//        attitudeTest2 = attitude;
//    }
//}
