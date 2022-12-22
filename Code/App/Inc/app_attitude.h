/**
 *  @brief      姿态解算应用
 *  @details    负责GY86数据采集，姿态解算任务
 *  @author     Harry-Qu
 *  @date       2022.04
 *  @version    1.2
 *  @par        Copyright (c):  四轴小组
 *
*/

#ifndef _APP_GY86_H
#define _APP_GY86_H

#include "app_cfg.h"
#include "main.h"
#include "driver_gy86.h"

#define MAG_CALIBRATION_ROM_PAGE 2  //存入的ROM页号(根据ROM最大支持的页数确定)
#define MAG_CALIBRATION_MAX_SIZE 6  //能存放的数据组数
#define MAG_Allowable_error 0.15f    //允许的地磁误差大小

typedef struct MAG_CALIBRATION_DATA_T mag_calibration_data_t;

struct MAG_CALIBRATION_DATA_T{
    vector3f_t scale_error; //刻度误差
    float magLength; //磁场强度
    vector3f_t offset_error; //零偏误差
    uint16_t property; //属性
    /*
     * bit0: 该数据最近是否被使用。0:未被使用,1:被使用
     */

    uint8_t sumCheck, addCheck; //校验码

};

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

/**
 * 磁力计校准
 * @details 校准方式:
 *      1.从eeprom读取校准参数
 *      2.测试
 */
void app_attitude_calibrate_mag(void);

#endif

