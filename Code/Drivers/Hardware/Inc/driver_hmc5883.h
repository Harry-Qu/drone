/** 
 *  @brief	    HMC5883驱动
 *  @details    提供HMC5883初始化，校准，数据获取等功能
 *  @author     Harry-Qu
 *  @date       2022/11/23
 *  @version    1.0.1
 *  @par        日志
 *              1.0     |       完成HMC5883驱动基本功能
 *              1.0.1   |       修复磁力计状态结构体命名命名错误
 *                              修改磁场数据为广州数据，并新增查询链接注释
*/
#ifndef HMC5883_H
#define HMC5883_H

#include "main.h"
#include "dataType.h"
#include "i2c.h"

/** =========配置========= **/

#define HMC5883_I2C hi2c1


//坐标查询链接:https://lbs.amap.com/tools/picker
//当地磁场强度查询链接:https://ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm

#define STANDAR_GAUSS 0.407899f //当地标准磁场
#define STANDAR_VERTICAL_GAUSS  0.306911f   //当地标准磁场水平强度
#define STANDAR_HORIZONTAL_GAUSS  0.268678f //当地标准磁场垂直强度

/** =========配置========= **/

#define AXIS_ADDRESS (uint8_t)0x1E    //HMC5883L设备I2C地址
#define AXIS_WRITE_ADDRESS    (AXIS_ADDRESS<<1)
#define AXIS_READ_ADDRESS    ((AXIS_ADDRESS<<1) | 1)

typedef vector3f_t mag_t;   //地磁处理后数据

enum MAG_STATUS_CODE{
    MAG_OK = 0,
    MAG_INIT_ERROR,
    MAG_REFRESH_ERROR,
};

extern enum MAG_STATUS_CODE magStatus;
extern mag_t mag_raw_data;
extern mag_t mag_calibrated_data;
extern vector3f_t mag_offset_error;
extern vector3f_t mag_scale_error;

/**
 * @brief 初始化磁力计
 * @return 初始化结果
 *         0: success
 *         1: fail
 */
uint8_t driver_HMC5883_Init(void);

/**
 * 从磁力计读取原始数据
 * @details 从HMC5883L读取原始数据（数据顺序XZY），并将数据写入mag_raw_data中
 * @return 获取数据结果
 *         0: success
 *         1: fail
 */
uint8_t driver_HMC5883_GetRawData(void);

/**
 * 校准磁力计数据
 * @details 从mag_raw_data读取数据，对数据校准后保存至mag_data中
 */
void driver_HMC5883_CalibrateData(void);

/**
 * 刷新磁力计数据
 * @details 从HMC5883L读取原始数据后，对数据校准，保存至mag_data中
 * @return 刷新数据结果
 *         0: success
 *         1: fail
 */
uint8_t driver_HMC5883_RefreshData(void);

/**
 * 使用自定义帧向匿名上位机发送磁力计原始数据
 */
void driver_HMC5883_TransmitRawData_Custom(void);

/**
 * 使用自定义帧向匿名上位机发送磁力计校准后数据
 */
void driver_HMC5883_TransmitCalibratedData_Custom(void);

/**
 * 使用自定义帧向匿名上位机发送磁力计原始数据和校准后数据
 */
void driver_HMC5883_TransmitRawAndCalibratedData_Custom(void);

/**
 * 起飞前记录地面地磁数据信息
 */
void driver_HMC5883_RecordGroundData(void);

/**
 * 使用8字校准法校准磁力计
 * @details 借助IMU中加速度计，使用8字校准法校准磁力计，误差数据保存在mag_mid_data中
 */
void driver_HMC5883_MeasureError_8shape(void);

/**
 * 计算零偏误差与刻度误差
 */
void driver_HMC5883_CalculateError_8shape(void);

#endif //HMC5883_H
