/** 
 *  @brief	    MS5611驱动
 *  @details    提供MS5611初始化，数据获取与计算功能
 *  @author     Harry-Qu
 *  @date       2022/11/23
 *  @version    1.0
 *  @par        日志
 *              1.0     |       实现MS5611驱动基本功能
*/

#ifndef MS5611_H
#define MS5611_H

#include "main.h"
#include "dataType.h"
#include "i2c.h"

/** =========配置========= **/

#define MS5611_I2C hi2c1

/** =========配置========= **/


#ifdef OS_uCOS_II_H
#define MS5611_DELAY(x) OSTimeDly(OS_TICKS(x))
#else
#define MS5611_DELAY(x) HAL_Delay(x)
#endif

#define APG_ADDRESS (uint8_t)0x77    //MS5611设备I2C地址
#define APG_WRITE_ADDRESS (APG_ADDRESS<<1)
#define APG_READ_ADDRESS ((APG_ADDRESS<<1) | 1)

#define APG_D1 0x40
#define APG_D2 0x50

enum APG_OSR_VALUE {
    APG_OSR_256 = 0,
    APG_OSR_512 = 2,
    APG_OSR_1024 = 4,
    APG_OSR_2048 = 6,
    APG_OSR_4096 = 8
};

typedef struct {
    uint32_t pressure;    //APG测量温度，单位：0.01mbar，eg:100009=1000.09mbar
    uint32_t temp;            //APG测量大气压，单位:0.01C，eg:2007=20.07C
} apg_t;    //气压计数据

typedef struct {
    uint16_t c1, c2, c3, c4, c5, c6;
} apg_calibration_t;    //APG校准数据，参数意义见文档

extern apg_t apg_data;
extern uint32_t ApgPressure;
extern uint32_t ApgTemp;

/**
 * @brief 初始化APG气压计
 * @param osr 精确度
 *              4096        0.012mbar
 *              2048        0.018mbar
 *              1024        0.027mbar
 *              512         0.042mbar
 *              256         0.065mbar
 * @return 初始化结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MS5611_Init(uint32_t osr);

/**
 * @brief 刷新APG的数据
 * @return 刷新结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MS5611_Refresh_Data(void);

#endif //MS5611_H
