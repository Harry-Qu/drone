/** 
 *  @brief	    MS5611驱动
 *  @details    提供MS5611初始化，数据获取与计算功能
 *  @author     Harry-Qu
 *  @date       2022/11/23
 *  @version    1.0
 *  @par        日志
 *              1.0     |       实现MS5611驱动基本功能
 *
 *  @code       示例代码
 *
 *              {
 *                  driver_MS5611_Init(2048);
 *                  driver_MS5611_Refresh_Data();
 *              }
 *  @endcode
 *
*/

#include "driver_ms5611.h"
#include "sdk_ano.h"
#include "sdk_i2c.h"

apg_t apg_data; //气压计数据
apg_calibration_t apg_config_data; //气压计参数数据
enum APG_OSR_VALUE apg_osr;
uint8_t apg_delayTime;

/**
 * MS5611发送i2C数据
 * @param DevAddress 设备地址
 * @param pData 发送数据指针
 * @param Size 发送数据大小
 * @param Timeout 超时时间
 * @return
 */
static uint8_t
driver_MS5611_transmitData(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_transmit_dma(&MS5611_I2C, APG_WRITE_ADDRESS, pData, Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else   //SDK_I2C

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Master_Transmit(&MS5611_I2C, APG_WRITE_ADDRESS, pData, Size, Timeout);
    OSSchedUnlock();
#else   //OS_uCOS_II_H
    status = HAL_I2C_Master_Transmit(&MS5611_I2C, APG_WRITE_ADDRESS, pData, Size, Timeout);
#endif  //OS_uCOS_II_H

#endif  //SDK_I2C

    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * MS5611接收数据
 * @param DevAddress i2C设备地址
 * @param pData 接收数据
 * @param Size 接收数据大小
 * @param Timeout 超时时间
 * @return
 */
static uint8_t driver_MS5611_receiveData(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_receive_dma(&MS5611_I2C, APG_READ_ADDRESS, pData, Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Master_Receive(&MS5611_I2C, APG_READ_ADDRESS, pData, Size, Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Master_Receive(&MS5611_I2C, APG_READ_ADDRESS, pData, Size, Timeout);
#endif

#endif

    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

static uint8_t
driver_MS5611_receiveData_memory(uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_memory_read_dma(&MS5611_I2C, APG_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData,
                                     Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Mem_Read(&MS5611_I2C, APG_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
                              Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Mem_Read(&MS5611_I2C, APG_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
#endif

#endif

    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

static uint8_t
driver_MS5611_transmitData_memory(uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_memory_write_dma(&MS5611_I2C, APG_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData,
                                      Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

    #ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Mem_Write(&MS5611_I2C, APG_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
                              Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Mem_Write(&MS5611_I2C, APG_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
#endif

#endif


    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t driver_MS5611_Init(uint32_t osr) {

    int i = 0;
    uint8_t status;
    uint8_t rawData[14];
    uint8_t sendData = 0xA0;

    sendData = 0x1E;    //reset
    status = driver_MS5611_transmitData((uint8_t *) &sendData, 1, 10);

    HAL_Delay(50);

    if (status != 0) {
        return 1;
    }

    if (osr <= 256) {
        apg_osr = APG_OSR_256;
        apg_delayTime = 1;
    } else if (osr <= 512) {
        apg_osr = APG_OSR_512;
        apg_delayTime = 2;
    } else if (osr <= 1024) {
        apg_osr = APG_OSR_1024;
        apg_delayTime = 3;
    } else if (osr <= 2048) {
        apg_osr = APG_OSR_2048;
        apg_delayTime = 5;
    } else {
        apg_osr = APG_OSR_4096;
        apg_delayTime = 9;
    }

    //read prom
    for (i = 1; i < 8; ++i) {
        sendData = 0xA0 | (i << 1);
        status = driver_MS5611_receiveData_memory(sendData, (uint8_t *) (rawData + 2 * i), 2, 10);
        if (status != 0) {
            return 1;
        }
        HAL_Delay(10);
    }

    apg_config_data.c1 = (rawData[2] << 8) | rawData[3];
    apg_config_data.c2 = (rawData[4] << 8) | rawData[5];
    apg_config_data.c3 = (rawData[6] << 8) | rawData[7];
    apg_config_data.c4 = (rawData[8] << 8) | rawData[9];
    apg_config_data.c5 = (rawData[10] << 8) | rawData[11];
    apg_config_data.c6 = (rawData[12] << 8) | rawData[13];


    return 0;
}

/**
 * @brief 解析APG气压计数据
 * @param preRawData 大气压原始数据
 * @param tempRawData 温度原始数据
 * @param apgCal 校准值
 * @param p	大气压计算结果
 * @param temp 温度计算结果
 * @retval None
 */
void driver_MS5611_cal(const uint8_t *preRawData, const uint8_t *tempRawData, apg_calibration_t *apgCal, uint32_t *p,
                         uint32_t *temp) {
    uint32_t d1, d2;

    uint32_t dt;

    uint64_t off, sens;

    d1 = (preRawData[0] << 16) | (preRawData[1] << 8) | (preRawData[2]);
    d2 = (tempRawData[0] << 16) | (tempRawData[1] << 8) | (tempRawData[2]);

    dt = d2 - (apgCal->c5 << 8);
    *temp = 2000 + ((dt * apgCal->c6) >> 23);

    off = (apgCal->c2 << 16) + ((apgCal->c4 * dt) >> 7);
    sens = (apgCal->c1 << 15) + ((apgCal->c3 * dt) >> 8);
    *p = (((uint64_t) d1 * sens >> 21) - off) >> 15;

}

uint8_t driver_MS5611_Refresh_Data() {

    uint8_t sendData = 0x00;
    uint8_t status;
    uint8_t tempRawData[3];
    uint8_t preRawData[3];


    sendData = APG_D1 | apg_osr;    //convert D1 OSR pre
    status = driver_MS5611_transmitData((uint8_t *) &sendData, 1, 10);
    if (status != 0) {
        return 1;
    }
    MS5611_DELAY(apg_delayTime);

    sendData = 0x00;
    status = driver_MS5611_receiveData_memory(sendData, preRawData, 3, 10);
    if (status != 0) {
        return 1;
    }


    sendData = APG_D2 | apg_osr;    //convert D2 OSR temp
    status = driver_MS5611_transmitData((uint8_t *) &sendData, 1, 10);
    if (status != 0) {
        return 1;
    }
    MS5611_DELAY(apg_delayTime);

    sendData = 0x00;
    status = driver_MS5611_receiveData_memory(sendData, tempRawData, 3, 10);
    if (status != 0) {
        return 1;
    }

    driver_MS5611_cal(preRawData, tempRawData, &apg_config_data, &apg_data.pressure, &apg_data.temp);

//    printf("%lu %lu\n", apg_data.pressure, apg_data.temp);

    return status;
}
