/** 
 *  @brief	    HMC5883驱动
 *  @details    提供HMC5883初始化，校准，数据获取等功能
 *  @author     Harry-Qu
 *  @date       2022/11/23
 *  @version    1.0.1
 *  @par        日志
 *              1.0     |       完成HMC5883驱动基本功能
 *              1.0.1   |       修复磁力计状态结构体命名错误
 *
 *  @code       示例代码
 *              {
 *                  driver_HMC5883_Init();
 *
 *                  driver_HMC5883_RefreshData();
 *              }
 *  @endcode
*/

#include "driver_hmc5883.h"
#include "sdk_math.h"
#include "sdk_ano.h"
#include "sdk_i2c.h"

enum MAG_STATUS_CODE magStatus;

mag_t mag_raw_data; //磁力计原始数据
mag_t mag_calibrated_data;  //磁力计校准数据
mag_t mag_ground_data; //静止时的磁力计数据
int16_t mag_gain = 1090; //磁力计增益

mag_t mag_max_data = {-1.0f, -1.0f, -1.0f}, mag_min_data = {1.0f, 1.0f, 1.0f}; //8字校准时最大与最小数据
vector3f_t mag_offset_error = {0.1073f, -0.1904f,
                               0.0000f};// = {0.2991f, -0.3601f, 0.0072f};//= {0.57298536f, -0.25526098f, 0.00723172f};
vector3f_t mag_scale_error = {1, 1, 1};// = {1.2771f, 1.0607f, 1.0000f};// = {1.346518f, 1.282580f, 1.813446f};

//offset error:0.3358,-0.4358,0.0072
//scale error:1.4659,1.3650,1.0000

/**
 * HMC5883发送i2C数据
 * @param DevAddress 设备地址
 * @param pData 发送数据指针
 * @param Size 发送数据大小
 * @param Timeout 超时时间
 * @return
 */
static uint8_t
driver_HMC5883_transmitData(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_transmit_dma(&HMC5883_I2C, AXIS_WRITE_ADDRESS, pData, Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else   //SDK_I2C

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Master_Transmit(&HMC5883_I2C, AXIS_WRITE_ADDRESS, pData, Size, Timeout);
    OSSchedUnlock();
#else   //OS_uCOS_II_H
    status = HAL_I2C_Master_Transmit(&HMC5883_I2C, AXIS_WRITE_ADDRESS, pData, Size, Timeout);
#endif  //OS_uCOS_II_H

#endif  //SDK_I2C

    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * HMC5883接收数据
 * @param DevAddress i2C设备地址
 * @param pData 接收数据
 * @param Size 接收数据大小
 * @param Timeout 超时时间
 * @return
 */
static uint8_t driver_HMC5883_receiveData(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_receive_dma(&HMC5883_I2C, AXIS_READ_ADDRESS, pData, Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Master_Receive(&HMC5883_I2C, AXIS_READ_ADDRESS, pData, Size, Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Master_Receive(&HMC5883_I2C, AXIS_READ_ADDRESS, pData, Size, Timeout);
#endif

#endif

    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

static uint8_t
driver_HMC5883_receiveData_memory(uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_memory_read_dma(&HMC5883_I2C, AXIS_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData,
                                     Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Mem_Read(&HMC5883_I2C, AXIS_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
                              Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Mem_Read(&HMC5883_I2C, AXIS_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
#endif

#endif

    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

static uint8_t
driver_HMC5883_transmitData_memory(uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_memory_write_dma(&HMC5883_I2C, AXIS_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData,
                                      Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Mem_Write(&HMC5883_I2C, AXIS_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
                              Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Mem_Write(&HMC5883_I2C, AXIS_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
#endif

#endif


    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t driver_HMC5883_Init(void) {
    uint8_t status;
    uint8_t data;

    data = 0x58;   //75Hz 4avg
    status = driver_HMC5883_transmitData_memory(0x00, &data, 1, 10);
    if (status != 0) {
        return 1;
    }

//    data = 0x80;
//    status = driver_HMC5883_transmitData_memory(0x01, &data, 1, 10);

    data = 0x00;
    status = driver_HMC5883_transmitData_memory(0x02, &data, 1, 10);
    if (status != 0) {
        return 1;
    }
//    printf("%d\n",status);

    HAL_Delay(67);

    return 0;
}

uint8_t driver_HMC5883_GetRawData(void) {
    uint8_t rawData[6];
    uint8_t status;

    if (magStatus == MAG_INIT_ERROR) {
        return 1;
    }
    status = driver_HMC5883_receiveData_memory(0x03, rawData, 6, 10);
//    printf("%d\n",status);

    if (status != 0) {
        return 1;
    }

    vector3int16_t mag_temp_Data;

    mag_temp_Data.x = (int16_t) ((rawData[0] << 8) | rawData[1]);
    mag_temp_Data.z = (int16_t) ((rawData[2] << 8) | rawData[3]);
    mag_temp_Data.y = (int16_t) ((rawData[4] << 8) | rawData[5]);

    if (mag_temp_Data.x > 0x07ff)
        mag_temp_Data.x -= 4096;
    if (mag_temp_Data.y > 0x07ff)
        mag_temp_Data.y -= 4096;
    if (mag_temp_Data.z > 0x07ff)
        mag_temp_Data.z -= 4096;

    mag_raw_data.x = ((float) mag_temp_Data.x) / mag_gain;
    mag_raw_data.y = ((float) mag_temp_Data.y) / mag_gain;
    mag_raw_data.z = ((float) mag_temp_Data.z) / mag_gain;
    return 0;
}

void driver_HMC5883_CalibrateData(void) {
    mag_calibrated_data.x = ((float) (mag_raw_data.x) - mag_offset_error.x) * mag_scale_error.x;
    mag_calibrated_data.y = ((float) (mag_raw_data.y) - mag_offset_error.y) * mag_scale_error.y;
    mag_calibrated_data.z = ((float) (mag_raw_data.z) - mag_offset_error.z) * mag_scale_error.z;

//    float invAxisRawDataLength = invSqrt((float) SQR(mag_calibrated_data.x) + SQR(mag_calibrated_data.y) + SQR(mag_calibrated_data.z));
//    mag_calibrated_data.x = (float) mag_calibrated_data.x * invAxisRawDataLength;
//    mag_calibrated_data.y = (float) mag_calibrated_data.y * invAxisRawDataLength;
//    mag_calibrated_data.z = (float) mag_calibrated_data.z * invAxisRawDataLength;
}

uint8_t driver_HMC5883_RefreshData(void) {
    uint8_t status;
    status = driver_HMC5883_GetRawData();
    if (status != 0) {
        return 1;
    }
    driver_HMC5883_CalibrateData();
    return 0;
}

void driver_HMC5883_TransmitRawData_Custom(void) {
#ifdef SDK_ANO
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } data;
    data.x = (int16_t) (mag_raw_data.x * 10000);
    data.y = (int16_t) (mag_raw_data.y * 10000);
    data.z = (int16_t) (mag_raw_data.z * 10000);
    sdk_ano_transmit_custom_data(0xF3, 6, (uint8_t *) &data);
#endif

//    printf("%.2f %.2f %.2f\n", mag_raw_data.x, mag_raw_data.y, mag_raw_data.z);
}

void driver_HMC5883_TransmitCalibratedData_Custom(void) {
#ifdef SDK_ANO
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } data;
    data.x = (int16_t) (mag_calibrated_data.x * 10000);
    data.y = (int16_t) (mag_calibrated_data.y * 10000);
    data.z = (int16_t) (mag_calibrated_data.z * 10000);
//    data.xy = (int16_t) (sqrtf(SQR(mag_calibrated_data.x) + SQR(mag_calibrated_data.y)) * 10000);
//    data.xyz = (int16_t) (sqrtf(SQR(mag_calibrated_data.x) + SQR(mag_calibrated_data.y)+ SQR(mag_calibrated_data.z)) * 10000);
//    printf("%d\n", data.xy);
    sdk_ano_transmit_custom_data(0xF4, 6, (uint8_t *) &data);
#endif

}

void driver_HMC5883_TransmitRawAndCalibratedData_Custom(void) {
    driver_HMC5883_TransmitRawData_Custom();
    driver_HMC5883_TransmitCalibratedData_Custom();
}

void driver_HMC5883_RecordGroundData(void) {
    driver_HMC5883_RefreshData();
    mag_ground_data.x = sqrtf(SQR(mag_calibrated_data.x) + SQR(mag_calibrated_data.y));
    mag_ground_data.z = mag_calibrated_data.z;

}

void driver_HMC5883_CalculateError_8shape(void) {
    float xLength = mag_max_data.x - mag_min_data.x,
            yLength = mag_max_data.y - mag_min_data.y,
            zLength = mag_max_data.z - mag_min_data.z;

    if (xLength > 0) {
        printf("x:%.2f %.2f\n", mag_max_data.x, mag_min_data.x);
        mag_offset_error.x = (mag_max_data.x + mag_min_data.x) / 2;
        mag_scale_error.x = STANDAR_VERTICAL_GAUSS / xLength * 2;
    } else {
        mag_scale_error.x = 1;
    }

    if (yLength > 0) {
        printf("y:%.2f %.2f\n", mag_max_data.y, mag_min_data.y);
        mag_offset_error.y = (mag_max_data.y + mag_min_data.y) / 2;
        mag_scale_error.y = STANDAR_VERTICAL_GAUSS / yLength * 2;
    } else {
        mag_scale_error.y = 1;
    }

    if (zLength > 0) {
        printf("z:%.2f %.2f\n", mag_max_data.z, mag_min_data.z);
        mag_offset_error.z = (mag_max_data.z + mag_min_data.z) / 2;
        mag_scale_error.z = STANDAR_HORIZONTAL_GAUSS / zLength * 2;
    } else {
        mag_scale_error.z = 1;
    }

    printf("offset error:%.4f,%.4f,%.4f\n", mag_offset_error.x, mag_offset_error.y, mag_offset_error.z);
    printf("scale error:%.4f,%.4f,%.4f\n", mag_scale_error.x, mag_scale_error.y, mag_scale_error.z);

}
