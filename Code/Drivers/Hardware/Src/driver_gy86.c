/**
 *  @brief      GY86驱动
 *  @details    提供GY86初始化、数据接收、数据解析操作
 *  @author     Harry-Qu
 *  @date       2021.10
 *  @version    1.1
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
				1.0		|		实现初始化、数据接收、数据解析等函数
				1.1     |       修改部分IMU数据结构体
				                修改磁力计处理数据的相关bug
				                新增四元数结构体
				                新增均值法计算IMU零偏误差功能
				                新增高斯牛顿迭代法计算零偏误差与刻度误差功能（没啥用）
				                新增8字校准法测量磁力计误差功能
				                新增计算加速度计校准后数据功能
				                新增向匿名上位机发送数据功能
				                新增梯度下降法计算姿态（四元数）功能
				                新增madgwick计算姿态（四元数）功能
                1.2     |       降低与sdk_i2c, sdk_ano的耦合，注释该头文件后仍可正常运行此代码，但无法提供DMA，数据发送功能

*/


#include <memory.h>
#include "driver_gy86.h"
#include "sdk_math.h"
#include "sdk_i2c.h"
#include "sdk_ano.h"
#include "sdk_time.h"
#include "filter.h"
#include "AHRS.h"


imu_config_factor_t imu_config_factor = {16384, 131}; //IMU量程参数数据
imu_t imu_raw_data; //原始IMU加速度与陀螺仪数据
imu_t imu_last_raw_data;
imu_t imu_calibrated_data; //校准后的IMU加速度与陀螺仪数据
imu_error_t imu_offset_error = {{0, 0, 0},
                                {0, 0, 0}}; //IMU零偏误差值
imu_error_t imu_scale_error = {{1, 1, 1},
                               {1, 1, 1}}; //IMU刻度误差值
lowPass_t imu_lowPass = {0.6f};


mag_t mag_raw_data; //磁力计原始数据
imu_t mag_last_raw_data;
mag_t mag_calibrated_data;  //磁力计校准数据
mag_t mag_ground_data; //静止时的磁力计数据
int16_t mag_gain = 1090; //磁力计增益

mag_t mag_max_data = {-1.0f, -1.0f, -1.0f}, mag_min_data = {1.0f, 1.0f, 1.0f}; //8字校准时最大与最小数据
vector3f_t mag_offset_error = {0.1073f, -0.1904f,
                               0.0000f};// = {0.2991f, -0.3601f, 0.0072f};//= {0.57298536f, -0.25526098f, 0.00723172f};
vector3f_t mag_scale_error = {1, 1, 1};// = {1.2771f, 1.0607f, 1.0000f};// = {1.346518f, 1.282580f, 1.813446f};

//offset error:0.3358,-0.4358,0.0072
//scale error:1.4659,1.3650,1.0000



apg_t apg_data; //气压计数据
apg_calibration_t apg_config_data; //气压计参数数据
enum APG_OSR_VALUE apg_osr;
uint8_t apg_delayTime;

enum GY86_STATUS_CODE gy86_status = 0;  //GY86状态

quat_t attitude;
vector3f_t attitudeAngle;
quat_t attitudeTest;
vector3f_t attitudeAngleTest;
quat_t attitudeTest2;
vector3f_t attitudeAngleTest2;


/**
 * gy86发送i2C数据
 * @param DevAddress 设备地址
 * @param pData 发送数据指针
 * @param Size 发送数据大小
 * @param Timeout 超时时间
 * @return
 */
static HAL_StatusTypeDef
driver_gy86_transmitData(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_transmit_dma(&GY86_I2C, DevAddress, pData, Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else   //SDK_I2C

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Master_Transmit(&GY86_I2C, DevAddress, pData, Size, Timeout);
    OSSchedUnlock();
#else   //OS_uCOS_II_H
    status = HAL_I2C_Master_Transmit(&GY86_I2C, DevAddress, pData, Size, Timeout);
#endif  //OS_uCOS_II_H

#endif  //SDK_I2C

    if (status != HAL_OK) {
#if OS_TRACE_EN > 0
        SEGGER_SYSVIEW_ErrorfTarget("gy86 i2cs %X", status);
#endif
    }

    return status;
}

/**
 * gy86接收数据
 * @param DevAddress i2C设备地址
 * @param pData 接收数据
 * @param Size 接收数据大小
 * @param Timeout 超时时间
 * @return
 */
static HAL_StatusTypeDef driver_gy86_receiveData(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_receive_dma(&GY86_I2C, DevAddress, pData, Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Master_Receive(&GY86_I2C, DevAddress, pData, Size, Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Master_Receive(&GY86_I2C, DevAddress, pData, Size, Timeout);
#endif

#endif

    if (status != HAL_OK) {
#if OS_TRACE_EN > 0
        SEGGER_SYSVIEW_ErrorfTarget("gy86 i2cr %X", status);
#endif
    }

    return status;
}

static HAL_StatusTypeDef
driver_gy86_receiveData_memory(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size,
                               uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_memory_read_dma(&GY86_I2C, DevAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData,
                                     Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Mem_Read(&GY86_I2C, DevAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
                              Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Mem_Read(&GY86_I2C, DevAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
#endif

#endif

    if (status != HAL_OK) {
#if OS_TRACE_EN > 0
        SEGGER_SYSVIEW_ErrorfTarget("gy86 i2cmem %X", status);
#endif
    }
    return status;
}

static void driver_gy86_init_args(void) {

}

HAL_StatusTypeDef driver_gy86_Init(uint8_t init_device_id) {
    uint32_t retryTime = 0;
    HAL_StatusTypeDef status;

    driver_gy86_init_args();

    if (init_device_id & GY86_IMU) {
        do {
            status = driver_gy86_IMU_Init(8, 500);
        } while (status != HAL_OK && (retryTime++) < RETRY_TIME);

        if (status != HAL_OK) {
            gy86_status |= INIT_IMU_ERROR;
        } else {

            driver_gy86_IMU_MeasureOffsetError_Avg(200);
            printf("acc offset error:%.2f %.2f %.2f\n", imu_offset_error.acc.x, imu_offset_error.acc.y,
                   imu_offset_error.acc.z);
            printf("gyro offset error:%.2f %.2f %.2f\n", imu_offset_error.gyro.x, imu_offset_error.gyro.y,
                   imu_offset_error.gyro.z);
        }
    }

    if (init_device_id & GY86_MAG) {
        retryTime = 0;

        do {
            status = driver_gy86_MAG_Init();
        } while (status != HAL_OK && (retryTime++) < RETRY_TIME);

        if (status != HAL_OK) {
            gy86_status |= INIT_MAG_ERROR;
        } else {

        }
    }

    if (init_device_id & GY86_APG) {
        retryTime = 0;

        do {
            status = driver_gy86_APG_Init(0);
        } while (status != HAL_OK && (retryTime++) < RETRY_TIME);

        if (status != HAL_OK) {
            gy86_status |= INIT_APG_ERROR;
        } else {

        }
    }

    if (gy86_status & (INIT_IMU_ERROR | INIT_MAG_ERROR | INIT_APG_ERROR)) {
        return HAL_ERROR;
    } else {
        return HAL_OK;
    }
}

uint32_t driver_gy86_Get_Status(void) {
    return gy86_status;
}

void driver_gy86_Get_Error_Message(uint32_t status, char *errMessage) {

    if (status == 0) {
        strcpy(errMessage, "OK!");
    } else if (status & INIT_IMU_ERROR) {
        strcpy(errMessage, "INIT IMU ERROR!");
    } else if (status & INIT_MAG_ERROR) {
        strcpy(errMessage, "INIT AXIS ERROR!");
    } else if (status & INIT_APG_ERROR) {
        strcpy(errMessage, "INIT APG ERROR!");
    } else if (status & REFRESH_IMU_ERROR) {
        strcpy(errMessage, "REFRESH IMU ERROR!");
    } else if (status & REFRESH_MAG_ERROR) {
        strcpy(errMessage, "REFRESH AXIS ERROR!");
    } else if (status & REFRESH_APG_ERROR) {
        strcpy(errMessage, "REFRESH APG ERROR!");
    } else {
        strcpy(errMessage, "Unknown!");
    }
}

HAL_StatusTypeDef driver_gy86_Refresh_Data(uint8_t refresh_device_id) {
    HAL_StatusTypeDef status = HAL_OK;

    if (refresh_device_id & GY86_IMU) {
        if (driver_gy86_IMU_RefreshData() != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    if (refresh_device_id & GY86_MAG) {
        if (driver_gy86_MAG_RefreshData() != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    if (refresh_device_id & GY86_APG) {
        if (driver_gy86_APG_Refresh_Data() != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    return status;
}

HAL_StatusTypeDef driver_gy86_IMU_Init(int accelRange, int gyroRange) {
    uint8_t rawResponseData[1];
    int accelConfigData = 0x00;
    int gyroConfigData = 0x00;
    HAL_StatusTypeDef status;


    if (accelRange == 4) {
        accelConfigData = 0x08;
        imu_config_factor.accel = 8192;
    } else if (accelRange == 8) {
        accelConfigData = 0x10;
        imu_config_factor.accel = 4096;
    } else if (accelRange == 16) {
        accelConfigData = 0x18;
        imu_config_factor.accel = 2048;
    } else {
        accelConfigData = 0x00;
        imu_config_factor.accel = 16384;
    }

    if (gyroRange == 500) {
        gyroConfigData = 0x08;
        imu_config_factor.gyro = 65.5f;
    } else if (gyroRange == 1000) {
        gyroConfigData = 0x10;
        imu_config_factor.gyro = 32.8f;
    } else if (gyroRange == 2000) {
        gyroConfigData = 0x18;
        imu_config_factor.gyro = 16.4f;
    } else {
        gyroConfigData = 0x00;
        imu_config_factor.gyro = 131.0f;
    }


    uint8_t addressAndData[2] = {IMU_WHOAMI, 0x00};
    driver_gy86_transmitData(IMU_WRITE_ADDRESS, (uint8_t *) addressAndData, 1,
                             10);    /* This should return the WHOAMI register */
    status = driver_gy86_receiveData(IMU_READ_ADDRESS, (uint8_t *) rawResponseData, 1, 10);

    if (status == HAL_ERROR || (rawResponseData[0] & 0x7E) != 0x68) {
        perror("imu error.\n");
        return HAL_ERROR;
    }

    addressAndData[0] = IMU_CFG;
    addressAndData[1] = 0x00;
    status = driver_gy86_transmitData(IMU_WRITE_ADDRESS, (uint8_t *) addressAndData, 2, 10);
    if (status != HAL_OK) {
        return status;
    }

    addressAndData[0] = IMU_GYRO_CFG;
    addressAndData[1] = gyroConfigData;
    status = driver_gy86_transmitData(IMU_WRITE_ADDRESS, (uint8_t *) addressAndData, 2, 10);
    if (status != HAL_OK) {
        return status;
    }

    addressAndData[0] = IMU_ACCEL_CFG;
    addressAndData[1] = accelConfigData;
    status = driver_gy86_transmitData(IMU_WRITE_ADDRESS, (uint8_t *) addressAndData, 2, 10);
    if (status != HAL_OK) {
        return status;
    }

    driver_gy86_IMU_SetRate(1000);

    addressAndData[0] = IMU_PWR_MGMT1;
    addressAndData[1] = 0x00;
    status = driver_gy86_transmitData(IMU_WRITE_ADDRESS, (uint8_t *) addressAndData, 2, 10);
    if (status != HAL_OK) {
        return status;
    }

    addressAndData[0] = IMU_INT_PIN_CFG;
    addressAndData[1] = 0x02;
    status = driver_gy86_transmitData(IMU_WRITE_ADDRESS, (uint8_t *) addressAndData, 2, 10);
    if (status != HAL_OK) {
        return status;
    }

    addressAndData[0] = IMU_USER_CTRL;
    addressAndData[1] = 0x00;
    status = driver_gy86_transmitData(IMU_WRITE_ADDRESS, (uint8_t *) addressAndData, 2, 10);
    return status;
}

HAL_StatusTypeDef driver_gy86_IMU_SetLPF(uint16_t lpf) {
    uint8_t addressAndData[2] = {IMU_CFG, 0};
    uint8_t data;
    if (lpf >= 188)data = 1;
    else if (lpf >= 98)data = 2;
    else if (lpf >= 42)data = 3;
    else if (lpf >= 20)data = 4;
    else if (lpf >= 10)data = 5;
    else data = 6;
    addressAndData[1] = data;
    return driver_gy86_transmitData(IMU_WRITE_ADDRESS, addressAndData, 2, 10);    //设置采样频率分频器
}


HAL_StatusTypeDef driver_gy86_IMU_SetRate(uint32_t rate) {
    uint8_t addressAndData[2] = {IMU_SAMPLE_RATE_DIVICER, 0};
    uint8_t data;
    if (rate > 1000)rate = 1000;
    if (rate < 4)rate = 4;
    data = 1000 / rate - 1;
    addressAndData[1] = data;
    driver_gy86_transmitData(IMU_WRITE_ADDRESS, addressAndData, 2, 10);    //设置采样频率分频器
    return driver_gy86_IMU_SetLPF(rate / 2);    //自动设置LPF为采样率的一半
}

HAL_StatusTypeDef driver_gy86_IMU_GetRawData(void) {
//    uint8_t address[1] = {0x3B}; /* Start Address */
    uint8_t rawResponseData[14];
    HAL_StatusTypeDef status;

    if (gy86_status & INIT_IMU_ERROR) {
        return HAL_ERROR;
    }

//    driver_gy86_transmitData(IMU_WRITE_ADDRESS, (uint8_t *) address, 1, 10);
//    status = driver_gy86_receiveData(IMU_READ_ADDRESS, (uint8_t *) rawResponseData, 14, 10);

    status = driver_gy86_receiveData_memory(IMU_WRITE_ADDRESS, 0x3B, rawResponseData, 14, 10);

    if (status != HAL_OK) {
        perror("IMU refresh error.\n");
        return status;
    }


    imu_raw_data.acc.x = (float) ((int16_t) ((rawResponseData[0] << 8) | rawResponseData[1])) / imu_config_factor.accel;
    imu_raw_data.acc.y = (float) ((int16_t) ((rawResponseData[2] << 8) | rawResponseData[3])) / imu_config_factor.accel;
    imu_raw_data.acc.z = (float) ((int16_t) ((rawResponseData[4] << 8) | rawResponseData[5])) / imu_config_factor.accel;

    imu_raw_data.temp = ((float) ((int16_t) ((rawResponseData[6] << 8) | rawResponseData[7])) + 12420.2f) / 340.0f;

    imu_raw_data.gyro.x =
            (float) ((int16_t) ((rawResponseData[8] << 8) | rawResponseData[9])) / imu_config_factor.gyro *
            DEGREE_TO_RAD;
    imu_raw_data.gyro.y =
            (float) ((int16_t) ((rawResponseData[10] << 8) | rawResponseData[11])) / imu_config_factor.gyro *
            DEGREE_TO_RAD;
    imu_raw_data.gyro.z =
            (float) ((int16_t) ((rawResponseData[12] << 8) | rawResponseData[13])) / imu_config_factor.gyro *
            DEGREE_TO_RAD;

    return HAL_OK;
}

void driver_gy86_IMU_CalibrateData(void) {
    vector3f_t acc_data, gyro_data;

    acc_data.x = lowPassFilter(imu_lowPass, imu_last_raw_data.acc.x, imu_raw_data.acc.x);
    acc_data.y = lowPassFilter(imu_lowPass, imu_last_raw_data.acc.y, imu_raw_data.acc.y);
    acc_data.z = lowPassFilter(imu_lowPass, imu_last_raw_data.acc.z, imu_raw_data.acc.z);

    gyro_data.x = lowPassFilter(imu_lowPass, imu_last_raw_data.gyro.x, imu_raw_data.gyro.x);
    gyro_data.y = lowPassFilter(imu_lowPass, imu_last_raw_data.gyro.y, imu_raw_data.gyro.y);
    gyro_data.z = lowPassFilter(imu_lowPass, imu_last_raw_data.gyro.z, imu_raw_data.gyro.z);

    memcpy(&imu_last_raw_data, &imu_raw_data, sizeof(imu_t));

//    memcpy(&acc_data, &imu_raw_data.acc, sizeof(vector3f_t));
//    memcpy(&gyro_data, &imu_raw_data.gyro, sizeof(vector3f_t));

    imu_calibrated_data.acc.x = (acc_data.x - imu_offset_error.acc.x) * imu_scale_error.acc.x;
    imu_calibrated_data.acc.y = (acc_data.y - imu_offset_error.acc.y) * imu_scale_error.acc.y;
    imu_calibrated_data.acc.z = (acc_data.z - imu_offset_error.acc.z) * imu_scale_error.acc.z;

    imu_calibrated_data.gyro.x = gyro_data.x - imu_offset_error.gyro.x;
    imu_calibrated_data.gyro.y = gyro_data.y - imu_offset_error.gyro.y;
    imu_calibrated_data.gyro.z = gyro_data.z - imu_offset_error.gyro.z;
}

HAL_StatusTypeDef driver_gy86_IMU_RefreshData(void) {
    HAL_StatusTypeDef status;
    status = driver_gy86_IMU_GetRawData();
    if (status != HAL_OK) {
        return status;
    }
    driver_gy86_IMU_CalibrateData();
    return HAL_OK;
}

void driver_gy86_IMU_MeasureOffsetError_Avg(int sampleSize) {
    vector3f_t acc_sum = {0, 0, 0}; //对多次采样加速度数值求和
    vector3f_t gyro_sum = {0, 0, 0}; //对多次采样角速度数值求和
    for (int i = 0; i < sampleSize; ++i) {
        driver_gy86_IMU_GetRawData();

        acc_sum.x += imu_raw_data.acc.x;
        acc_sum.y += imu_raw_data.acc.y;
        acc_sum.z += imu_raw_data.acc.z;

        gyro_sum.x += imu_raw_data.gyro.x;
        gyro_sum.y += imu_raw_data.gyro.y;
        gyro_sum.z += imu_raw_data.gyro.z;
        HAL_Delay(2);
    }

    //默认校准时状态为静止状态，z轴加速度大小为g。
    imu_offset_error.acc.x = acc_sum.x / sampleSize;
    imu_offset_error.acc.y = acc_sum.y / sampleSize;
    imu_offset_error.acc.z = acc_sum.z / sampleSize - 1;

    imu_offset_error.gyro.x = gyro_sum.x / sampleSize;
    imu_offset_error.gyro.y = gyro_sum.y / sampleSize;
    imu_offset_error.gyro.z = gyro_sum.z / sampleSize;

}

void driver_gy86_IMU_TransmitRawData(void) {
#ifdef SDK_ANO

    struct {
        int16_t ACC_X;
        int16_t ACC_Y;
        int16_t ACC_Z;
        int16_t GYR_X;
        int16_t GYR_Y;
        int16_t GYR_Z;
        uint8_t SHOCK_STA;
    } send_raw_data;

    send_raw_data.ACC_X = (int16_t) (imu_raw_data.acc.x * 1000);
    send_raw_data.ACC_Y = (int16_t) (imu_raw_data.acc.y * 1000);
    send_raw_data.ACC_Z = (int16_t) (imu_raw_data.acc.z * 1000);

    send_raw_data.GYR_X = (int16_t) (imu_raw_data.gyro.x * 1000);
    send_raw_data.GYR_Y = (int16_t) (imu_raw_data.gyro.y * 1000);
    send_raw_data.GYR_Z = (int16_t) (imu_raw_data.gyro.z * 1000);

    send_raw_data.SHOCK_STA = 0;

    sdk_ano_transmit_inertial_sensor_data(&send_raw_data);
#endif
}

void driver_gy86_IMU_TransmitCalibratedData(void) {
#ifdef SDK_ANO
    struct {
        int16_t ACC_X;
        int16_t ACC_Y;
        int16_t ACC_Z;
        int16_t GYR_X;
        int16_t GYR_Y;
        int16_t GYR_Z;
        uint8_t SHOCK_STA;
    } send_calibrated_data;

    send_calibrated_data.ACC_X = (int16_t) (imu_calibrated_data.acc.x * 1000);
    send_calibrated_data.ACC_Y = (int16_t) (imu_calibrated_data.acc.y * 1000);
    send_calibrated_data.ACC_Z = (int16_t) (imu_calibrated_data.acc.z * 1000);

    send_calibrated_data.GYR_X = (int16_t) (imu_calibrated_data.gyro.x * 1000);
    send_calibrated_data.GYR_Y = (int16_t) (imu_calibrated_data.gyro.y * 1000);
    send_calibrated_data.GYR_Z = (int16_t) (imu_calibrated_data.gyro.z * 1000);

    send_calibrated_data.SHOCK_STA = 0;

    sdk_ano_transmit_inertial_sensor_data(&send_calibrated_data);
#endif

//    printf("%.2f %.2f %.2f\n", imu_calibrated_data.acc.x, imu_calibrated_data.acc.y, imu_calibrated_data.acc.z);
}

void driver_gy86_IMU_ACC_TransmitRawAndCalibratedData_Custom(void) {
#ifdef SDK_ANO
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } send_raw_data, send_calibrated_data;

    send_raw_data.x = (int16_t) (imu_raw_data.acc.x * 1000);
    send_raw_data.y = (int16_t) (imu_raw_data.acc.y * 1000);
    send_raw_data.z = (int16_t) (imu_raw_data.acc.z * 1000);

    send_calibrated_data.x = (int16_t) (imu_calibrated_data.acc.x * 1000);
    send_calibrated_data.y = (int16_t) (imu_calibrated_data.acc.y * 1000);
    send_calibrated_data.z = (int16_t) (imu_calibrated_data.acc.z * 1000);

    sdk_ano_transmit_custom_data(0xF1, 6, (uint8_t *) &send_raw_data);
    sdk_ano_transmit_custom_data(0xF2, 6, (uint8_t *) &send_raw_data);
#endif
}

void driver_gy86_IMU_TransmitAttitude(void) {
#ifdef SDK_ANO
    struct {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        uint8_t fusion_sta;
    } data;
    float roll = 0, pitch = 0, yaw = 0;

    roll = atanf(imu_calibrated_data.acc.y /
                 sqrtf(SQR(imu_calibrated_data.acc.x) + SQR(imu_calibrated_data.acc.z)));
    pitch = atanf(imu_calibrated_data.acc.x /
                  sqrtf(SQR(imu_calibrated_data.acc.y) + SQR(imu_calibrated_data.acc.z)));


    data.roll = (int16_t) roundf(roll / PI * 180 * 100);
    data.pitch = (int16_t) roundf(pitch / PI * 180 * 100);
    data.yaw = (int16_t) roundf(yaw / PI * 180 * 100);
    data.fusion_sta = 0;


    sdk_ano_transmit_flight_control_attitude_euler_angle(&data);
#endif
}

/**
 * 将磁力计的地址指针移至输出地址(0x03)
 */
static void driver_gy86_MAG_MoveAddressPointToOutputReg(void) {
    uint8_t addressAndData[1] = {0x03};
    HAL_StatusTypeDef status;
    status = driver_gy86_transmitData(AXIS_WRITE_ADDRESS, addressAndData, 1, 10);
}

HAL_StatusTypeDef driver_gy86_MAG_Init(void) {
    uint8_t addressAndData[2];
    HAL_StatusTypeDef status;

//    status = HAL_I2C_IsDeviceReady(&GY86_I2C, AXIS_ADDRESS, 10, 100);

    addressAndData[0] = 0x00;
    addressAndData[1] = 0x58;   //75Hz 4avg
    status = driver_gy86_transmitData(AXIS_WRITE_ADDRESS, addressAndData, 2, 10);
    if (status != HAL_OK) {
        return status;
    }

//    addressAndData[0] = 0x01;
//    addressAndData[1] = 0x80;
//    status = driver_gy86_transmitData(AXIS_WRITE_ADDRESS, addressAndData, 2, 10);


//    printf("%d\n",status);

    addressAndData[0] = 0x02;
    addressAndData[1] = 0x00;
    status = driver_gy86_transmitData(AXIS_WRITE_ADDRESS, addressAndData, 2, 10);
    if (status != HAL_OK) {
        return status;
    }
//    printf("%d\n",status);

//    driver_gy86_MAG_MoveAddressPointToOutputReg();
    HAL_Delay(67);

    return status;
}

HAL_StatusTypeDef driver_gy86_MAG_GetRawData(void) {
    uint8_t rawData[6];
    HAL_StatusTypeDef status;

    if (gy86_status & INIT_MAG_ERROR) {
        return HAL_ERROR;
    }

    driver_gy86_MAG_MoveAddressPointToOutputReg();
//
    status = driver_gy86_receiveData(AXIS_READ_ADDRESS, (uint8_t *) rawData, 6, 10);
//    status = driver_gy86_receiveData_memory(AXIS_READ_ADDRESS, 0x03, rawData, 6, 10);
//    printf("%d\n",status);

    if (status != HAL_OK) {
        return status;
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
    return HAL_OK;
}

void driver_gy86_MAG_CalibrateData(void) {
//    mag_calibrated_data.x = (float) (mag_raw_data.x - mag_mid_data.x) / 1090;
//    mag_calibrated_data.y = (float) (mag_raw_data.y - mag_mid_data.y) / 1090;
//    mag_calibrated_data.z = (float) (mag_raw_data.z - mag_mid_data.z) / 1090;

    mag_calibrated_data.x = ((float) (mag_raw_data.x) - mag_offset_error.x) * mag_scale_error.x;
    mag_calibrated_data.y = ((float) (mag_raw_data.y) - mag_offset_error.y) * mag_scale_error.y;
    mag_calibrated_data.z = ((float) (mag_raw_data.z) - mag_offset_error.z) * mag_scale_error.z;

//    float invAxisRawDataLength = invSqrt((float) SQR(mag_calibrated_data.x) + SQR(mag_calibrated_data.y) + SQR(mag_calibrated_data.z));
//    mag_calibrated_data.x = (float) mag_calibrated_data.x * invAxisRawDataLength;
//    mag_calibrated_data.y = (float) mag_calibrated_data.y * invAxisRawDataLength;
//    mag_calibrated_data.z = (float) mag_calibrated_data.z * invAxisRawDataLength;
}

HAL_StatusTypeDef driver_gy86_MAG_RefreshData(void) {
    HAL_StatusTypeDef status;
    status = driver_gy86_MAG_GetRawData();
    if (status != HAL_OK) {
        return status;
    }
    driver_gy86_MAG_CalibrateData();
    return HAL_OK;
}

void driver_gy86_MAG_TransmitRawData_Custom(void) {
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

void driver_gy86_MAG_TransmitCalibratedData_Custom(void) {
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

void driver_gy86_MAG_TransmitRawAndCalibratedData_Custom(void) {
    driver_gy86_MAG_TransmitRawData_Custom();
    driver_gy86_MAG_TransmitCalibratedData_Custom();
}

void driver_gy86_MAG_RecordGroundData(void) {
    driver_gy86_MAG_RefreshData();
    mag_ground_data.x = sqrtf(SQR(mag_calibrated_data.x) + SQR(mag_calibrated_data.y));
//    mag_ground_data.x = mag_calibrated_data.x;
//    mag_ground_data.y = mag_calibrated_data.y;
    mag_ground_data.z = mag_calibrated_data.z;

}

void driver_gy86_MAG_MeasureError_8shape(void) {
    uint32_t lastTick = 0;

    float detaTime = 0.05f;
    float minRadian = 0, maxRadian = 0, radian = 0;   //当前转动的最小角度与最大角度，用于计算是否转满一圈
    float ax = 0, ay = 0, az = 0, recipNorm;
    uint8_t finishAxisNum = 0;    // 已校准的轴的数量
    uint8_t finishAxis = 0; // 已校准的轴,按位表示,bit0=x,bit1=y,bit2=z
    uint8_t nowAxis = 0;    //正在校准的轴,按位表示

    while (finishAxisNum < 1) {
        driver_gy86_IMU_RefreshData();
        driver_gy86_MAG_GetRawData();

        if (lastTick) {
            detaTime = (HAL_GetTick() - lastTick) / 1000.0f;
        }
        lastTick = HAL_GetTick();

//        recipNorm = invSqrt(
//                SQR(imu_calibrated_data.acc.x) + SQR(imu_calibrated_data.acc.y) + SQR(imu_calibrated_data.acc.z));
        ax = imu_calibrated_data.acc.x;
        ay = imu_calibrated_data.acc.y;
        az = imu_calibrated_data.acc.z;

//        printf("%.2f %.2f %.2f\n", imu_calibrated_data.gyro.x,  imu_calibrated_data.gyro.y,
//               imu_calibrated_data.gyro.z);

        if (fabsf(az) > 0.95f && ((finishAxis & 4) == 0)) {
            if (nowAxis != 4) {
                nowAxis = 4;
                radian = 0;
                minRadian = 0;
                maxRadian = 0;
            }
            RECORD_EXTREME_VALUE(mag_raw_data.x, mag_min_data.x, mag_max_data.x);
            RECORD_EXTREME_VALUE(mag_raw_data.y, mag_min_data.y, mag_max_data.y);
            radian += imu_calibrated_data.gyro.z * detaTime;
            RECORD_EXTREME_VALUE(radian, minRadian, maxRadian);
//            printf("z %.2f %.2f %.2f %d\n", radian, minRadian, maxRadian, finishAxis);
        }

        if (maxRadian - minRadian > 1.3 * PI) {
            finishAxis |= nowAxis;
            finishAxisNum++;
//            printf("完成一个轴.\n");
        }

        HAL_Delay(15);
    }

    float xLength = mag_max_data.x - mag_min_data.x,
            yLength = mag_max_data.y - mag_min_data.y,
            zLength = mag_max_data.z - mag_min_data.z;

    printf("%f %f %f\n", xLength, yLength, zLength);

    if ((finishAxis & 1) == 0 && mag_max_data.x < 1 && mag_min_data.x > -1) {
        printf("x:%.2f %.2f\n", mag_max_data.x, mag_min_data.x);
        mag_offset_error.x = (mag_max_data.x + mag_min_data.x) / 2;
        mag_scale_error.x = STANDAR_VERTICAL_GAUSS / xLength * 2;
    } else {
        mag_scale_error.x = 1;
    }

    if ((finishAxis & 1) == 0 && mag_max_data.y < 1 && mag_min_data.y > -1) {
        printf("y:%.2f %.2f\n", mag_max_data.y, mag_min_data.y);
        mag_offset_error.y = (mag_max_data.y + mag_min_data.y) / 2;
        mag_scale_error.y = STANDAR_VERTICAL_GAUSS / yLength * 2;
    } else {
        mag_scale_error.y = 1;
    }

    if ((finishAxis & 6) == 0 && mag_max_data.z < 1 && mag_min_data.z > -1) {
        printf("z:%.2f %.2f\n", mag_max_data.z, mag_min_data.z);
        mag_offset_error.z = (mag_max_data.z + mag_min_data.z) / 2;
        mag_scale_error.z = STANDAR_HORIZONTAL_GAUSS / zLength * 2;
    } else {
        mag_scale_error.z = 1;
    }

    printf("offset error:%.4ff,%.4ff,%.4ff\n", mag_offset_error.x, mag_offset_error.y, mag_offset_error.z);
    printf("scale error:%.4ff,%.4ff,%.4ff\n", mag_scale_error.x, mag_scale_error.y, mag_scale_error.z);
}

void driver_gy86_MAG_CalculateError_8shape(void) {
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

void driver_gy86_Attitude_InitQuat_MAG(void) {
    driver_gy86_MAG_RefreshData();

    float roll = 0, pitch = 0, yaw = 0;

    yaw = atan2f(mag_calibrated_data.y, mag_calibrated_data.x);

    yaw = -yaw;

    attitude.a = cosf(yaw / 2);
    attitude.b = 0;
    attitude.c = 0;
    attitude.d = sinf(yaw / 2);

    attitudeTest = attitude;
}

void driver_gy86_Attitude_InitQuat_IMUAndAXIS(void) {

    float cosRoll, cosPitch, cosYaw;
    float sinRoll, sinPitch, sinYaw;

    float roll = 0, pitch = 0, yaw = 0;

    driver_gy86_IMU_RefreshData();
    driver_gy86_MAG_RefreshData();

    roll = atanf(imu_calibrated_data.acc.y /
                 sqrtf(SQR(imu_calibrated_data.acc.x) + SQR(imu_calibrated_data.acc.z)));
    pitch = atanf(imu_calibrated_data.acc.x /
                  sqrtf(SQR(imu_calibrated_data.acc.y) + SQR(imu_calibrated_data.acc.z)));
    yaw = -atan2f(mag_calibrated_data.y, mag_calibrated_data.x);

    cosRoll = cosf(roll / 2);
    cosPitch = cosf(pitch / 2);
    cosYaw = cosf(yaw / 2);
    sinRoll = sinf(roll / 2);
    sinPitch = sinf(pitch / 2);
    sinYaw = sinf(yaw / 2);


    attitude.a = cosRoll * cosPitch * cosYaw - sinRoll * sinPitch * sinYaw;
    attitude.b = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    attitude.c = cosRoll * sinPitch * cosYaw - sinRoll * cosPitch * sinYaw;
    attitude.d = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}

void driver_gy86_Attitude_Update_Madgwick(void) {
    AHRS_MadgWick(&attitude, imu_calibrated_data.acc, imu_calibrated_data.gyro, mag_calibrated_data);
}

void driver_gy86_Attitude_Update_Mahony(void) {
    AHRS_Mahony(&attitude, imu_calibrated_data.acc, imu_calibrated_data.gyro, mag_calibrated_data);
}

void driver_gy86_Attitude_Update_Grad(void) {
    const float step = 0.02f;
    const float esp = 0.1f;
    const uint32_t maxCnt = 20;
    float JtFq[6] = {0}, Fq[6] = {0};
    float JtFqLength;
    float a = attitude.a, b = attitude.b, c = attitude.c, d = attitude.d;
    float ax = imu_calibrated_data.acc.x, ay = imu_calibrated_data.acc.y, az = imu_calibrated_data.acc.z;
    float mx = mag_calibrated_data.x, my = mag_calibrated_data.y, mz = mag_calibrated_data.z;

    uint32_t cnt = 0;
    float bx = mag_ground_data.x, by = mag_ground_data.y, bz = mag_ground_data.z;


    //对数据归一化
    float invMLength = invSqrt((float) SQR(mx) + SQR(my) + SQR(mz));
    mx *= invMLength;
    my *= invMLength;
    mz *= invMLength;

    float invALength = invSqrt((float) SQR(ax) + SQR(ay) + SQR(az));
    ax *= invALength;
    ay *= invALength;
    az *= invALength;


    while ((cnt++) < maxCnt) {
        float aa = a * a, ab = a * b, ac = a * c, ad = a * d;
        float bb = b * b, bc = b * c, bd = b * d;
        float cc = c * c, cd = c * d;
        float dd = d * d;


        Fq[0] = 2 * bd - 2 * ac - ax;
        Fq[1] = 2 * ab + 2 * cd - ay;
        Fq[2] = (1 - 2 * bb - 2 * cc) - az;

        Fq[3] = bx * (1 - 2 * cc - 2 * dd) + 2 * bz * (bd - ac) - mx;
        Fq[4] = 2 * bx * (bc - ad) + 2 * bz * (ab + bd) - my;
        Fq[5] = 2 * bx * (ac + bd) + bz * (1 - 2 * bb - 2 * cc) - mz;

//        Fq[3] = bx * (1 - 2 * cc - 2 * dd) + 2 * by * (bc + ad) + 2 * bz * (bd - ac) - mag_calibrated_data.x;
//        Fq[4] = 2 * bx * (bc - ad) + by * (1 - 2 * cc - 2 * dd) + 2 * bz * (ab + bd) - mag_calibrated_data.y;
//        Fq[5] = 2 * bx * (ac + bd) + 2 * by * (-ab + cd) + bz * (1 - 2 * bb - 2 * cc) - mag_calibrated_data.z;

        //以下数据约掉了*2
        JtFq[0] =
                -c * Fq[0] + b * Fq[1]
                - bz * c * Fq[3] +
                (bz * b - bx * d) * Fq[4] +
                (bx * c) * Fq[5];
        JtFq[1] = d * Fq[0] + a * Fq[1] - 2 * b * Fq[2]
                  + bz * d * Fq[3] +
                  (bx * c + bz * a) * Fq[4] +
                  (bx * d - 2 * bz * b) * Fq[5];
        JtFq[2] = -a * Fq[0] + d * Fq[1] - 2 * c * Fq[2]
                  + (-2 * bx * c - bz * a) * Fq[3] +
                  (bx * b + bz * d) * Fq[4] +
                  (bx * a - 2 * bz * c) * Fq[5];
        JtFq[3] =
                b * Fq[0] + c * Fq[1] +
                (bz * b - 2 * bx * d) * Fq[3] +
                (-bx * a + bz * c) * Fq[4] +
                (bx * b) * Fq[5];

//        JtFq[0] = -c * Fq[0] + b * Fq[1] +
//                  (by * d - bz * c) * Fq[3] +
//                  (bz * b - bx * d) * Fq[4] +
//                  (-by * b + bx * c) * Fq[5];
//        JtFq[1] = d * Fq[0] + a * Fq[1] - 2 * b * Fq[2]
//                  + (by * c + bz * d) * Fq[3] +
//                  (bx * c + bz * a) * Fq[4] +
//                  (bx * d - by * a - 2 * bz * b) * Fq[5];
//        JtFq[2] = -a * Fq[0] + d * Fq[1] - 2 * c * Fq[2]
//                  + (-2 * bx * c + by * b - bz * a) * Fq[3] +
//                  (bx * b - 2 * by * c + bz * d) * Fq[4] +
//                  (bx * a + by * d - 2 * bz * c) * Fq[5];
//        JtFq[3] = b * Fq[0] + c * Fq[1] +
//                  (bz * b + by * a - 2 * bx * d) * Fq[3] +
//                  (-bx * a - 2 * by * d + bz * c) * Fq[4] +
//                  (bx * b + by * c) * Fq[5];

        JtFqLength = sqrtf(SQR(JtFq[0]) + SQR(JtFq[1]) + SQR(JtFq[2]) + SQR(JtFq[3]));

        if (JtFqLength < esp) {
//            driver_gy86_TransmitAttitude_Quat(&attitude);
            break;
        }

        a -= step * JtFq[0] / JtFqLength;
        b -= step * JtFq[1] / JtFqLength;
        c -= step * JtFq[2] / JtFqLength;
        d -= step * JtFq[3] / JtFqLength;
    }


    attitude.a = a;
    attitude.b = b;
    attitude.c = c;
    attitude.d = d;


}

void driver_gy86_TransmitAttitude_Quat(quat_t *q) {
#ifdef SDK_ANO
    struct {
        int16_t a;
        int16_t b;
        int16_t c;
        int16_t d;
        uint8_t fusion_sta;
    } sendData;

    sendData.a = (int16_t) (q->a * 10000);
    sendData.b = (int16_t) (q->b * 10000);
    sendData.c = (int16_t) (q->c * 10000);
    sendData.d = (int16_t) (q->d * 10000);
    sendData.fusion_sta = 0;
    sdk_ano_transmit_flight_control_attitude_quaternion(&sendData);
#endif
}

void driver_gy86_TransmitAttitude_Angle(vector3f_t *angle) {
#ifdef SDK_ANO
    struct {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        uint8_t fusion_sta;
    } sendData;

    sendData.roll = (int16_t) (angle->x * 100);
    sendData.pitch = (int16_t) (angle->y * 100);
    sendData.yaw = (int16_t) (angle->z * 100);
    sendData.fusion_sta = 0;
    sdk_ano_transmit_flight_control_attitude_euler_angle(&sendData);
#endif
}

HAL_StatusTypeDef driver_gy86_APG_Init(uint32_t osr) {

    int i = 0;
    HAL_StatusTypeDef status;
    uint8_t rawData[14];
    uint8_t sendData = 0xA0;

    sendData = 0x1E;    //reset
    status = driver_gy86_transmitData(APG_WRITE_ADDRESS, (uint8_t *) &sendData, 1, 10);

    HAL_Delay(50);

    if (status != HAL_OK) {
        return status;
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
        status = driver_gy86_receiveData_memory(APG_WRITE_ADDRESS, sendData, (uint8_t *) (rawData + 2 * i), 2, 10);
        if (status != HAL_OK) {
            return HAL_ERROR;
        }
        HAL_Delay(10);
    }

    apg_config_data.c1 = (rawData[2] << 8) | rawData[3];
    apg_config_data.c2 = (rawData[4] << 8) | rawData[5];
    apg_config_data.c3 = (rawData[6] << 8) | rawData[7];
    apg_config_data.c4 = (rawData[8] << 8) | rawData[9];
    apg_config_data.c5 = (rawData[10] << 8) | rawData[11];
    apg_config_data.c6 = (rawData[12] << 8) | rawData[13];


    return HAL_OK;
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
void driver_gy86_apg_cal(const uint8_t *preRawData, const uint8_t *tempRawData, apg_calibration_t *apgCal, uint32_t *p,
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

HAL_StatusTypeDef driver_gy86_APG_Refresh_Data() {

    uint8_t sendData = 0x00;
    HAL_StatusTypeDef status;
    uint8_t tempRawData[3] = {0x86, 0x9d, 0xe0};
    uint8_t preRawData[3];


    sendData = APG_D1 | apg_osr;    //convert D1 OSR pre
    status = driver_gy86_transmitData(APG_WRITE_ADDRESS, (uint8_t *) &sendData, 1, 10);
    GY86_Delay(apg_delayTime);

    sendData = 0x00;
    status = driver_gy86_receiveData_memory(APG_READ_ADDRESS, sendData, preRawData, 3, 10);
//    status = driver_gy86_transmitData(APG_WRITE_ADDRESS, (uint8_t *) &sendData, 1, 10);
//    status = driver_gy86_receiveData(APG_READ_ADDRESS, (uint8_t *) preRawData, 3, 10);
    if (status != HAL_OK) {
        return status;
    }


//    sendData = APG_D2 | apg_osr;    //convert D2 OSR temp
//    status = driver_gy86_transmitData(APG_WRITE_ADDRESS, (uint8_t *) &sendData, 1, 10);
//    GY86_Delay(apg_delayTime);
//
//    sendData = 0x00;
//    status = driver_gy86_receiveData_memory(APG_READ_ADDRESS, sendData, tempRawData, 3, 10);
//    status = driver_gy86_transmitData(APG_WRITE_ADDRESS, (uint8_t *) &sendData, 1, 10);
//    status = driver_gy86_receiveData(APG_READ_ADDRESS, (uint8_t *) tempRawData, 3, 10);


    driver_gy86_apg_cal(preRawData, tempRawData, &apg_config_data, &apg_data.pressure, &apg_data.temp);

    printf("%lu %lu\n", apg_data.pressure, apg_data.temp);

    return HAL_OK;
}

