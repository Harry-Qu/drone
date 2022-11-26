/** 
 *  @brief	    MPU6050驱动
 *  @details    提供MPU6050初始化，数据获取，数据校准等基本操作。
 *  @author     Harry-Qu
 *  @date       2022/11/23
 *  @version    1.0
 *  @par        日志
 *              1.0     |       完成MPU6050驱动基本功能
 *              
 *              
 *  @code       示例代码
 *              {
 *                  driver_MPU6050_Init(2, 500); //初始化MPU6050
 *                  driver_MPU6050_MeasureOffsetError_Avg(200); //计算零偏误差
 *
 *                  driver_MPU6050_RefreshData(); //刷新数据
 *              }
 *  
*/

#include "driver_mpu6050.h"
#include "sdk_math.h"
#include "sdk_i2c.h"
#include "sdk_ano.h"
#include "sdk_time.h"

#ifdef MPU6050_FILTER

#include "filter.h"

#endif

enum IMU_STATUS_CODE imuStatus = IMU_OK;

imu_config_factor_t imu_config_factor = {16384, 131}; //IMU量程参数数据
imu_t imu_raw_data; //原始IMU加速度与陀螺仪数据

imu_t imu_calibrated_data; //校准后的IMU加速度与陀螺仪数据
imu_error_t imu_offset_error = {{0, 0, 0},
                                {0, 0, 0}}; //IMU零偏误差值
imu_error_t imu_scale_error = {{1, 1, 1},
                               {1, 1, 1}}; //IMU刻度误差值

#ifdef MPU6050_FILTER
lowPass_t imu_lowPass = {0.6f};
imu_t imu_last_raw_data;
#endif

/**
 * MPU6050发送i2C数据
 * @param DevAddress 设备地址
 * @param pData 发送数据指针
 * @param Size 发送数据大小
 * @param Timeout 超时时间
 * @return
 */
static uint8_t
driver_MPU6050_transmitData(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_transmit_dma(&MPU6050_I2C, MPU6050_WRITE_ADDRESS, pData, Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else   //SDK_I2C

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050_WRITE_ADDRESS, pData, Size, Timeout);
    OSSchedUnlock();
#else   //OS_uCOS_II_H
    status = HAL_I2C_Master_Transmit(&MPU6050_I2C, MPU6050_WRITE_ADDRESS, pData, Size, Timeout);
#endif  //OS_uCOS_II_H

#endif  //SDK_I2C

    if (status != HAL_OK) {
        return 1;
    }

    return 0;
}

/**
 * MPU6050接收数据
 * @param DevAddress i2C设备地址
 * @param pData 接收数据
 * @param Size 接收数据大小
 * @param Timeout 超时时间
 * @return
 */
static uint8_t driver_MPU6050_receiveData(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_receive_dma(&MPU6050_I2C, MPU6050_READ_ADDRESS, pData, Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Master_Receive(&MPU6050_I2C, MPU6050_READ_ADDRESS, pData, Size, Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Master_Receive(&MPU6050_I2C, MPU6050_READ_ADDRESS, pData, Size, Timeout);
#endif

#endif

    if (status != HAL_OK) {
        return 1;
    }

    return 0;
}

static uint8_t
driver_MPU6050_receiveData_memory(uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_memory_read_dma(&MPU6050_I2C, MPU6050_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData,
                                     Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
                              Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_READ_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
#endif

#endif

    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

static uint8_t
driver_MPU6050_transmitData_memory(uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
#ifdef SDK_I2C
    uint8_t err;
//    OS_TRACE_MARKER_START(6);
    status = sdk_i2c_memory_write_dma(&MPU6050_I2C, MPU6050_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData,
                                      Size, Timeout, &err);
//    OS_TRACE_MARKER_STOP(6);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    status = HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
                              Timeout);
    OSSchedUnlock();
#else
    status = HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_WRITE_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
#endif

#endif


    if (status != HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t driver_MPU6050_Init(int accelRange, int gyroRange) {
    uint8_t status;

    status = driver_MPU6050_IsOnline();
    if (status != 0) {
        perror("imu error.\n");
        imuStatus = IMU_INIT_ERROR;
        return 1;
    }

    status = driver_MPU6050_SetAccelRange(accelRange);
    if (status != 0) {
        imuStatus = IMU_INIT_ERROR;
        return 1;
    }

    status = driver_MPU6050_SetGyroRange(gyroRange);
    if (status != 0) {
        imuStatus = IMU_INIT_ERROR;
        return 1;
    }

    status = driver_MPU6050_SetRate(1000);
    if (status != 0) {
        imuStatus = IMU_INIT_ERROR;
        return 1;
    }

    status = driver_MPU6050_SetSleepMode(0);
    if (status != 0) {
        imuStatus = IMU_INIT_ERROR;
        return 1;
    }

    status = driver_MPU6050_SetByPassI2C();
    if (status != 0) {
        imuStatus = IMU_INIT_ERROR;
        return 1;
    }

    return status;
}

uint8_t driver_MPU6050_IsOnline(void) {
    uint8_t data;
    uint8_t status;

    //Reg 0x75 Bit6:Bit1 is 110 100
    status = driver_MPU6050_receiveData_memory(MPU6050_WHOAMI, &data, 1, 10);
    if (status != 0) {
        return status;
    }
    if ((data & 0x7E) != 0x68) {
        return 2;
    }
    return 0;

}

uint8_t driver_MPU6050_SetSleepMode(uint8_t sleepMode) {
    uint8_t status;
    uint8_t data;

    data = 0x00 | (sleepMode << 6);
    status = driver_MPU6050_transmitData_memory(MPU6050_PWR_MGMT1, &data, 1, 10);
    return status;
}

uint8_t driver_MPU6050_SetAccelRange(int accelRange) {
    uint8_t accelConfigData = 0x00;
    uint8_t status;

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

    status = driver_MPU6050_transmitData_memory(MPU6050_ACCEL_CFG, &accelConfigData, 1, 10);
    return status;
}

uint8_t driver_MPU6050_SetGyroRange(int gyroRange) {
    uint8_t gyroConfigData = 0x00;
    uint8_t status;

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

    status = driver_MPU6050_transmitData_memory(MPU6050_GYRO_CFG, &gyroConfigData, 1, 10);
    return status;
}

uint8_t driver_MPU6050_SetLPF(uint16_t lpf) {
    uint8_t data;
    if (lpf >= 188)data = 1;
    else if (lpf >= 98)data = 2;
    else if (lpf >= 42)data = 3;
    else if (lpf >= 20)data = 4;
    else if (lpf >= 10)data = 5;
    else data = 6;
    return driver_MPU6050_transmitData_memory(MPU6050_CFG, &data, 1, 10);    //设置采样频率分频器
}

uint8_t driver_MPU6050_SetRate(uint32_t rate) {
    uint8_t data;
    if (rate > 1000)rate = 1000;
    if (rate < 4)rate = 4;
    data = 1000 / rate - 1;
    driver_MPU6050_transmitData_memory(MPU6050_SAMPLE_RATE_DIVICER, &data, 1, 10);    //设置采样频率分频器

    return driver_MPU6050_SetLPF(rate / 2);    //自动设置LPF为采样率的一半
}

uint8_t driver_MPU6050_SetByPassI2C(void) {
    uint8_t status;
    uint8_t data;

    data = 0x02;
    status = driver_MPU6050_transmitData_memory(MPU6050_INT_PIN_CFG, &data, 1, 10);
    if (status != 0) {
        return status;
    }

    data = 0x00;
    status = driver_MPU6050_transmitData_memory(MPU6050_USER_CTRL, &data, 1, 10);
    if (status != 0) {
        return status;
    }

    return status;
}

uint8_t driver_MPU6050_GetRawData(void) {
    uint8_t rawResponseData[14];
    uint8_t status;

    if (imuStatus == IMU_INIT_ERROR) {
        return 1;
    }

    status = driver_MPU6050_receiveData_memory(MPU6050_ACCEL_OUT, rawResponseData, 14, 10);

    if (status != 0) {
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

    return 0;
}

void driver_MPU6050_CalibrateData(void) {
    vector3f_t acc_data, gyro_data;

#ifdef MPU6050_FILTER
    acc_data.x = lowPassFilter(imu_lowPass, imu_last_raw_data.acc.x, imu_raw_data.acc.x);
    acc_data.y = lowPassFilter(imu_lowPass, imu_last_raw_data.acc.y, imu_raw_data.acc.y);
    acc_data.z = lowPassFilter(imu_lowPass, imu_last_raw_data.acc.z, imu_raw_data.acc.z);

    gyro_data.x = lowPassFilter(imu_lowPass, imu_last_raw_data.gyro.x, imu_raw_data.gyro.x);
    gyro_data.y = lowPassFilter(imu_lowPass, imu_last_raw_data.gyro.y, imu_raw_data.gyro.y);
    gyro_data.z = lowPassFilter(imu_lowPass, imu_last_raw_data.gyro.z, imu_raw_data.gyro.z);

    imu_last_raw_data = imu_raw_data;
#else

    acc_data = imu_raw_data.acc;
    gyro_data = imu_raw_data.gyro;
#endif

    imu_calibrated_data.acc.x = (acc_data.x - imu_offset_error.acc.x) * imu_scale_error.acc.x;
    imu_calibrated_data.acc.y = (acc_data.y - imu_offset_error.acc.y) * imu_scale_error.acc.y;
    imu_calibrated_data.acc.z = (acc_data.z - imu_offset_error.acc.z) * imu_scale_error.acc.z;

    imu_calibrated_data.gyro.x = gyro_data.x - imu_offset_error.gyro.x;
    imu_calibrated_data.gyro.y = gyro_data.y - imu_offset_error.gyro.y;
    imu_calibrated_data.gyro.z = gyro_data.z - imu_offset_error.gyro.z;
}

uint8_t driver_MPU6050_RefreshData(void) {
    uint8_t status;
    status = driver_MPU6050_GetRawData();
    if (status != 0) {
        return 1;
    }
    driver_MPU6050_CalibrateData();
    return 0;
}

void driver_MPU6050_MeasureOffsetError_Avg(int sampleSize) {
    vector3f_t acc_sum = {0, 0, 0}; //对多次采样加速度数值求和
    vector3f_t gyro_sum = {0, 0, 0}; //对多次采样角速度数值求和
    for (int i = 0; i < sampleSize; ++i) {
        driver_MPU6050_GetRawData();

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

    printf("acc offset error:%.2f %.2f %.2f\n", imu_offset_error.acc.x, imu_offset_error.acc.y,
           imu_offset_error.acc.z);
    printf("gyro offset error:%.2f %.2f %.2f\n", imu_offset_error.gyro.x, imu_offset_error.gyro.y,
           imu_offset_error.gyro.z);
}

void driver_MPU6050_TransmitRawData(void) {
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

void driver_MPU6050_TransmitCalibratedData(void) {
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

void driver_MPU6050_ACC_TransmitRawAndCalibratedData_Custom(void) {
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

void driver_MPU6050_TransmitAttitude(void) {
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
