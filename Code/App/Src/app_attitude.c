/**
 *  @brief      姿态解算应用
 *  @details    负责GY86数据采集，姿态解算任务
 *  @author     Harry-Qu
 *  @date       2022.04
 *  @version    1.3.0
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
 *  			1.0.0   |		实现基本功能
 *  			1.1.0   |       支持OSTimeDly延时方式，修复初始化时延时的bug
 *  			1.1.1   |       将任务修改为定时器调用方式，周期控制更加精准
 *  			1.2.0   |       新增8字校准法，使用EEPROM保存校准数据
 *  			1.3.0   |       新增数据互斥访问功能
 *
 *
 *  @note
 *              姿态解算任务初始化：app_attitude_init();
 *              姿态解算任务：app_attitude_task
 *
*/

/**
 * ROM中数据存储结构，采用clock置换法
 * 0x00 u8  误差数据组数，合法值:[0,6]
 * 0x01 u8  误差数据组数,同0x00，做校验用
 * 0x02 u8  下一次写入应替换的数据序号，合法值:[0,5]
 *
 * 0x10-0xCF mag_calibration_data_t(32B)    误差数据
 */

#include "driver_at24c0x.h"
#include "driver_rgb.h"
#include "app_attitude.h"
#include "AHRS.h"
#include "sdk_math.h"

static OS_EVENT *sem_attitude_task;
float debugMagExpectLength, debugMagFactLength;
OS_EVENT *sem_attitude_data;

quat_t attitude;
vector3f_t attitudeAngle;

#define IMU_GYRO_AVG_SAMPLE_SIZE 5
vector3f_t imu_gyro_avg_data = {0};
vector3f_t imu_gyro_calibrated_history_data[IMU_GYRO_AVG_SAMPLE_SIZE] = {0};
uint8_t imu_gyro_avg_index = 0;

/**
 * 计算校准数据校验和
 * @param data 校准数据
 * @param check 校验和（结果输出）,u8*2
 */
void app_attitude_calibrate_mag_CalValidateCode(const uint8_t *data, uint8_t *check) {
    uint8_t addCheck = 0, sumCheck = 0;
    uint8_t i = 0;
    for (i = 0; i < 28; ++i) {
        sumCheck += data[i];
        addCheck += sumCheck;
    }
    check[0] = sumCheck;
    check[1] = addCheck;
}

/**
 * 从ROM中获取数据校准数据并检验是否符合当前磁场
 * @note 若符合当前磁场要求，则将数据作为误差数据使用。
 * @return 检查结果
 *          0:成功
 *          1:失败
 */
uint8_t app_attitude_calibrate_mag_getErrorDataByRom(void) {
    struct {
        uint8_t dataSize, dataSizeValidate;
    } sizeData;
    mag_calibration_data_t mag_calibration_data[MAG_CALIBRATION_MAX_SIZE];
    uint8_t check[2];
    uint8_t status;

    printf("read ROM data.\n");

    status = driver_at24c0x_readData_page(MAG_CALIBRATION_ROM_PAGE, 0x00, &sizeData, 2);
    printf("dataSize:%d.\n", sizeData.dataSize);
    if (status > 0 || sizeData.dataSize != sizeData.dataSizeValidate || sizeData.dataSize > 10) {
        return 1;
    }

    status = driver_at24c0x_readData_page(MAG_CALIBRATION_ROM_PAGE, 0x10, mag_calibration_data,
                                          sizeof(mag_calibration_data_t) * sizeData.dataSize);
    if (status > 0) {
        printf("[warning] read Detail Error Data.\n");
        return 1;
    }

    status = driver_HMC5883_GetRawData();
    if (status > 0) {
        printf("[warning] read MAG Data.\n");
        return 1;
    }

    for (uint8_t i = 0; i < sizeData.dataSize; i++) {
        mag_calibration_data_t *pmag_calibration_data = &mag_calibration_data[i];
        app_attitude_calibrate_mag_CalValidateCode((uint8_t *) pmag_calibration_data, check);
//        printf("%02X %02X.\n", check[0], check[1]);
//        printf("%02X %02X.\n", pmag_calibration_data->sumCheck, pmag_calibration_data->addCheck);
        if (check[0] != pmag_calibration_data->sumCheck || check[1] != pmag_calibration_data->addCheck) {
            printf("[warning] validate Data.\n");
            continue;
        }

        float length;
        length = sqrtf(
                SQ((mag_raw_data.x - pmag_calibration_data->offset_error.x) * pmag_calibration_data->scale_error.x) +
                SQ((mag_raw_data.y - pmag_calibration_data->offset_error.y) * pmag_calibration_data->scale_error.y) +
                SQ((mag_raw_data.z - pmag_calibration_data->offset_error.z) * pmag_calibration_data->scale_error.z));
        if (fabsf(length - pmag_calibration_data->magLength) < MAG_Allowable_error) {
            printf("[OK] check pass.\n");
            debugMagExpectLength = pmag_calibration_data->magLength;
            mag_offset_error = pmag_calibration_data->offset_error;
            mag_scale_error = pmag_calibration_data->scale_error;
            return 0;
        }
        printf("[warning] check.\tfactLength:%.2f,expectLength:%.2f.\n", length, pmag_calibration_data->magLength);
    }
    return 1;
}

/**
 * 8字测量地磁误差
 */
void app_attitude_MeasureError_8shape(void) {
    mag_t mag_min_data, mag_max_data;
    uint32_t lastTick = 0;

    float detaTime = 0.05f;
    float minRadian = 0, maxRadian = 0, radian = 0;   //当前转动的最小角度与最大角度，用于计算是否转满一圈
    float ax = 0, ay = 0, az = 0, recipNorm;
    uint8_t finishAxisNum = 0;    // 已校准的轴的数量
    uint8_t finishAxis = 0; // 已校准的轴,按位表示,bit0=x,bit1=y,bit2=z
    uint8_t nowAxis = 0;    //正在校准的轴,按位表示

    driver_rgb_setColor(YELLOW, 1);

    while (finishAxisNum < 1) {
        driver_MPU6050_RefreshData();
        driver_HMC5883_GetRawData();

        if (lastTick) {
            detaTime = (HAL_GetTick() - lastTick) / 1000.0f;
        }
        lastTick = HAL_GetTick();

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
        }

        if (maxRadian - minRadian > 1.3 * PI) {
            finishAxis |= nowAxis;
            finishAxisNum++;
//            printf("完成一个轴.\n");
        }

        HAL_Delay(2);
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
    driver_rgb_setColor(YELLOW, 5);
}

/**
 * 将误差数据写入ROM中
 * @return 写入结果
 *         0:成功
 *         1:失败
 */
uint8_t app_attitude_calibrate_mag_recordErrorDataToRom(void) {
    struct {
        uint8_t dataSize, dataSizeValidate;
        uint8_t writeIndex;
    } sizeData;

    uint8_t status = driver_at24c0x_readData_page(MAG_CALIBRATION_ROM_PAGE, 0x00, &sizeData, 3);
    if (status != 0) {
        return 1;
    }
    if (sizeData.dataSize != sizeData.dataSizeValidate || sizeData.dataSize > MAG_CALIBRATION_MAX_SIZE ||
        sizeData.writeIndex >= MAG_CALIBRATION_MAX_SIZE) {
        //需重置整个存储区
        sizeData.dataSize = sizeData.dataSizeValidate = 0;  //当前数据数为0
        sizeData.writeIndex = 0;    //写入序号0的位置
    } else {
        uint16_t property;
        if (sizeData.dataSize == MAG_CALIBRATION_MAX_SIZE) {
            //当前没有剩余空间
            while (1) {
                //找到一个被替换的位置
                uint8_t address = 0x10 + 0x20 * sizeData.writeIndex;
                uint8_t readStatus = driver_at24c0x_readData_page(MAG_CALIBRATION_ROM_PAGE, address + 0x1C, &property,
                                                                  sizeof(property));
                if (readStatus != 0) {
                    //读取属性失败
                    break;
                }
                if ((property & 0x0001) == 0) {
                    //该数据最近未被使用，可以被替换
                    break;
                }
                property &= ~(0x0001);
                driver_at24c0x_writeData_page(MAG_CALIBRATION_ROM_PAGE, address, &property,
                                              sizeof(property));
                sizeData.writeIndex = (sizeData.writeIndex + 1) % MAG_CALIBRATION_MAX_SIZE;
            }
        }
    }
    float length;
    length = sqrtf(SQ(mag_calibrated_data.x) + SQ(mag_calibrated_data.y) + SQ(mag_calibrated_data.z));
    debugMagExpectLength = length;
    mag_calibration_data_t mag_calibration_data = {
            .scale_error=mag_scale_error,
            .magLength=length,
            .offset_error=mag_offset_error,
            .property=0x0001
    };
    app_attitude_calibrate_mag_CalValidateCode((const uint8_t *) &mag_calibration_data, &mag_calibration_data.sumCheck);
    printf("%02X %02X.\n", mag_calibration_data.sumCheck, mag_calibration_data.addCheck);
    uint8_t address = 0x10 + 0x20 * sizeData.writeIndex;

    status = driver_at24c0x_writeData_page(MAG_CALIBRATION_ROM_PAGE, address, &mag_calibration_data,
                                           sizeof(mag_calibration_data_t));
    printf("writeErrorData status:%d\n", status);
    if (sizeData.dataSize < MAG_CALIBRATION_MAX_SIZE) {
        sizeData.dataSize = sizeData.dataSizeValidate = sizeData.dataSize + 1;
    }
    sizeData.writeIndex = (sizeData.writeIndex + 1) % MAG_CALIBRATION_MAX_SIZE;
    status = driver_at24c0x_writeData_page(MAG_CALIBRATION_ROM_PAGE, 0x00, &sizeData, sizeof(sizeData));
    printf("writeSizeData status:%d\n", status);
    return status;
}

/**
 * 校准磁力计
 */
void app_attitude_calibrate_mag(void) {

    uint8_t status = app_attitude_calibrate_mag_getErrorDataByRom();
    if (status == 0) {
        //rom里有校准的数据且通过检验
        return;
    }

    app_attitude_MeasureError_8shape();
    driver_HMC5883_RefreshData();
    app_attitude_calibrate_mag_recordErrorDataToRom();

}

void app_attitude_init(void) {
    driver_gy86_Init(GY86_IMU | GY86_MAG | GY86_APG);  //初始化GY86各芯片

//    app_attitude_calibrate_mag();

    driver_HMC5883_RefreshData();
    AHRS_InitQuat_MAG(&attitude, mag_calibrated_data);    //计算一个初始的四元数

    sem_attitude_data = OSSemCreate(1);
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

    uint8_t status;

    while (1) {
        OSSemPend(sem_attitude_task, 0, &err);
        OS_TRACE_MARKER_START(2);

        status = driver_MPU6050_GetRawData();
        if (status == 0) {
            driver_MPU6050_CalibrateData();
            OSSemPend(sem_attitude_data, OS_TICKS(2), &err);
            imu_gyro_avg_data.x -= imu_gyro_calibrated_history_data[imu_gyro_avg_index].x / IMU_GYRO_AVG_SAMPLE_SIZE;
            imu_gyro_avg_data.x += imu_calibrated_data.gyro.x / IMU_GYRO_AVG_SAMPLE_SIZE;
            imu_gyro_avg_data.y -= imu_gyro_calibrated_history_data[imu_gyro_avg_index].y / IMU_GYRO_AVG_SAMPLE_SIZE;
            imu_gyro_avg_data.y += imu_calibrated_data.gyro.y / IMU_GYRO_AVG_SAMPLE_SIZE;
            imu_gyro_avg_data.z -= imu_gyro_calibrated_history_data[imu_gyro_avg_index].z / IMU_GYRO_AVG_SAMPLE_SIZE;
            imu_gyro_avg_data.z += imu_calibrated_data.gyro.z / IMU_GYRO_AVG_SAMPLE_SIZE;
            OSSemPost(sem_attitude_data);
            imu_gyro_calibrated_history_data[imu_gyro_avg_index] = imu_calibrated_data.gyro;
            imu_gyro_avg_index = (imu_gyro_avg_index + 1) % IMU_GYRO_AVG_SAMPLE_SIZE;
        }

        status = driver_HMC5883_GetRawData();
        if (status == 0) {
            OSSemPend(sem_attitude_data, OS_TICKS(2), &err);
            driver_HMC5883_CalibrateData();
            OSSemPost(sem_attitude_data);
        }
        debugMagFactLength = sqrtf(SQ(mag_calibrated_data.x) + SQ(mag_calibrated_data.y) + SQ(mag_calibrated_data.z));

        OSSemPend(sem_attitude_data, OS_TICKS(2), &err);
//        driver_gy86_Attitude_Update_Madgwick();
        AHRS_MadgWickTest(&attitude, imu_calibrated_data.acc, imu_calibrated_data.gyro, mag_calibrated_data);
//        driver_gy86_Attitude_Update_Mahony();
        OSSemPost(sem_attitude_data);
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
