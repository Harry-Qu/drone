/** 
 *  @brief	    MPU6050驱动
 *  @details    提供MPU6050初始化，数据获取，数据校准等基本操作。
 *  @author     Harry-Qu
 *  @date       2022/11/23
 *  @version    1.1
 *  @par        日志
 *              1.0     |       完成MPU6050驱动基本功能
 *              1.1     |       修改部分全局变量为静态变量
 *                              新增固定误差功能
*/

#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include "dataType.h"
#include "i2c.h"

/** =========配置========= **/

#define MPU6050_I2C hi2c1

#define MPU6050_FILTER  //是否对数据滤波

//#define FIXED_ERROR //是否采用固定误差

#define ACC_OFFSET_ERROR_X 0.04f
#define ACC_OFFSET_ERROR_Y 0.02f
#define ACC_OFFSET_ERROR_Z -0.12f

#define GYRO_OFFSET_ERROR_X -0.02f
#define GYRO_OFFSET_ERROR_Y -0.03f
#define GYRO_OFFSET_ERROR_Z 0.02f

/** =========配置========= **/

#define MPU6050_ADDRESS    (uint8_t)0x68    //MPU6050设备I2C地址
#define MPU6050_WRITE_ADDRESS    (MPU6050_ADDRESS<<1)
#define MPU6050_READ_ADDRESS    ((MPU6050_ADDRESS<<1) | 1)


#define MPU6050_SAMPLE_RATE_DIVICER 0x19    //采样率分频器
#define MPU6050_CFG 0x1A
#define MPU6050_GYRO_CFG 0x1B    //陀螺仪配置
#define MPU6050_ACCEL_CFG 0x1C    //加速度传感器配置
#define MPU6050_INT_PIN_CFG 0x37    //INT Pin / Bypass Enable Configuration Register
#define MPU6050_ACCEL_OUT 0x3B  //加速度输出寄存器
#define MPU6050_USER_CTRL 0x6A    //User Control Register
#define MPU6050_PWR_MGMT1 0x6B    //电源管理
#define MPU6050_WHOAMI 0x75

#define GRAVITATIONAL_ACCELERATION 9.80665f //重力加速度

typedef struct {
    vector3f_t acc;  //加速度
    vector3f_t gyro;  //角速度
    float temp, valid;  //温度
} imu_t;    //IMU数据结构体

typedef struct {
    vector3f_t acc;  //加速度
    vector3f_t gyro;  //角速度
} imu_error_t; //IMU零偏误差结构体

typedef struct {
    float accel;
    float gyro;
} imu_config_factor_t;  //IMU比例数据

enum IMU_STATUS_CODE{
    IMU_OK = 0,
    IMU_INIT_ERROR,
    IMU_REFRESH_ERROR,
};

extern enum IMU_STATUS_CODE imuStatus;
extern imu_t imu_calibrated_data;


/**
 * @brief 初始化IMU加速度计
 * @param accelRange 线加速度测量范围，可取值为:2(默认)、4、8、16
 * @param gyroRange	角加速度测量范围，可取值为:250(默认)、500、1000、2000
 * @return 初始化结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_Init(int accelRange, int gyroRange);

/**
 * 获取MPU6050在线状态
 * @details 在线状态检测方式包含：
 *          通过I2C访问MPU6050，是否有ACK
 *          获取MPU6050 WHO_AM_I寄存器，返回值是否为0x68
 * @return 在线结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_IsOnline(void);

/**
 * 设置MPU6050睡眠模式
 * @note 初始化时必须通过此接口唤醒MPU6050
 * @param sleepMode 睡眠模式
 *                  0: 唤醒
 *                  1: 低功耗睡眠模式
 * @return 设置结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_SetSleepMode(uint8_t sleepMode);

/**
 * 设置MPU6050线加速度量程范围
 * @param accelRange 量程范围(g/s)，可取2,4,6,8
 * @return 设置结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_SetAccelRange(int accelRange);

/**
 * 设置MPU6050角加速度量程范围
 * @param gyroRange 量程范围(d/s)，可取250, 500, 1000, 2000
 * @return 设置结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_SetGyroRange(int gyroRange);

/**
 * 设置IMU低通滤波
 * @param lpf 低通滤波频率
 * @return 设置结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_SetLPF(uint16_t lpf);

/**
 * 设置IMU采样率
 * @param rate 采样率
 * @return 设置结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_SetRate(uint32_t rate);

/**
 * 释放MPU6050对旁路I2C设备控制
 * @return 设置结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_SetByPassI2C(void);

/**
 * 获取IMU原始数据
 * @details 从MPU6050中读取原始数据，并将数据保存至imu_raw_data中。其中，加速度单位为g，角速度单位为rad/s
 * @return 获取结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_GetRawData(void);

/**
 * 对IMU的数据进行校准
 * @details 从imu_raw_data获取数据，对于IMU的加速度值消除零偏误差与刻度误差，对于角速度数据消除零偏误差，将结果保存至imu_calibrate_data
 */
void driver_MPU6050_CalibrateData(void);

/**
 * @brief 刷新IMU数据
 * @details 从MPU6050读取数据并对数据进行校准，结果保存至imu_calibrate_data中
 * @return 数据刷新结果
 *         0: success
 *         1: fail
 */
uint8_t driver_MPU6050_RefreshData(void);

/**
 * 使用均值法测量计算IMU加速度计与陀螺仪的零偏误差
 * @details 使用均值法计算IMU加速度计与陀螺仪的零偏误差，结果保存至imu_offset_error中
 * @param sampleSize 采样的原始数据数量
 */
void driver_MPU6050_MeasureOffsetError_Avg(int sampleSize);

/**
 * 发送原始IMU数据给匿名上位机
 */
void driver_MPU6050_TransmitRawData(void);

/**
 * 发送校准后的IMU数据给匿名上位机
 */
void driver_MPU6050_TransmitCalibratedData(void);

/**
 * 以自定义形式发送原始与校准后加速度数据给匿名上位机
 */
void driver_MPU6050_ACC_TransmitRawAndCalibratedData_Custom(void);

/**
 * 向匿名上位机发送姿态数据
 * @details 此函数采用加速度计的值直接计算俯仰角与横滚角，偏航角大小为0。
 */
void driver_MPU6050_TransmitAttitude(void);

#endif //MPU6050_H
