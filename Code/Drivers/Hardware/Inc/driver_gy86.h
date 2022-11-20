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


#ifndef __GY86_H
#define __GY86_H

#include "app_cfg.h"


#include "i2c.h"


#define RETRY_TIME 10   //初始化失败时尝试次数

#define GY86_Delay HAL_Delay
#define GY86_I2C hi2c1

#define STANDAR_GAUSS 0.511116f //当地标准磁场
#define STANDAR_VERTICAL_GAUSS  0.381951f   //当地标准磁场水平强度
#define STANDAR_HORIZONTAL_GAUSS  0.339637f //当地标准磁场垂直强度


#define GRAVITATIONAL_ACCELERATION 9.80665f //重力加速度

#define APG_OSR_DEFAULT_VALUE 2048


//用于识别需要初始化和获取数据的设备
#define GY86_IMU 0x01
#define GY86_APG 0x02
#define GY86_MAG 0x04


#define IMU_ADDRESS    (uint8_t)0x68    //MPU6050设备I2C地址
#define IMU_WRITE_ADDRESS    (IMU_ADDRESS<<1)
#define IMU_READ_ADDRESS    ((IMU_ADDRESS<<1) | 1)

#define AXIS_ADDRESS (uint8_t)0x1E    //HMC5883L设备I2C地址
#define AXIS_WRITE_ADDRESS    (AXIS_ADDRESS<<1)
#define AXIS_READ_ADDRESS    ((AXIS_ADDRESS<<1) | 1)

#define APG_ADDRESS (uint8_t)0x77    //MS5611设备I2C地址
#define APG_WRITE_ADDRESS (APG_ADDRESS<<1)
#define APG_READ_ADDRESS ((APG_ADDRESS<<1) | 1)


#define IMU_WHOAMI 0x75
#define IMU_PWR_MGMT1 0x6B    //电源管理
#define IMU_SAMPLE_RATE_DIVICER 0x19    //采样率分频器
#define IMU_CFG 0x1A
#define IMU_GYRO_CFG 0x1B    //陀螺仪配置
#define IMU_ACCEL_CFG 0x1C    //加速度传感器配置
#define IMU_INT_PIN_CFG 0x37    //INT Pin / Bypass Enable Configuration Register
#define IMU_USER_CTRL 0x6A    //User Control Register


#define APG_D1 0x40
#define APG_D2 0x50

enum APG_OSR_VALUE {
    APG_OSR_256 = 0,
    APG_OSR_512 = 2,
    APG_OSR_1024 = 4,
    APG_OSR_2048 = 6,
    APG_OSR_4096 = 8
};



typedef vector3int16_t mag_raw_t;   //地磁原属数据
typedef vector3f_t mag_t;   //地磁处理后数据

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
    uint32_t pressure;    //APG测量温度，单位：0.01mbar，eg:100009=1000.09mbar
    uint32_t temp;            //APG测量大气压，单位:0.01C，eg:2007=20.07C
} apg_t;    //气压计数据

typedef struct {
    uint16_t c1, c2, c3, c4, c5, c6;
} apg_calibration_t;    //APG校准数据，参数意义见文档

typedef struct {
    float accel;
    float gyro;
} imu_config_factor_t;  //IMU比例数据


extern mag_t mag_calibrated_data;
extern imu_t imu_calibrated_data;
extern apg_t apg_data;

extern quat_t attitude;
extern vector3f_t attitudeAngle;

extern quat_t attitudeTest;
extern vector3f_t attitudeAngleTest;
extern quat_t attitudeTest2;
extern vector3f_t attitudeAngleTest2;

extern uint32_t ApgPressure;
extern uint32_t ApgTemp;


enum GY86_STATUS_CODE {
    GY86_OK = 0,
    INIT_IMU_ERROR = (1 << 0),
    INIT_MAG_ERROR = (1 << 1),
    INIT_APG_ERROR = (1 << 2),
    REFRESH_IMU_ERROR = (1 << 3),
    REFRESH_MAG_ERROR = (1 << 4),
    REFRESH_APG_ERROR = (1 << 5),
};

/**
 * @brief 获取GY86当前状态
 * @return 状态码
 */
uint32_t driver_gy86_Get_Status(void);

/**
 * @brief 获取GY86的错误信息
 * @param status 状态码
 * @param errMessage 返回的错误信息字符串
 */
void driver_gy86_Get_Error_Message(uint32_t status, char *errMessage);

/**
  * @brief 初始化GY86
  * @param init_device_id	需要初始化的设备编号，示例值:GY86_IMU|GY86_APG|GY86_MAG
  * @retval None
  */
HAL_StatusTypeDef driver_gy86_Init(uint8_t init_device_id);

/**
 * @brief 更新GY86数据
 * @param refresh_device_id 需要更新的设备编号，示例值:GY86_IMU|GY86_APG|GY86_MAG
 * @retval None
 */
HAL_StatusTypeDef driver_gy86_Refresh_Data(uint8_t refresh_device_id);

/**
 * @brief 初始化IMU加速度计
 * @param accelRange	线加速度测量范围，可取值为:2(默认)、4、8、16
 * @param gyroRange	角加速度测量范围，可取值为:250(默认)、500、1000、2000
 * @retval None
 */
HAL_StatusTypeDef driver_gy86_IMU_Init(int accelRange, int gyroRange);

/**
 * 设置IMU低通滤波
 * @param lpf
 * @return
 */
HAL_StatusTypeDef driver_gy86_IMU_SetLPF(uint16_t lpf);

/**
 * 设置IMU采样率
 * @param rate
 * @return
 */
HAL_StatusTypeDef driver_gy86_IMU_SetRate(uint32_t rate);

/**
 * 获取IMU原始数据
 * @details 从MPU6050中读取原始数据，并将数据保存至imu_raw_data中。其中，加速度单位为g，角速度单位为rad/s
 */
HAL_StatusTypeDef driver_gy86_IMU_GetRawData(void);

/**
 * 对IMU的数据进行校准
 * @details 从imu_raw_data获取数据，对于IMU的加速度值消除零偏误差与刻度误差，对于角速度数据消除零偏误差，将结果保存至imu_calibrate_data
 */
void driver_gy86_IMU_CalibrateData(void);

/**
 * @brief 刷新IMU数据
 * @details 从MPU6050读取数据并对数据进行校准，结果保存至imu_calibrate_data中
 */
HAL_StatusTypeDef driver_gy86_IMU_RefreshData(void);

/**
 * 使用均值法测量计算IMU加速度计与陀螺仪的零偏误差
 * @details 使用均值法计算IMU加速度计与陀螺仪的零偏误差，结果保存至imu_offset_error中
 * @param sampleSize 采样的原始数据数量
 */
void driver_gy86_IMU_MeasureOffsetError_Avg(int sampleSize);

/**
 * 发送原始IMU数据给匿名上位机
 */
void driver_gy86_IMU_TransmitRawData(void);

/**
 * 发送校准后的IMU数据给匿名上位机
 */
void driver_gy86_IMU_TransmitCalibratedData(void);

/**
 * 以自定义形式发送原始与校准后加速度数据给匿名上位机
 */
void driver_gy86_IMU_ACC_TransmitRawAndCalibratedData_Custom(void);

/**
 * 向匿名上位机发送姿态数据
 * @details 此函数采用加速度计的值直接计算俯仰角与横滚角，偏航角大小为0。
 */
void driver_gy86_IMU_TransmitAttitude(void);

/**
 * @brief 初始化磁力计
 * @retval None
 */
HAL_StatusTypeDef driver_gy86_MAG_Init(void);

/**
 * 从磁力计读取原始数据
 * @details 从HMC5883L读取原始数据（数据顺序XZY），并将数据写入mag_raw_data中
 */
HAL_StatusTypeDef driver_gy86_MAG_GetRawData(void);

/**
 * 校准磁力计数据
 * @details 从mag_raw_data读取数据，对数据校准后保存至mag_data中
 */
void driver_gy86_MAG_CalibrateData(void);

/**
 * 刷新磁力计数据
 * @details 从HMC5883L读取原始数据后，对数据校准，保存至mag_data中
 */
HAL_StatusTypeDef driver_gy86_MAG_RefreshData(void);

/**
 * 使用自定义帧向匿名上位机发送磁力计原始数据
 */
void driver_gy86_MAG_TransmitRawData_Custom(void);

/**
 * 使用自定义帧向匿名上位机发送磁力计校准后数据
 */
void driver_gy86_MAG_TransmitCalibratedData_Custom(void);

/**
 * 使用自定义帧向匿名上位机发送磁力计原始数据和校准后数据
 */
void driver_gy86_MAG_TransmitRawAndCalibratedData_Custom(void);

/**
 * 起飞前记录地面地磁数据信息
 */
void driver_gy86_MAG_RecordGroundData(void);

/**
 * 使用8字校准法校准磁力计
 * @details 借助IMU中加速度计，使用8字校准法校准磁力计，误差数据保存在mag_mid_data中
 */
void driver_gy86_MAG_MeasureError_8shape(void);

/**
 * 计算零偏误差与刻度误差
 */
void driver_gy86_MAG_CalculateError_8shape(void);

/**
 * 根据磁力计数据初始化四元数
 */
void driver_gy86_Attitude_InitQuat_MAG(void);

/**
 * 在静态环境下使用梯度下降法计算姿态
 * @details 利用加速度计与磁力计数据，在静态环境下使用梯度下降法计算四轴飞行器姿态。
 */
void driver_gy86_Attitude_Update_Grad(void);

/**
 * madgwick
 */
void driver_gy86_Attitude_Update_Madgwick(void);

/**
 *
 */
void driver_gy86_Attitude_Update_Mahony(void);

/**
 * 发送姿态数据（四元数）给匿名上位机
 */
void driver_gy86_TransmitAttitude_Quat(quat_t *q);

/**
 * 发送姿态数据（欧拉角）给匿名上位机
 */
void driver_gy86_TransmitAttitude_Angle(vector3f_t *angle);

/**
 * @brief 初始化APG气压计
 * @param osr 精确度
 * 参考数值:
 *              4096        0.012mbar
 *              2048        0.018mbar
 *              1024        0.027mbar
 *              512         0.042mbar
 *              256         0.065mbar
 */
HAL_StatusTypeDef driver_gy86_APG_Init(uint32_t osr);

/**
 * @brief 刷新APG的数据
 * @retval None
 */
HAL_StatusTypeDef driver_gy86_APG_Refresh_Data(void);

#endif  //__GY86_H
