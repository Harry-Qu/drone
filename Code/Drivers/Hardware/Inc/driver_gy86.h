/**
 *  @brief      GY86驱动
 *  @details    提供GY86初始化、数据接收、数据解析操作
 *  @author     Harry-Qu
 *  @date       2021.10
 *  @version    1.3
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
                1.3     |       拆分各芯片驱动代码

*/


#ifndef __GY86_H
#define __GY86_H

#include "app_cfg.h"
#include "i2c.h"
#include "driver_mpu6050.h"
#include "driver_hmc5883.h"
#include "driver_ms5611.h"


#define RETRY_TIME 10   //初始化失败时尝试次数

#define GY86_Delay HAL_Delay
#define GY86_I2C hi2c1

//用于识别需要初始化和获取数据的设备
#define GY86_IMU 0x01
#define GY86_APG 0x02
#define GY86_MAG 0x04



extern quat_t attitude;
extern vector3f_t attitudeAngle;



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
uint8_t driver_gy86_Init(uint8_t init_device_id);

/**
 * @brief 更新GY86数据
 * @param refresh_device_id 需要更新的设备编号，示例值:GY86_IMU|GY86_APG|GY86_MAG
 * @retval None
 */
uint8_t driver_gy86_Refresh_Data(uint8_t refresh_device_id);


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


#endif  //__GY86_H
