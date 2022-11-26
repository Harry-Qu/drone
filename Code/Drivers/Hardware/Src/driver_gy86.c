/**
 *  @brief      GY86驱动
 *  @details    提供GY86初始化、数据接收、数据解析操作
 *  @author     Harry-Qu
 *  @date       2021.10
 *  @version    1.3
 *  @par        Copyright (c):  四轴小组
 *  @par        版本变更:
 *  			1.0		|		实现初始化、数据接收、数据解析等函数
 *  			1.1     |       修改部分IMU数据结构体
 *  			                修改磁力计处理数据的相关bug
 *  			                新增四元数结构体
 *  			                新增均值法计算IMU零偏误差功能
 *  			                新增高斯牛顿迭代法计算零偏误差与刻度误差功能（没啥用）
 *  			                新增8字校准法测量磁力计误差功能
 *  			                新增计算加速度计校准后数据功能
 *  			                新增向匿名上位机发送数据功能
 *  			                新增梯度下降法计算姿态（四元数）功能
 *  			                新增madgwick计算姿态（四元数）功能
 *              1.2     |       降低与sdk_i2c, sdk_ano的耦合，注释该头文件后仍可正常运行此代码，但无法提供DMA，数据发送功能
 *              1.3     |       拆分各芯片驱动代码
 *                              修改部分函数体状态返回类型为uint8_t
 *                              去除姿态测试代码
 *                              修改纯梯度下降法算法
 *
*/


#include <memory.h>
#include "driver_gy86.h"
#include "sdk_math.h"
#include "sdk_i2c.h"
#include "sdk_ano.h"
#include "sdk_time.h"
#include "filter.h"
#include "AHRS.h"


enum GY86_STATUS_CODE gy86_status = 0;  //GY86状态

quat_t attitude;
vector3f_t attitudeAngle;

static void driver_gy86_init_args(void) {

}

uint8_t driver_gy86_Init(uint8_t init_device_id) {
    uint32_t retryTime = 0;
    uint8_t status;

    driver_gy86_init_args();

    if (init_device_id & GY86_IMU) {
        do {
            status = driver_MPU6050_Init(8, 500);
        } while (status != 0 && (retryTime++) < RETRY_TIME);

        if (status != 0) {
            gy86_status |= INIT_IMU_ERROR;
            perror("init imu fail.\n");
        } else {
            driver_MPU6050_MeasureOffsetError_Avg(200);
        }
    }

    if (init_device_id & GY86_MAG) {
        retryTime = 0;

        do {
            status = driver_HMC5883_Init();
        } while (status != 0 && (retryTime++) < RETRY_TIME);

        if (status != 0) {
            gy86_status |= INIT_MAG_ERROR;
            perror("init mag fail.\n");
        } else {

        }
    }

    if (init_device_id & GY86_APG) {
        retryTime = 0;

        do {
            status = driver_MS5611_Init(0);
        } while (status != 0 && (retryTime++) < RETRY_TIME);

        if (status != 0) {
            gy86_status |= INIT_APG_ERROR;
            perror("init apg fail.\n");
        } else {

        }
    }

    if (gy86_status & (INIT_IMU_ERROR | INIT_MAG_ERROR | INIT_APG_ERROR)) {
        return 1;
    } else {
        return 0;
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

uint8_t driver_gy86_Refresh_Data(uint8_t refresh_device_id) {
    uint8_t status = 0;

    if (refresh_device_id & GY86_IMU) {
        if (driver_MPU6050_RefreshData() != 0) {
            status = 1;
        }
    }

    if (refresh_device_id & GY86_MAG) {
        if (driver_HMC5883_RefreshData() != 0) {
            status = 1;
        }
    }

    if (refresh_device_id & GY86_APG) {
        if (driver_MS5611_Refresh_Data() != 0) {
            status = 1;
        }
    }

    return status;
}

void driver_gy86_Attitude_InitQuat_MAG(void) {
    driver_HMC5883_RefreshData();

    float roll = 0, pitch = 0, yaw = 0;

    yaw = atan2f(mag_calibrated_data.y, mag_calibrated_data.x);

    yaw = -yaw;

    attitude.a = cosf(yaw / 2);
    attitude.b = 0;
    attitude.c = 0;
    attitude.d = sinf(yaw / 2);
}

void driver_gy86_Attitude_InitQuat_IMUAndAXIS(void) {

    float cosRoll, cosPitch, cosYaw;
    float sinRoll, sinPitch, sinYaw;

    float roll = 0, pitch = 0, yaw = 0;

    driver_MPU6050_RefreshData();
    driver_HMC5883_RefreshData();

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

//void driver_gy86_Attitude_Update_Grad(void) {
//    //TODO 修改纯梯度下降法
//    const float step = 0.02f;
//    const float esp = 0.1f;
//    const uint32_t maxCnt = 20;
//    float JtFq[6] = {0}, Fq[6] = {0};
//    float JtFqLength;
//    float a = attitude.a, b = attitude.b, c = attitude.c, d = attitude.d;
//    float ax = imu_calibrated_data.acc.x, ay = imu_calibrated_data.acc.y, az = imu_calibrated_data.acc.z;
//    float mx = mag_calibrated_data.x, my = mag_calibrated_data.y, mz = mag_calibrated_data.z;
//
//    uint32_t cnt = 0;
//    float bx = mag_ground_data.x, by = mag_ground_data.y, bz = mag_ground_data.z;
//
//
//    //对数据归一化
//    float invMLength = invSqrt((float) SQR(mx) + SQR(my) + SQR(mz));
//    mx *= invMLength;
//    my *= invMLength;
//    mz *= invMLength;
//
//    float invALength = invSqrt((float) SQR(ax) + SQR(ay) + SQR(az));
//    ax *= invALength;
//    ay *= invALength;
//    az *= invALength;
//
//
//    while ((cnt++) < maxCnt) {
//        float aa = a * a, ab = a * b, ac = a * c, ad = a * d;
//        float bb = b * b, bc = b * c, bd = b * d;
//        float cc = c * c, cd = c * d;
//        float dd = d * d;
//
//
//        Fq[0] = 2 * bd - 2 * ac - ax;
//        Fq[1] = 2 * ab + 2 * cd - ay;
//        Fq[2] = (1 - 2 * bb - 2 * cc) - az;
//
//        Fq[3] = bx * (1 - 2 * cc - 2 * dd) + 2 * bz * (bd - ac) - mx;
//        Fq[4] = 2 * bx * (bc - ad) + 2 * bz * (ab + bd) - my;
//        Fq[5] = 2 * bx * (ac + bd) + bz * (1 - 2 * bb - 2 * cc) - mz;
//
////        Fq[3] = bx * (1 - 2 * cc - 2 * dd) + 2 * by * (bc + ad) + 2 * bz * (bd - ac) - mag_calibrated_data.x;
////        Fq[4] = 2 * bx * (bc - ad) + by * (1 - 2 * cc - 2 * dd) + 2 * bz * (ab + bd) - mag_calibrated_data.y;
////        Fq[5] = 2 * bx * (ac + bd) + 2 * by * (-ab + cd) + bz * (1 - 2 * bb - 2 * cc) - mag_calibrated_data.z;
//
//        //以下数据约掉了*2
//        JtFq[0] =
//                -c * Fq[0] + b * Fq[1]
//                - bz * c * Fq[3] +
//                (bz * b - bx * d) * Fq[4] +
//                (bx * c) * Fq[5];
//        JtFq[1] = d * Fq[0] + a * Fq[1] - 2 * b * Fq[2]
//                  + bz * d * Fq[3] +
//                  (bx * c + bz * a) * Fq[4] +
//                  (bx * d - 2 * bz * b) * Fq[5];
//        JtFq[2] = -a * Fq[0] + d * Fq[1] - 2 * c * Fq[2]
//                  + (-2 * bx * c - bz * a) * Fq[3] +
//                  (bx * b + bz * d) * Fq[4] +
//                  (bx * a - 2 * bz * c) * Fq[5];
//        JtFq[3] =
//                b * Fq[0] + c * Fq[1] +
//                (bz * b - 2 * bx * d) * Fq[3] +
//                (-bx * a + bz * c) * Fq[4] +
//                (bx * b) * Fq[5];
//
////        JtFq[0] = -c * Fq[0] + b * Fq[1] +
////                  (by * d - bz * c) * Fq[3] +
////                  (bz * b - bx * d) * Fq[4] +
////                  (-by * b + bx * c) * Fq[5];
////        JtFq[1] = d * Fq[0] + a * Fq[1] - 2 * b * Fq[2]
////                  + (by * c + bz * d) * Fq[3] +
////                  (bx * c + bz * a) * Fq[4] +
////                  (bx * d - by * a - 2 * bz * b) * Fq[5];
////        JtFq[2] = -a * Fq[0] + d * Fq[1] - 2 * c * Fq[2]
////                  + (-2 * bx * c + by * b - bz * a) * Fq[3] +
////                  (bx * b - 2 * by * c + bz * d) * Fq[4] +
////                  (bx * a + by * d - 2 * bz * c) * Fq[5];
////        JtFq[3] = b * Fq[0] + c * Fq[1] +
////                  (bz * b + by * a - 2 * bx * d) * Fq[3] +
////                  (-bx * a - 2 * by * d + bz * c) * Fq[4] +
////                  (bx * b + by * c) * Fq[5];
//
//        JtFqLength = sqrtf(SQR(JtFq[0]) + SQR(JtFq[1]) + SQR(JtFq[2]) + SQR(JtFq[3]));
//
//        if (JtFqLength < esp) {
////            driver_gy86_TransmitAttitude_Quat(&attitude);
//            break;
//        }
//
//        a -= step * JtFq[0] / JtFqLength;
//        b -= step * JtFq[1] / JtFqLength;
//        c -= step * JtFq[2] / JtFqLength;
//        d -= step * JtFq[3] / JtFqLength;
//    }
//
//
//    attitude.a = a;
//    attitude.b = b;
//    attitude.c = c;
//    attitude.d = d;
//
//
//}

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


