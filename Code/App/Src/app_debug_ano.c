/** 
 *  @brief	    调试任务-匿名上位机部分
 *  @details    该文件内函数会被周期性调用，用于向匿名上位机发送数据
 *  @author     Harry-Qu
 *  @date       2022/11/2
 *  @version    1.1.1
 *  @par        日志
 *              1.0     |       实现发送数据帧给匿名上位机的基础功能
 *              1.1     |       新增数据互斥访问功能
 *                              新增发送PID数据功能
 *              1.1.1   |       新增0xFB刷新帧功能
*/

#include "app_debug_ano.h"
#include "driver_gy86.h"
#include "sdk_ano.h"
#include "AHRS.h"
#include "app_control.h"

static char replyMsg[ANO_MAX_LENGTH];
static uint8_t isReply;

extern float debugMagExpectLength, debugMagFactLength;
extern vector3f_t attitudeAngle;
extern OS_EVENT *sem_pid_data, *sem_attitude_data;
extern vector3f_t imu_gyro_avg_data;

void app_debug_ano_log(char *msg) {
    if (strlen(msg) > ANO_MAX_LENGTH - 1) {
        msg[ANO_MAX_LENGTH - 1] = '\0';
    }
    strcpy(replyMsg, msg);
    isReply = 1;
}

static void app_debug_ano_transmitPid(pid_type *pid) {
    struct {
        int32_t piIntegral;
        int32_t pidOut;
        int32_t pidPOut;
        int32_t pidIOut;
        int32_t pidDOut;
        int32_t pidTarget;
    } sendData;
    uint8_t err;

    OSSemPend(sem_pid_data, 10, &err);
    sendData.piIntegral = (int32_t) (pid->integral * 1000);
    sendData.pidOut = (int32_t) (pid->out * 1000);
    sendData.pidPOut = (int32_t) (pid->pOut * 1000);
    sendData.pidIOut = (int32_t) (pid->iOut * 1000);
    sendData.pidDOut = (int32_t) (pid->dOut * 1000);
    sendData.pidTarget = (int32_t) (pid->target * 1000);
    OSSemPost(sem_pid_data);
    sdk_ano_transmit_custom_data(0xF9, 24, (uint8_t *) &sendData);
}

static void app_debug_ano_transmitMag() {
    struct {
        int16_t magFactLength;
        int16_t magExpectLength;
        uint16_t magDeltaLength;
    } sendData;

    sendData.magFactLength = (int16_t) (debugMagFactLength * 10000);
    sendData.magExpectLength = (int16_t) (debugMagExpectLength * 10000);
//    printf("%.2f ", fabsf(debugMagFactLength - debugMagExpectLength));
    sendData.magDeltaLength = (uint16_t) (fabsf(debugMagFactLength - debugMagExpectLength) * 10000);
    sdk_ano_transmit_custom_data(0xF8, 6, (uint8_t *) &sendData);

}

static void app_debug_ano_transmitAttitude() {
    struct {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        int16_t step;
    } sendData;

    sendData.roll = (int16_t) (attitudeAngle.x * 100);
    sendData.pitch = (int16_t) (attitudeAngle.y * 100);
    sendData.yaw = (int16_t) (attitudeAngle.z * 100);
    sendData.step = (int16_t) (stepSize * 100);
    sdk_ano_transmit_custom_data(0xF6, 8, (uint8_t *) &sendData);
}

static void app_debug_ano_transmitLog() {
    if (isReply) {
        isReply = 0;
        sdk_ano_transmit_log_string(1, replyMsg);
    }
}

void app_debug_ano(void) {
//    printf("%lu\n", HAL_GetTick());
//    driver_gy86_TransmitAttitude_Quat(&attitude);
    uint8_t err;

//    app_control_transmitMotorSpeed();//14B
//    OSTimeDly(OS_TICKS(7));
//    driver_MPU6050_TransmitCalibratedData();

    struct {
        int16_t ACC_X;
        int16_t ACC_Y;
        int16_t ACC_Z;
        int16_t GYR_X;
        int16_t GYR_Y;
        int16_t GYR_Z;
        uint8_t SHOCK_STA;
    } send_calibrated_data;

    OSSemPend(sem_attitude_data, 20, &err);
    send_calibrated_data.ACC_X = (int16_t) (imu_calibrated_data.acc.x * 1000);
    send_calibrated_data.ACC_Y = (int16_t) (imu_calibrated_data.acc.y * 1000);
    send_calibrated_data.ACC_Z = (int16_t) (imu_calibrated_data.acc.z * 1000);

    send_calibrated_data.GYR_X = (int16_t) (imu_gyro_avg_data.x * 1000);
    send_calibrated_data.GYR_Y = (int16_t) (imu_gyro_avg_data.y * 1000);
    send_calibrated_data.GYR_Z = (int16_t) (imu_gyro_avg_data.z * 1000);
    OSSemPost(sem_attitude_data);

    send_calibrated_data.SHOCK_STA = 0;

//    sdk_ano_transmit_inertial_sensor_data(&send_calibrated_data);//19B
//
//    OSTimeDly(OS_TICKS(10));

//    AHRS_ConvertQuatToDegree(&attitudeTest, &attitudeAngleTest);
    driver_gy86_TransmitAttitude_Angle(&attitudeAngle);//13B
    OSTimeDly(OS_TICKS(2));
//    driver_HMC5883_TransmitCalibratedData_Custom();

    app_debug_ano_transmitLog();


    app_debug_ano_transmitPid(&pidRollOuter);//30B
    OSTimeDly(OS_TICKS(5));

//    app_debug_ano_transmitMag();
//    app_debug_ano_transmitAttitude();
//    driver_HMC5883_TransmitRawAndCalibratedData_Custom();

    sdk_ano_transmit_refresh_frame();//6B
}


