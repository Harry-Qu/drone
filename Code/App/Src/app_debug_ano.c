/** 
 *  @brief	    调试任务-匿名上位机部分
 *  @details    该文件内函数会被周期性调用，用于向匿名上位机发送数据
 *  @author     Harry-Qu
 *  @date       2022/11/2
 *  @version    1.0
 *  @par        日志
*/

#include "app_debug_ano.h"
#include "driver_gy86.h"
#include "sdk_ano.h"
#include "AHRS.h"
#include "app_control.h"

static char replyMsg[100];
static uint8_t isReply;

void app_debug_ano_log(char *msg) {
    strcpy(replyMsg, msg);
    isReply = 1;
}

void app_debug_ano(void) {
//    printf("%lu\n", HAL_GetTick());
//    driver_gy86_TransmitAttitude_Quat(&attitude);
    app_control_transmitMotorSpeed();
    driver_MPU6050_TransmitCalibratedData();
//    AHRS_ConvertQuatToDegree(&attitudeTest, &attitudeAngleTest);
    driver_gy86_TransmitAttitude_Angle(&attitudeAngle);
    driver_HMC5883_TransmitCalibratedData_Custom();

    if (isReply) {
        isReply = 0;
        sdk_ano_transmit_log_string(1, replyMsg);
    }


//    struct {
//        int16_t roll;
//        int16_t pitch;
//        int16_t yaw;
//        int16_t step;
//    } sendData;
//
//    sendData.roll = (int16_t) (attitudeAngle.x * 100);
//    sendData.pitch = (int16_t) (attitudeAngle.y * 100);
//    sendData.yaw = (int16_t) (attitudeAngle.z * 100);
//    sendData.step = (int16_t) (stepSize * 100);
//    sdk_ano_transmit_custom_data(0xF6, 8, (uint8_t *) &sendData);

//    struct {
//        int16_t roll;
//        int16_t pitch;
//        int16_t yaw;
//    } sendData2;
//
//    sendData2.roll = (int16_t) (attitudeAngleTest2.x * 100);
//    sendData2.pitch = (int16_t) (attitudeAngleTest2.y * 100);
//    sendData2.yaw = (int16_t) (attitudeAngleTest2.z * 100);
//    sdk_ano_transmit_custom_data(0xF7, 6, (uint8_t *) &sendData2);
//    printf("%.2f\n", attitudeTest2.a);

//    driver_HMC5883_TransmitRawAndCalibratedData_Custom();
}


