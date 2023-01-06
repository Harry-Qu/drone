/**
 *  @file       sdk_ano.c
 *  @brief	    匿名上位机数据传输
 *  @details    匿名上位机数据传输中间件，基于V7.12协议
 *  @author     Harry-Qu
 *  @date       2022/10/4
 *  @version    1.1.1
 *  @par        日志
 *              1.0.0   |       实现匿名上位机发送数据基本功能
 *                              支持发送传感器基本数据，四轴姿态，用户自定义数据功能。
 *              1.0.1   |       添加高度、PWM、log日志等格式数据帧
 *              1.1.0   |       降低与sdk_usart内函数的耦合
 *              1.1.1   |       新增0xFB刷新帧功能
*/

#include <memory.h>
#include "sdk_ano.h"
#include "sdk_usart.h"  //注释此头文件后不提供DMA发送功能

void sdk_ano_init_anoData(struct ANO_DATA_T *anoData) {
    anoData->head = ANO_DATA_HEAD;
    anoData->address = ANO_DATA_ADDRESS;
}


/**
 * 计算通信帧的校验和
 * @param anoData 匿名上位机通信帧
 */
static void sdk_ano_cal_check(struct ANO_DATA_T *anoData) {
    uint8_t sumCheck = 0;
    uint8_t addCheck = 0;
    sumCheck += anoData->head;
    addCheck += sumCheck;
    sumCheck += anoData->address;
    addCheck += sumCheck;
    sumCheck += anoData->functionId;
    addCheck += sumCheck;
    sumCheck += anoData->length;
    addCheck += sumCheck;
    for (uint8_t i = 0; i < anoData->length; ++i) {
        sumCheck += anoData->data[i];
        addCheck += sumCheck;
    }

    anoData->sumCheck = sumCheck;
    anoData->addCheck = addCheck;
}

/**
 * 发送通信帧给上位机
 * @param anoData 匿名上位机通信帧
 */
static void sdk_ano_transmit_data(struct ANO_DATA_T *anoData) {
    uint8_t err;
    uint8_t sendData[100];
    memcpy(sendData, anoData, 4);
    memcpy(sendData + 4, anoData->data, anoData->length);
    memcpy(sendData + 4 + anoData->length, &(anoData->sumCheck), 2);
#ifdef SDK_UART
    sdk_uart_transmit_dma(&ANO_UART, (uint8_t *) sendData, anoData->length + 6, 10, &err);
#else

#ifdef OS_uCOS_II_H
    OSSchedLock();
    HAL_UART_Transmit(&ANO_UART, (uint8_t *) sendData, anoData->length + 6, 10);
    OSSchedUnlock();
#else
    HAL_UART_Transmit(&ANO_UART, (uint8_t *) sendData, anoData->length + 6, 10);
#endif

#endif

}

/**
 * 将数据打包成数据帧
 * @param anoData 数据帧（已分配空间）
 * @param functionId 功能码
 * @param dataLength 数据长度
 * @param data 数据
 */
static void sdk_ano_combine_data(struct ANO_DATA_T *anoData, uint8_t functionId, uint8_t dataLength, uint8_t *data) {
    sdk_ano_init_anoData(anoData);
    anoData->functionId = functionId;
    anoData->length = dataLength;
    anoData->data = data;
    sdk_ano_cal_check(anoData);
}

void sdk_ano_transmit_inertial_sensor_data(const void *inertialSensorData) {
    struct ANO_DATA_T anoData;
    sdk_ano_combine_data(&anoData, INERTIAL_SENSOR_DATA, 13, (uint8_t *) inertialSensorData);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_compass_air_pressure_temperature_sensor_data(void *APTSensorData) {
    struct ANO_DATA_T anoData;
    sdk_ano_combine_data(&anoData, COMPASS_AIR_PRESSURE_TEMPERATURE_SENSOR_DATA, 14, (uint8_t *) APTSensorData);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_flight_control_attitude_euler_angle(void *attitudeEulerAngle) {
    struct ANO_DATA_T anoData;
    sdk_ano_combine_data(&anoData, FLIGHT_CONTROL_ATTITUDE_EULER_ANGLE, 7, (uint8_t *) attitudeEulerAngle);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_flight_control_attitude_quaternion(void *controlAttitudeQuaternion) {
    struct ANO_DATA_T anoData;
    sdk_ano_combine_data(&anoData, FLIGHT_CONTROL_ATTITUDE_QUATERNION, 9, (uint8_t *) controlAttitudeQuaternion);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_height(void *heightData) {
    struct ANO_DATA_T anoData;
    sdk_ano_combine_data(&anoData, HEIGHT_DATA, 9, (uint8_t *) heightData);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_pwm(void *pwmData) {
    struct ANO_DATA_T anoData;
    sdk_ano_combine_data(&anoData, PWM_CONTROL_QUANTITY, 8, (uint8_t *) pwmData);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_log_string(uint8_t color, char *data) {
    struct ANO_DATA_T anoData;
    uint8_t logData[100];
    logData[0] = color;
    strcpy((char *) logData + 1, data);

    sdk_ano_combine_data(&anoData, 0xA0, strlen(data) + 1, (uint8_t *) logData);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_log_stringAndNumber(int32_t value, char *data) {
    struct ANO_DATA_T anoData;
    uint8_t logData[100];
    memcpy(logData, &value, 4);
    strcpy((char *) logData + 4, data);

    sdk_ano_combine_data(&anoData, 0xA1, strlen(data), (uint8_t *) logData);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_custom_data(uint8_t functionId, uint8_t dataLength, uint8_t *data) {
    struct ANO_DATA_T anoData;

    if (functionId < 0xF1 || functionId > 0xFA) {
        return;
    }
    sdk_ano_combine_data(&anoData, functionId, dataLength, (uint8_t *) data);
    sdk_ano_transmit_data(&anoData);
}

void sdk_ano_transmit_refresh_frame() {
    struct ANO_DATA_T anoData;
    sdk_ano_combine_data(&anoData, 0xFB, 0, NULL);
    sdk_ano_transmit_data(&anoData);
}


