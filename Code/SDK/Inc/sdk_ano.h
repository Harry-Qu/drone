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

#ifndef GY86_SDK_ANO_H
#define GY86_SDK_ANO_H

#define SDK_ANO

#include "main.h"

#define ANO_UART huart6

#define ANO_DATA_HEAD 0xAA
#define ANO_DATA_ADDRESS 0xFF

enum ANO_DATA_FUNCTION_ID {
    INERTIAL_SENSOR_DATA = 0X01,
    COMPASS_AIR_PRESSURE_TEMPERATURE_SENSOR_DATA = 0x02,
    FLIGHT_CONTROL_ATTITUDE_EULER_ANGLE = 0x03,
    FLIGHT_CONTROL_ATTITUDE_QUATERNION = 0x04,
    HEIGHT_DATA = 0x05,
    OPERATING_MODE = 0x06,
    SPEED_DATA = 0x07,
    POSITION_OFFSET_DATA = 0x08,
    WIND_SPEED_ESTIMATION = 0x09,
    TARGET_ATTITUDE_DATA = 0x0A,
    TARGET_SPEED_DATA = 0x0B,

    PWM_CONTROL_QUANTITY = 0x20

};

struct ANO_DATA_T {
    uint8_t head;
    uint8_t address;
    uint8_t functionId;
    uint8_t length;
    uint8_t *data;
    uint8_t sumCheck;
    uint8_t addCheck;
};

/**
 * 初始化匿名上位机通信帧的帧头与目标地址
 * @param anoData 匿名上位机通信帧
 */
void sdk_ano_init_anoData(struct ANO_DATA_T *anoData);

/**
 * 发送惯性传感器数据给匿名上位机
 * @param inertialSensorData 惯性传感器数据
 * 数据格式：
 *          int16   ACC_X   加速度
 *          int16   ACC_Y   加速度
 *          int16   ACC_Z   加速度
 *          int16   GYR_X   陀螺仪
 *          int16   GYR_Y   陀螺仪
 *          int16   GYR_Z   陀螺仪
 *          uint8   SHOCK_STA   震动状态
 */
void sdk_ano_transmit_inertial_sensor_data(const void *inertialSensorData);

/**
 * 发送罗盘、气压、温度传感器数据给匿名上位机
 * @param APTSensorData 罗盘、气压、温度传感器数据
 * 数据格式：
 *          int16   MAG_X   磁罗盘数据
 *          int16   MAG_Y   磁罗盘数据
 *          int16   MAG_Z   磁罗盘数据
 *          int32   ALT_BAR 气压计高度，单位cm
 *          int16   TMP     传感器温度，放大10倍传输，0.1摄氏度
 *          uint8   BAR_STA 气压状态
 *          uint8   MAG_STA 罗盘状态
 */
void sdk_ano_transmit_compass_air_pressure_temperature_sensor_data(void *APTSensorData);

/**
 * 发送飞控姿态（欧拉角格式）数据给匿名上位机
 * @param attitudeEulerAngle 飞控姿态数据（欧拉角格式）
 * 数据格式：
 *          int16   ROL*100 横滚角
 *          int16   PIT*100 俯仰角
 *          int16   YAW*100 航向角
 *          uint8   FUSION_STA  融合状态
 */
void sdk_ano_transmit_flight_control_attitude_euler_angle(void *attitudeEulerAngle);

/**
 * 发送飞控姿态（四元数格式）数据给匿名上位机
 * @param controlAttitudeQuaternion 飞控姿态数据（四元数格式）
 * 数据格式：
 *          int16   v0*10000    四元数，传输时放大10000倍
 *          int16   v1*10000    四元数，传输时放大10000倍
 *          int16   v2*10000    四元数，传输时放大10000倍
 *          int16   v3*10000    四元数，传输时放大10000倍
 *          uint8   FUSION_STA  融合状态
 */
void sdk_ano_transmit_flight_control_attitude_quaternion(void *controlAttitudeQuaternion);

/**
 * 向匿名上位机发送高度数据
 * @param heightData 高度数据
 * 数据格式:
 *          int32   ALT_FU      融合后对地高度，单位厘米。
 *          int32   ALT_ADD     附加高度传感高度数据，如超声波、激光测距，单位厘米。
 *          uint8   ALT_STA     测距状态
 */
void sdk_ano_transmit_height(void *heightData);

/**
 * 发送PWM控制量数据给匿名上位机
 * @param pwmData PWM数据
 * 数据格式
 *          uint16  PWM1    范围0-10000，单位0.01%油门
 *          uint16  PWM2    范围0-10000，单位0.01%油门
 *          uint16  PWM3    范围0-10000，单位0.01%油门
 *          uint16  PWM4    范围0-10000，单位0.01%油门
 */
void sdk_ano_transmit_pwm(void *pwmData);

/**
 * 发送字符串日志信息给匿名上位机
 * @param color 消息颜色
 * @param data 消息内容（字符串形式，以'\0'结尾）
 */
void sdk_ano_transmit_log_string(uint8_t color, char *data);

/**
 * 发送字符串和数字日志信息给匿名上位机
 * @param value 数值
 * @param data 字符串数字
 */
void sdk_ano_transmit_log_stringAndNumber(int32_t value, char *data);

/**
 * 向匿名上位机发送灵活格式数据
 * @param functionId 功能码,0xF1-0xFA
 * @param dataLength 数据长度,1-40
 * @param data 数据内容
 */
void sdk_ano_transmit_custom_data(uint8_t functionId, uint8_t dataLength, uint8_t *data);

/**
 * 向匿名上位机发送0xFB折线图刷新帧
 */
void sdk_ano_transmit_refresh_frame();

#endif //GY86_SDK_ANO_H
