/** 
 *  @brief	    控制相关任务
 *  @details    提供控制相关任务，包含对遥控器数据解算，计算期望角度，PID计算，动能分配，输出PWM
 *  @author     Harry-Qu
 *  @date       2022/10/28
 *  @version    1.0
 *  @par        日志
*/

#include "main.h"
#include "pid.h"


#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#define MOTOR_FL 2
#define MOTOR_FR 1
#define MOTOR_BR 0
#define MOTOR_BL 3

#define HOVER_MOTOR_SPEED 2800.0f //悬停时电机转速，54
#define IDLE_MOTOR_SPEED 800.0f  //怠速时电机速度

#define MAX_ROLL_ANGLE 25.0f    //最大横滚角度（角度制） degree
#define MAX_PITCH_ANGLE 25.0f   //最大俯仰角度（角度制） degree

#define MAX_VERTICAL_SPEED 3.0f      //最大上升速度（m/s）
#define MAX_YAW_SPEED 30.0f     //最大旋转角速度（degree/s

//  PID参数

#define PID_ROLL_OUTER_KP 0.1f
#define PID_ROLL_OUTER_KI 0.0f
#define PID_ROLL_OUTER_KD 0.0f
#define PID_ROLL_OUTER_INTEGRAL_LIMIT 0.0f

#define PID_ROLL_INNER_KP 0.1f
#define PID_ROLL_INNER_KI 0.0f
#define PID_ROLL_INNER_KD 0.0f
#define PID_ROLL_INNER_INTEGRAL_LIMIT 0.0f

#define PID_PITCH_OUTER_KP 0.0f
#define PID_PITCH_OUTER_KI 0.0f
#define PID_PITCH_OUTER_KD 0.0f
#define PID_PITCH_OUTER_INTEGRAL_LIMIT 0.0f

#define PID_PITCH_INNER_KP 0.1f
#define PID_PITCH_INNER_KI 0.0f
#define PID_PITCH_INNER_KD 0.0f
#define PID_PITCH_INNER_INTEGRAL_LIMIT 0.0f

#define PID_YAW_KP 0.5f
#define PID_YAW_KI 0.0f
#define PID_YAW_KD 0.0f
#define PID_YAW_INTEGRAL_LIMIT 0.0f

#define PID_HEIGHT_KP 0.5f
#define PID_HEIGHT_KI 0.0f
#define PID_HEIGHT_KD 0.0f
#define PID_HEIGHT_INTEGRAL_LIMIT 0.0f

enum FLY_MODE{
    STOP = 0,   //停桨模式
    RUNNING,    //开桨状态

};

enum FLY_STATE{
    INIT = 0,   // 初始化中
    MANUAL_CALIBRATION,  // 人工校准状态
    READY,      // 就绪状态
    SENSOR_ERROR,

    IDLE,       // 怠速状态
    HOVER,      // 悬停状态
    MOVING      // 运动状态
};

extern pid_type pidRollInner, pidRollOuter, pidPitchInner, pidPitchOuter, pidYaw, pidHeight;

extern float motorSpeed[4];

/**
 * 发送四个电机转速给匿名上位机
 */
void app_control_transmitMotorSpeed(void);

/**
 * 修改飞行模式
 * @details 根据遥控器数据切换状态
 */
void app_control_modifyFlyMode(void);

/**
 * 切换运行状态（仅允许在STOP模式操作）
 * 用于自检阶段切换状态
 * @param state 要切换到的状态
 */
void app_control_toggleFlyState(enum FLY_STATE state);

/**
 * 控制任务
 * @param args
 */
void app_control_task(void *args);

/**
 * 控制初始化
 */
void app_control_init(void);

#endif //APP_CONTROL_H
