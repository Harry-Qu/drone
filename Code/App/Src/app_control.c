/** 
 *  @brief	    控制相关任务
 *  @details    提供控制相关任务，包含对遥控器数据解算，计算期望角度，PID计算，动能分配，输出PWM
 *  @author     Harry-Qu
 *  @date       2022/10/28
 *  @version    1.0.2
 *  @par        日志
 *                  1.0.0   |   实现基本控制功能
 *                  1.0.1   |   新增限制最大转速功能
 *                  1.0.2   |   修改电机转速测试代码中满油门值为限制最大转速值
*/

/**
 * 四轴的模式介绍
 * FLY_MODE：四轴飞行器的模式，可通过拨杆调整
 * FLY_MODE：四轴飞行器状态，一般通过打杆自动调整
 *
 *
 *          FLY_MODE - FLY_STATE        概述          详细介绍
 *
 *          STOP                        停桨          当四轴飞行器飞行时，左侧拨杆调至上时，可紧急停桨
 *          STOP-INIT                   初始化         上电时默认为此状态，四轴飞行器将进行初始化操作
 *          STOP-MANUAL_CALIBRATION     需人工校准      如磁场干扰较大情况下，需人工进行校准操作
 *          STOP-READY                  准备就绪       四轴飞行器初始化完成，可随时起飞
 *
 *          RUNNING                     桨叶解锁       左侧拨杆调制下时，解锁电机
 *          RUNNING-READY               桨叶解锁但不动 电机解锁，未给杆量
 *          RUNNING-IDLE                桨叶怠速       内八解锁后，桨叶将以低速旋转
 *          RUNNING-HOVER               悬停          四轴飞行器保持悬停
 *          RUNNING-MOVING              运动状态       四轴飞行器按照杆量指令运动
 */

#include "app_control.h"
#include "pid.h"
#include "AHRS.h"
#include "driver_gy86.h"
#include "driver_dbus.h"
#include "driver_motor.h"
#include "driver_rgb.h"
#include "sdk_math.h"


enum FLY_MODE flyMode = STOP;   //飞行模式（锁定or解锁电机）
enum FLY_STATE flyState = INIT; //飞行状态

float motorSpeed[4];    //电机转速

pid_type pidRollInner, pidRollOuter, pidPitchInner, pidPitchOuter, pidYaw, pidHeight;

OS_EVENT *sem_control_task;


static void app_control_cal_motorSpeed_hover(void);

static void app_control_cal_motorSpeed_moving(void);

static void app_control_cal_motorSpeed(void);

/**
 * 对电机转速进行限制，限制范围为[0, LIMIT_MOTOR_SPEED]
 */
void app_control_limit_motorSpeed(void);

/**
 * 电机转速测试函数，仅在测试中使用！！！！！
 * 各电机转速值 = 油门量*0.8
 */
static void app_control_cal_TestSpeed(void);

void app_control_modifyFlyMode(void) {
    switch (rc_data.S1) {
        case DRIVER_DBUS_RC_SW_UP:
            //初始状态或紧急停桨模式
            flyMode = STOP;

            switch (flyState) {
                case IDLE:
                case HOVER:
                case MOVING:
                    //RUNNING -> STOP-READY  从飞行中紧急停桨
                    driver_rgb_setColor(RED, 5);
                    flyState = READY;
                default:
                    break;
            }

            switch (rc_data.S2) {
                case DRIVER_DBUS_RC_SW_DOWN:
                    //S1=UP && S2=DOWN 切换到人工校准模式
                    flyState = MANUAL_CALIBRATION;
                    break;
            }
            break;
        case DRIVER_DBUS_RC_SW_DOWN:
            switch (flyMode) {
                case STOP:
                    if (flyState == READY) {
                        //STOP-READY -> RUNNING-READY
                        //自检完成，左侧拨杆下拨解锁电机
                        flyMode = RUNNING;
                    }
                    break;
                case RUNNING:
                    switch (flyState) {
                        case READY:
                            if ((rc_data.ch0 <= DRIVER_DBUS_RC_CH_MAX + DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) &&
                                (rc_data.ch1 <= DRIVER_DBUS_RC_CH_MIN + DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) &&
                                (rc_data.ch2 >= DRIVER_DBUS_RC_CH_MAX - DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) &&
                                (rc_data.ch3 <= DRIVER_DBUS_RC_CH_MIN + DRIVER_DBUS_RC_CH_TOLERANCE_VALUE)) {
                                // 内八操作
                                // RUNNING-READY -> RUNNING-IDLE
                                driver_rgb_setColor(GREEN, 0);
                                flyState = IDLE;
                            }
                            break;
                        case IDLE:
                            if (rc_data.ch3 >= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) {
                                // 油门向上操作
                                // RUNNING-IDLE -> RUNNING-MOVING
                                driver_rgb_setColor(GREEN, 2);
                                flyState = MOVING;
                            }
                            break;
                        case HOVER:
                            if ((rc_data.ch0 >= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE ||
                                 rc_data.ch0 <= -DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) ||
                                (rc_data.ch1 >= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE ||
                                 rc_data.ch1 <= -DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) ||
                                (rc_data.ch2 >= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE ||
                                 rc_data.ch2 <= -DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) ||
                                (rc_data.ch3 >= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE ||
                                 rc_data.ch3 <= -DRIVER_DBUS_RC_CH_TOLERANCE_VALUE)) {
                                // 打杆操作
                                // RUNNING-HOVER -> RUNNING-MOVING
                                driver_rgb_setColor(GREEN, 2);
                                flyState = MOVING;
                            }
                            break;
                        case MOVING:
                            if ((rc_data.ch0 <= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE &&
                                 rc_data.ch0 > -DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) &&
                                (rc_data.ch1 <= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE &&
                                 rc_data.ch1 > -DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) &&
                                (rc_data.ch2 <= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE &&
                                 rc_data.ch2 > -DRIVER_DBUS_RC_CH_TOLERANCE_VALUE) &&
                                (rc_data.ch3 <= DRIVER_DBUS_RC_CH_TOLERANCE_VALUE &&
                                 rc_data.ch3 > -DRIVER_DBUS_RC_CH_TOLERANCE_VALUE)) {
                                // 回中操作
                                // RUNNING-MOVING -> RUNNING-HOVER
                                driver_rgb_setColor(GREEN, 2);
                                flyState = HOVER;
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;
    }
}

void app_control_toggleFlyState(enum FLY_STATE state) {
    if (flyMode == STOP) {
        flyState = state;
    }
}

void app_control_cal_TestSpeed(void) {
    switch (flyMode) {
        case STOP:
            // 电机锁定模式
            motorSpeed[0] = 0;
            motorSpeed[1] = 0;
            motorSpeed[2] = 0;
            motorSpeed[3] = 0;
            break;
        case RUNNING:
            // 电机解锁模式
            switch (flyState) {
                case IDLE:
                    // 怠速状态
                case HOVER:
                    // 悬停状态
                case MOVING:
                    // 运动状态
                {
                    if (rc_data.ch3 >= 0) {
                        motorSpeed[0] =
                        motorSpeed[1] =
                        motorSpeed[2] =
                        motorSpeed[3] =
                                ((float) rc_data.ch3 / DRIVER_DBUS_RC_CH_MAX) * LIMIT_MOTOR_SPEED;
                    }
                    break;
                }
                default:
                    // 出现异常状态，锁定电机
                    motorSpeed[0] = 0;
                    motorSpeed[1] = 0;
                    motorSpeed[2] = 0;
                    motorSpeed[3] = 0;
                    break;
            }
            break;
        default:
            break;
    }
}

void app_control_cal_motorSpeed_hover(void) {
    app_control_cal_motorSpeed_moving();
    return;

    //TODO 完成悬停状态代码
    float targetRoll = 0, targetPitch = 0;
    float targetYawSpeed = 0, targetVerticalSpeed = 0;
}

void app_control_cal_motorSpeed_moving(void) {
    float targetRoll = 0, targetPitch = 0;
    float targetYawSpeed = 0, targetVerticalSpeed = 0;

    float wx = 0, wy = 0, wz = 0, wh = 0;


//    绕着坐标轴逆时针旋转为角度的负方向


//TODO 待实机测试
    targetRoll = -MAX_ROLL_ANGLE * ((float) rc_data.ch0 / DRIVER_DBUS_RC_CH_MAX);
    targetPitch = -MAX_PITCH_ANGLE * ((float) rc_data.ch1 / DRIVER_DBUS_RC_CH_MAX);
    targetYawSpeed = MAX_YAW_SPEED * ((float) rc_data.ch2 / DRIVER_DBUS_RC_CH_MAX);
    targetVerticalSpeed = MAX_VERTICAL_SPEED * ((float) rc_data.ch3 / DRIVER_DBUS_RC_CH_MAX);

//    pid_setCurrent(pidRollOuter, attitudeAngle.x);
//    pid_setTarget(pidRollOuter, targetRoll);
//    pid_setCurrent(pidRollInner, imu_calibrated_data.gyro.x);
//    pid_setTarget(pidRollInner, PID_calculate(&pidRollOuter));
//    wx = PID_calculate(&pidRollInner);

    pid_setCurrent(pidPitchOuter, attitudeAngle.y);
    pid_setTarget(pidPitchOuter, targetPitch);
    pid_setCurrent(pidPitchInner, imu_calibrated_data.gyro.y);
    pid_setTarget(pidPitchInner, 0);
    wy = PID_calculate(&pidPitchInner);
//    printf("%.2f\n", wy);

//    pid_setCurrent(pidYaw, imu_calibrated_data.gyro.z);
//    pid_setTarget(pidYaw, targetYawSpeed);
//    wz = PID_calculate(&pidYaw);


    //TODO 高度环
//    pid_setTarget(pidHeight, targetVerticalSpeed);
//    wh = targetVerticalSpeed * 5;

//    motorSpeed[0] = motorSpeed[1] = motorSpeed[2] = motorSpeed[3] = 0;

    motorSpeed[MOTOR_FL] = HOVER_MOTOR_SPEED - wx - wy - wz + wh;
    motorSpeed[MOTOR_FR] = HOVER_MOTOR_SPEED + wx - wy + wz + wh;
    motorSpeed[MOTOR_BL] = HOVER_MOTOR_SPEED - wx + wy + wz + wh;
    motorSpeed[MOTOR_BR] = HOVER_MOTOR_SPEED + wx + wy - wz + wh;

}

void app_control_cal_motorSpeed(void) {
    switch (flyMode) {
        case STOP:
            // 电机锁定模式
            motorSpeed[0] = 0;
            motorSpeed[1] = 0;
            motorSpeed[2] = 0;
            motorSpeed[3] = 0;
            break;
        case RUNNING:
            // 电机解锁模式
            switch (flyState) {
                case IDLE:
                    // 怠速状态
                    motorSpeed[0] = IDLE_MOTOR_SPEED;
                    motorSpeed[1] = IDLE_MOTOR_SPEED;
                    motorSpeed[2] = IDLE_MOTOR_SPEED;
                    motorSpeed[3] = IDLE_MOTOR_SPEED;
                    break;
                case HOVER:
                    // 悬停状态
                    app_control_cal_motorSpeed_hover();
                    break;
                case MOVING:
                    // 运动状态
                    app_control_cal_motorSpeed_moving();
                    break;
                default:
                    // 出现异常状态，锁定电机
                    motorSpeed[0] = 0;
                    motorSpeed[1] = 0;
                    motorSpeed[2] = 0;
                    motorSpeed[3] = 0;
                    break;
            }
            break;
        default:
            break;
    }
    app_control_limit_motorSpeed();
}

void app_control_limit_motorSpeed(void) {
    LIMIT(motorSpeed[0], 0, LIMIT_MOTOR_SPEED);
    LIMIT(motorSpeed[1], 0, LIMIT_MOTOR_SPEED);
    LIMIT(motorSpeed[2], 0, LIMIT_MOTOR_SPEED);
    LIMIT(motorSpeed[3], 0, LIMIT_MOTOR_SPEED);
}

void app_control_transmitMotorSpeed(void) {
    struct {
        uint16_t pwm[4];
    } sendData;

    sendData.pwm[0] = (uint16_t) (motorSpeed[0] / MAX_MOTOR_SPEED * 10000);
    sendData.pwm[1] = (uint16_t) (motorSpeed[1] / MAX_MOTOR_SPEED * 10000);
    sendData.pwm[2] = (uint16_t) (motorSpeed[2] / MAX_MOTOR_SPEED * 10000);
    sendData.pwm[3] = (uint16_t) (motorSpeed[3] / MAX_MOTOR_SPEED * 10000);
//    printf("%d %d %d %d\n", sendData.pwm[0], sendData.pwm[1], sendData.pwm[2], sendData.pwm[3]);
    sdk_ano_transmit_pwm(&sendData);
}

void app_control(void) {
    driver_dbus_Analysis();
//    printf("%d %d %d %d\n", rc_data.ch0, rc_data.ch1, rc_data.ch2, rc_data.ch3);
    AHRS_ConvertQuatToDegree(&attitude, &attitudeAngle);

    app_control_modifyFlyMode();
//    printf("%d %d\n", flyMode, flyState);

    app_control_cal_motorSpeed();
//    app_control_cal_TestSpeed();

    driver_motor_set_speed(motorSpeed);
}

void app_control_tmr_callback(void *ptmr, void *parg) {
    (void) ptmr;
    (void) parg;
    OSSemPost(sem_control_task);
}

void app_control_task(void *args) {
    (void) args;
    uint8_t err;
    OS_TMR *tmr_control_task;
    sem_control_task = OSSemCreate(0);
    tmr_control_task = OSTmrCreate(0, OS_TICKS(20), OS_TMR_OPT_PERIODIC, app_control_tmr_callback, (void *) 0,
                                   (INT8U *) "control_TASK_Tmr", &err);
    OSTmrStart(tmr_control_task, &err);

    while (1) {
        OSSemPend(sem_control_task, 0, &err);
        OS_TRACE_MARKER_START(1);
        app_control();
//        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        OS_TRACE_MARKER_STOP(1);
    }
}

void app_control_init(void) {
    pidRollInner = PID_Init(PID_ROLL_INNER_KP, PID_ROLL_INNER_KI, PID_ROLL_INNER_KD, PID_ROLL_INNER_INTEGRAL_LIMIT);
    pidRollOuter = PID_Init(PID_ROLL_OUTER_KP, PID_ROLL_OUTER_KI, PID_ROLL_OUTER_KD, PID_ROLL_OUTER_INTEGRAL_LIMIT);
    pidPitchInner = PID_Init(PID_PITCH_INNER_KP, PID_PITCH_INNER_KI, PID_PITCH_INNER_KD,
                             PID_PITCH_INNER_INTEGRAL_LIMIT);
    pidPitchOuter = PID_Init(PID_PITCH_OUTER_KP, PID_PITCH_OUTER_KI, PID_PITCH_OUTER_KD,
                             PID_PITCH_OUTER_INTEGRAL_LIMIT);
    pidYaw = PID_Init(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_YAW_INTEGRAL_LIMIT);
    pidHeight = PID_Init(PID_HEIGHT_KP, PID_HEIGHT_KI, PID_HEIGHT_KD, PID_HEIGHT_INTEGRAL_LIMIT);

    driver_motor_Init();
    driver_dbus_Init();
}