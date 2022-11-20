/**
 *  @brief      调试任务-OLED显示部分
 *  @details    提供OLED调试功能函数，如显示指定数据等
 *  @author     Harry-Qu
 *  @date       2021.10
 *  @version    1.3.1
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
				1.0		|		实现OLED显示gy86原始数据、遥控器数据，页面切换功能
				1.1     |       更改为定时器触发方式，增加marker跟踪功能
				1.1.1   |       修改定时器回调函数的参数类型和回调函数名
				1.2     |       新增超声波数值显示功能
				1.3     |       新增浮点数显示测试功能，修改字体大小变量名为fontSize
				1.3.1   |       修改IMU，磁力计数据的相关格式。
*/

#include <string.h>
#include <stdio.h>
#include "app_debug_oled.h"
#include "app_cfg.h"

#if DEBUG_OLED_EN > 0

#define PRESS_BUTTON_SWITCH_MIN_TICK_DELTA  50


#include "driver_gy86.h"
#include "driver_dbus.h"
//#include "app_motor.h"

enum OLED_FUNCTION oled_function = RC, last_oled_function;
static uint32_t lastSwitchOledTick;


void app_oled_Show_Data(void) {
    if (last_oled_function != oled_function) {
        app_oled_Show_Template();
    }
    switch (oled_function) {
        case (GY86): {
            driver_oled_Show_Float(18, 1, imu_calibrated_data.acc.x, 2, 8);
            driver_oled_Show_Float(18, 2, imu_calibrated_data.acc.y, 2, 8);
            driver_oled_Show_Float(18, 3, imu_calibrated_data.acc.z, 2, 8);
            driver_oled_Show_Float(84, 1, imu_calibrated_data.gyro.x, 2, 8);
            driver_oled_Show_Float(84, 2, imu_calibrated_data.gyro.y, 2, 8);
            driver_oled_Show_Float(84, 3, imu_calibrated_data.gyro.z, 2, 8);
            driver_oled_Show_Float(12, 4, mag_calibrated_data.x, 2, 8);
            driver_oled_Show_Float(12, 5, mag_calibrated_data.y, 2, 8);
            driver_oled_Show_Float(12, 6, mag_calibrated_data.z, 2, 8);
            driver_oled_Show_Num(84, 4, apg_data.pressure, 8);

            break;
        }
        case (RC): {
            driver_oled_Clear_Part(24, 48, 2, 3);


            driver_oled_Show_Num(24, 2, rc_data.ch0, 8);
            driver_oled_Show_Num(24, 3, rc_data.ch2, 8);


            driver_oled_Clear_Part(88, 112, 2, 3);
            driver_oled_Show_Num(84, 2, rc_data.ch1, 8);
            driver_oled_Show_Num(84, 3, rc_data.ch3, 8);

            driver_oled_Show_Num(24, 5, rc_data.S1, 8);
            driver_oled_Show_Num(84, 5, rc_data.S2, 8);

            break;
        }
#if DEVICE_ULTRASOUND_EN > 0
            case (ULTRASOUND): {
                driver_oled_Show_Float(24, 1, ultrasoundDistance, 2, 8);
                break;
            }
#endif
        case MOTOR: {
//            driver_oled_Show_Float(24, 2, realSpeed[0], 2, 8);
//            driver_oled_Show_Float(24, 3, realSpeed[1], 2, 8);
//            driver_oled_Show_Float(24, 4, realSpeed[2], 2, 8);
//            driver_oled_Show_Float(24, 5, realSpeed[3], 2, 8);
            break;
        }

    }

}

void app_oled_Show_Template(void) {
    driver_oled_Clear();

    switch (oled_function) {

        case (GY86): {
            driver_oled_Show_String(0, 0, "GY-86", 8);

            driver_oled_Show_String(0, 1, "ax:", 8);
            driver_oled_Show_String(0, 2, "ay:", 8);
            driver_oled_Show_String(0, 3, "az:", 8);

            driver_oled_Show_String(66, 1, "gr:", 8);
            driver_oled_Show_String(66, 2, "gp:", 8);
            driver_oled_Show_String(66, 3, "gy:", 8);

            driver_oled_Show_String(0, 4, "x:", 8);
            driver_oled_Show_String(0, 5, "y:", 8);
            driver_oled_Show_String(0, 6, "z:", 8);

            driver_oled_Show_String(60, 4, "pre:", 8);

            break;
        }

        case (RC): {
            driver_oled_Show_String(0, 0, "Remote Control", 8);

            driver_oled_Show_String(0, 2, "ch0:", 8);
            driver_oled_Show_String(64, 2, "ch1:", 8);
            driver_oled_Show_String(0, 3, "ch2:", 8);
            driver_oled_Show_String(64, 3, "ch3:", 8);

            driver_oled_Show_String(0, 5, "S1:", 8);
            driver_oled_Show_String(64, 5, "S2:", 8);

            break;
        }
#if DEVICE_ULTRASOUND_EN > 0
            case (ULTRASOUND): {
                driver_oled_Show_String(0, 0, "UltraSound", 8);
                driver_oled_Show_String(0, 1, "dis:", 8);
                break;
            }
#endif
        case (MOTOR): {
            driver_oled_Show_String(0, 0, "Motor", 8);
            driver_oled_Show_String(0, 2, "M0:", 8);
            driver_oled_Show_String(0, 3, "M1:", 8);
            driver_oled_Show_String(0, 4, "M2:", 8);
            driver_oled_Show_String(0, 5, "M3:", 8);
            break;
        }
        default:
            break;
    }
    last_oled_function = oled_function;
}

void app_oled_Init(void) {
    driver_oled_Init();
    app_oled_Show_Template();
}

static uint8_t check_oled_function(enum OLED_FUNCTION f) {
    return 1;
}

void app_oled_Switch_Function(void) {
    do {
        oled_function = (oled_function + 1) % MAX_OLED_FUNCTION_NUM;
    } while (!check_oled_function(oled_function));
}


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    if (GPIO_Pin == B1_Pin && !(B1_GPIO_Port->IDR & B1_Pin)) {
//        if (HAL_GetTick() - lastSwitchOledTick > PRESS_BUTTON_SWITCH_MIN_TICK_DELTA) {
//            lastSwitchOledTick = HAL_GetTick();
//            app_oled_Switch_Function();
//        }
//    }
//}

#endif
