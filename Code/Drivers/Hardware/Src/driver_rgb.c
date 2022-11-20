/**
 *  @brief      RGB灯驱动
 *  @details    控制机头机尾的RGB指示灯状态
 *  @author     Harry-Qu
 *  @date       2021.11.25
 *  @version    1.1
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
				1.0		|		构建文件，实现基本功能
				1.1     |       修改为仅支持单RGB灯，添加定时器功能，支持控制每秒闪烁次数
*/

#include "driver_rgb.h"

#include "app_cfg.h"


#if DEVICE_RGB_EN > 0

enum RGB_COLOR rgbColor;
uint8_t rgbLightTimes;  //每秒闪灯次数, 0代表常量

void driver_rgb_Off() {
    HAL_GPIO_WritePin(R_GPIO_Port, R_PIN, 1);
    HAL_GPIO_WritePin(G_GPIO_Port, G_PIN, 1);
    HAL_GPIO_WritePin(B_GPIO_Port, B_PIN, 1);
}

void driver_rgb_On(enum RGB_COLOR color) {
    HAL_GPIO_WritePin(R_GPIO_Port, R_PIN, ((color >> 2) & 0x01));
    HAL_GPIO_WritePin(G_GPIO_Port, G_PIN, ((color >> 1) & 0x01));
    HAL_GPIO_WritePin(B_GPIO_Port, B_PIN, ((color >> 0) & 0x01));
}

void driver_rgb_setColor(enum RGB_COLOR color, uint8_t lightTimes) {
    rgbColor = color;
    rgbLightTimes = lightTimes;
}

void driver_rgb_toggle_it_callback(TIM_HandleTypeDef *htim) {

    static uint8_t cnt = 0;
    if ((++cnt) > 10) {
        cnt = 1;
    }

    if (rgbLightTimes > 0) {
        if ((cnt % 2) && (cnt <= 2 * rgbLightTimes)) {
            driver_rgb_On(rgbColor);
        } else {
            driver_rgb_Off();
        }
    } else {
        driver_rgb_On(rgbColor);
    }
}

void driver_rgb_init(void) {
    HAL_TIM_RegisterCallback(&htim10, HAL_TIM_PERIOD_ELAPSED_CB_ID, driver_rgb_toggle_it_callback);
    HAL_TIM_Base_Start_IT(&htim10);
}

#endif
