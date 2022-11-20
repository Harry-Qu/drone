/** 
 *  @brief	    定时器基础操作
 *  @details    支持设定定时器的分频、重载系数并开启中断；支持开启定时器
 *  @author     Harry-Qu
 *  @date       2022/7/13
 *  @version    1.0
 *  @par        日志
*/

#ifndef GY86_BSP_TIM_H
#define GY86_BSP_TIM_H

#include "main.h"

/**
 * 初始化定时器
 * @param tim 定时器
 * @param psc 分频系数，传入前需确保不会超过限定值
 * @param arr 重装载值，传入前需确保不会超过限定值
 */
void bsp_tim_init_period(TIM_TypeDef *tim, uint32_t psc, uint32_t arr);

/**
 * 开启定时器计数
 * @param tim 定时器
 */
void bsp_tim_start(TIM_TypeDef *tim);

#endif //GY86_BSP_TIM_H
