/** 
 *  @brief	    定时器基础操作
 *  @details    支持设定定时器的分频、重载系数并开启中断；支持开启定时器
 *  @author     Harry-Qu
 *  @date       2022/7/13
 *  @version    1.0
 *  @par        日志
*/

#include "bsp_tim.h"


void bsp_tim_init_period(TIM_TypeDef *tim, uint32_t psc, uint32_t arr) {

    IRQn_Type TIMx_IRQn = TIM2_IRQn;

    if (tim == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        TIMx_IRQn = TIM2_IRQn;
    } else if (tim == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        TIMx_IRQn = TIM3_IRQn;
    } else if (tim == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        TIMx_IRQn = TIM4_IRQn;
    } else if (tim == TIM5) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        TIMx_IRQn = TIM5_IRQn;
    }

    tim->CR2 |= TIM_CR2_MMS_1;  //选择触发更新事件模式

    tim->DIER = 0;
    tim->DIER |= TIM_DIER_UIE;  //使能更新事件中断

    tim->CNT = 0;
    tim->PSC = psc;
    tim->ARR = arr;


    NVIC_SetPriority(TIMx_IRQn, 0);
    NVIC_EnableIRQ(TIMx_IRQn);

    tim->EGR = TIM_EGR_UG;
}

void bsp_tim_start(TIM_TypeDef *tim) {
    tim->CR1 |= TIM_CR1_CEN;    //开始计数
}
