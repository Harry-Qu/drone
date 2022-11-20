#include <memory.h>
#include "ucos_ii.h"
#include "stm32f4xx.h"
#include "bsp_tim.h"

#define OS_TIM TIM5

void OSInitHookBegin(void) {}

void OSInitHookEnd(void) {
    uint32_t psc = SystemCoreClock / 1000 / OS_TICKS_PER_SEC;   //分频系数
    uint32_t arr = SystemCoreClock / psc / OS_TICKS_PER_SEC;    //计数值
    bsp_tim_init_period(OS_TIM, psc - 1, arr - 1);
    bsp_tim_start(OS_TIM);
}

void OSTaskCreateHook(OS_TCB *ptcb) {}

void OSTaskDelHook(OS_TCB *ptcb) {}


void OSTaskIdleHook(void) {
//    if (OSIdleCtr>=10320295){
//        OSTaskIdleHook2();
//        SEGGER_SYSVIEW_Print("test");
//    }
}

void OSTaskReturnHook(OS_TCB *ptcb) {}

void OSTaskStatHook(void) {}


void OSTCBInitHook(OS_TCB *ptcb) {}


void TIM5_IRQHandler(void) {
    if ((OS_TIM->SR & TIM_SR_UIF) == TIM_SR_UIF) {
//        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        OSIntEnter();
        OSTimeTick();
#if OS_TMR_EN > 0u
        OSTmrSignal();
#endif
        OSIntExit();
        OS_TIM->SR = ~TIM_SR_UIF;
//        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

}

/**
 * 堆栈初始化函数
 * @details 本函数将根据参数opt的值决定入栈方式。若opt的bit2为1，则使用浮点运算单元的方式入栈，否则使用不带浮点运算单元的方式入栈。
 *          对于EXC_RETURN的值，如果使用浮点运算单元则为0xFFFFFFED，否则为0xFFFFFFFD（具体含义参照异常返回值，PM0214手册44页）
 *          堆栈顺序如下
 *              floating-point                  without floating-point
 *                  blank（留空）       top             xPSR
 *                  FPSCR               |               PC(task)
 *                  S15                 |               LR(task)
 *                   |                  |               R12
 *                  S0                  |               R3
 *                  xPSR                |               R2
 *                  PC(task)            |               R1
 *                  LR(task)            |               R0
 *                  R12                 |               LR(EXC_RETURN)
 *                  R3                  |               R11
 *                  R2                  |                |
 *                  R1                  |               R4
 *                  R0                  |
 *                  S31                 |
 *                   |                  |
 *                  S16                 |
 *                  LR(EXC_RETURN)      |
 *                  R11                 |
 *                   |                  |
 *                  R4                bottom
 *
 * @param task 任务函数地址
 * @param p_arg 输入参数
 * @param ptos 栈顶地址
 * @param opt 属性参数（同os_task.c中OSTaskCreateExt函数opt参数类型的定义）
 *                          OS_TASK_OPT_STK_CHK      Stack checking to be allowed for the task
 *                          OS_TASK_OPT_STK_CLR      Clear the stack when the task is created
 *                          OS_TASK_OPT_SAVE_FP      If the CPU has floating-point registers, save them
 *                                                   during a context switch.
 * @return
 */
OS_STK *OSTaskStkInit(void (*task)(void *p_arg), void *p_arg, OS_STK *ptos, INT16U opt) {
    OS_STK *stk;
    stk = ptos;

    if ((opt & OS_TASK_OPT_SAVE_FP) > 0u) {
        //填充S0-S15,FPSCR,保留位
        memset(stk - 17, 0, 18 * sizeof(OS_STK));
        stk -= 18;
    }

    *(stk) = 0x01000000;    //xPSR
    *(--stk) = (OS_STK) task;    //PC
    *(--stk) = 0xFFFFFFFF;    //任务保存的lr
    *(--stk) = 0;    //R12
    *(--stk) = 0;    //R3
    *(--stk) = 0;    //R2
    *(--stk) = 0;    //R1
    *(--stk) = (OS_STK) p_arg;    //R0

    if ((opt & OS_TASK_OPT_SAVE_FP) > 0u) {
        //填充S16-S31
        stk -= 16;
        memset(stk, 0, 16 * sizeof(OS_STK));
    }

    if ((opt & OS_TASK_OPT_SAVE_FP) > 0u) {
        *(--stk) = 0xFFFFFFED;    //中断的lr
    } else {
        *(--stk) = 0xFFFFFFFD;    //中断的lr
    }
    *(--stk) = 0;    //R11
    *(--stk) = 0;    //R10
    *(--stk) = 0;    //R9
    *(--stk) = 0;    //R8
    *(--stk) = 0;    //R7
    *(--stk) = 0;    //R6
    *(--stk) = 0;    //R5
    *(--stk) = 0;    //R4

    return stk;
}

void OSTaskSwHook(void) {
    OS_TRACE_TASK_SWITCHED_IN(OSTCBHighRdy);
}

void OS_CPU_ExceptStkBase(void) {}

void OS_KA_BASEPRI_Boundary(void) {}


void OS_TASK_SW(void) {
//    OSCtxSw();
    *(uint32_t *) (0xE000ED04) |= 1 << 28;
}