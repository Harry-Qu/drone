/**
 *  @brief	    串口SDK
 *  @details    普通,DMA方式发送数据，向指定串口注册DMA接收回调函数
 *  @author     Harry-Qu
 *  @date       2022/5/20
 *  @version    1.2
 *  @par        日志
 *              1.0     |       实现普通和DMA方式发送串口数据功能。
 *              1.1     |       实现向指定串口注册DMA接收回调函数功能
 *              1.2     |       降低与ucos-II的耦合，可在非操作系统环境下使用
*/

#include "sdk_usart.h"

#ifdef OS_uCOS_II_H

#if MULTITASK_OCCUPIES_THE_SAME_UART > 0u
OS_EVENT *usartFreeSem[USART_TOTAL_NUM];
#endif

OS_EVENT *usartCompletedSem[USART_TOTAL_NUM];

#endif


sdk_uart_receiver_dma_t uartReceiveDmaData[USART_TOTAL_NUM];

/**
 * 获取中断号
 * @return 获取当前正在执行的中断号
 */
static uint32_t getIntId(void) {
    return ((*(uint32_t *) (0xE000ED04)) & 0x1FF);
}

void sdk_usartdma_init() {
#ifdef OS_uCOS_II_H

    uint8_t i;
    for (i = 0; i < USART_TOTAL_NUM; ++i) {
#if MULTITASK_OCCUPIES_THE_SAME_UART > 0u
        usartFreeSem[i] = OSSemCreate(1);
#endif
        usartCompletedSem[i] = OSSemCreate(0);
    }
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
#ifdef OS_uCOS_II_H
    OS_EVENT *huartCompleteSem = Get_USART_Sem(usartCompletedSem, huart);

    OS_SEM_DATA semData;
    if (OSSemQuery(huartCompleteSem, &semData) == OS_ERR_NONE && semData.OSCnt == 0) {
        //信号量为0
        OSSemPost(huartCompleteSem);
    }
#endif
}

void sdk_uart_RxEventCallback(struct __UART_HandleTypeDef *huart, uint16_t Pos) {
#ifdef OS_uCOS_II_H
    OSIntEnter();
#endif
    uint8_t uartId = UART_ID(huart);
    uartReceiveDmaData[uartId].callbackFunction(huart, Pos);
    memset(uartReceiveDmaData[uartId].pData, 0, uartReceiveDmaData[uartId].size);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uartReceiveDmaData[uartId].pData, uartReceiveDmaData[uartId].size);
#ifdef OS_uCOS_II_H
    OSIntExit();
#endif
}

void sdk_uart_register_receive_dma_callback(void (*callbackFunction)(UART_HandleTypeDef *huart, uint16_t length),
                                            UART_HandleTypeDef *huart,
                                            uint8_t *pData,
                                            uint8_t size) {
    HAL_UART_RegisterRxEventCallback(huart, sdk_uart_RxEventCallback);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, size);

    uint8_t uartId = UART_ID(huart);
    uartReceiveDmaData[uartId].callbackFunction = callbackFunction;
    uartReceiveDmaData[uartId].pData = pData;
    uartReceiveDmaData[uartId].size = size;
    uartReceiveDmaData[uartId].huart = huart;

}


HAL_StatusTypeDef sdk_uart_transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;

#ifdef OS_uCOS_II_H
    if (OSRunning == OS_TRUE) {
        OSSchedLock();
        status = HAL_UART_Transmit(huart, pData, Size, Timeout);
        OSSchedUnlock();
    } else {
        status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    }
#else
    status = HAL_UART_Transmit(huart, pData, Size, Timeout);
#endif

    return status;
}

HAL_StatusTypeDef
sdk_uart_transmit_dma(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout, uint8_t *perr) {
#ifdef OS_uCOS_II_H

#if USART_DMA_EN == 0
    *perr = 0;
    return sdk_uart_transmit(huart, pData, Size, Timeout > 0 ? Timeout : 100);
#else

    if (OSRunning == OS_FALSE || huart->hdmatx == NULL) {
        *perr = 0;
        return sdk_uart_transmit(huart, pData, Size, Timeout > 0 ? Timeout : 100);
    }

    OS_EVENT *huartCompleteSem = Get_USART_Sem(usartCompletedSem, huart);
    OSSemSet(huartCompleteSem, 0, perr);


#if MULTITASK_OCCUPIES_THE_SAME_UART > 0u
    OS_EVENT *huartFreeSem = Get_USART_Sem(usartFreeSem, huart);
    OSSemPend(huartFreeSem, Timeout, perr);
#endif


    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, pData, Size);
    if (status != HAL_OK) {
#if MULTITASK_OCCUPIES_THE_SAME_UART > 0u
        OSSemPost(huartFreeSem);
#endif

#if OS_TRACE_EN > 0
        SEGGER_SYSVIEW_ErrorfTarget("usart async:%X", status);
#endif
        *perr = 0;
        return status;
    }


    OSSemPend(huartCompleteSem, Timeout, perr);


    if (*perr != OS_ERR_NONE) {
        perror("usart timeout");
    }

#if MULTITASK_OCCUPIES_THE_SAME_UART > 0u
    OSSemPost(huartFreeSem);
#endif

    return HAL_OK;

#endif  //USART_DMA_EN

#else   //OS_uCOS_II_H
    *perr = 0;
    return sdk_uart_transmit(huart, pData, Size, Timeout > 0 ? Timeout : 100);
#endif  //OS_uCOS_II_H
}
