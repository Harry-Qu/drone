/**
 *  @brief	    I2C SDK
 *  @details    普通,DMA方式发送,接收数据
 *  @author     Harry-Qu
 *  @date       2022/5/18
 *  @version    1.3
 *  @par        日志
 *              1.0     |       实现普通和DMA方式发送接收数据功能。
 *              1.1     |       新增从指定内存地址读取数据功能。
 *              1.2     |       降低与ucos-II的耦合，可在非操作系统环境下使用
 *              1.2.1   |       新增向I2C设备指定内存地址写入数据功能
 *                              新增接口为sdk_i2c_memory_write, sdk_i2c_memory_write_dma
 *              1.3     |       新增I2C总线恢复功能
*/

#include "sdk_i2c.h"
#include "sdk_time.h"

#ifdef OS_uCOS_II_H

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
OS_EVENT *i2cFreeSem[I2C_TOTAL_NUM];
#endif

OS_EVENT *i2cCompletedSem[I2C_TOTAL_NUM];

#endif

void sdk_i2c_dma_init() {


#ifdef OS_uCOS_II_H

    uint8_t i;
    for (i = 0; i < I2C_TOTAL_NUM; ++i) {
#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
        i2cFreeSem[i] = OSSemCreate(1);
#endif
        i2cCompletedSem[i] = OSSemCreate(0);
    }

#endif


}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
#if defined(OS_uCOS_II_H) && I2C_DMA_EN > 0

    OS_EVENT *hi2cCompleteSem = Get_I2C_Sem(i2cCompletedSem, hi2c);
    OS_SEM_DATA semData;
    if (OSSemQuery(hi2cCompleteSem, &semData) == OS_ERR_NONE && semData.OSCnt == 0) {
        OSSemPost(hi2cCompleteSem);
    }

#endif
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
#if defined(OS_uCOS_II_H) && I2C_DMA_EN > 0

    OS_EVENT *hi2cCompleteSem = Get_I2C_Sem(i2cCompletedSem, hi2c);
    OS_SEM_DATA semData;
    if (OSSemQuery(hi2cCompleteSem, &semData) == OS_ERR_NONE && semData.OSCnt == 0) {
        OSSemPost(hi2cCompleteSem);
    }

#endif
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
#if defined(OS_uCOS_II_H) && I2C_DMA_EN > 0

    OS_EVENT *hi2cCompleteSem = Get_I2C_Sem(i2cCompletedSem, hi2c);
    OS_SEM_DATA semData;
    if (OSSemQuery(hi2cCompleteSem, &semData) == OS_ERR_NONE && semData.OSCnt == 0) {
        OSSemPost(hi2cCompleteSem);
    }

#endif
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
#if defined(OS_uCOS_II_H) && I2C_DMA_EN > 0

    OS_EVENT *hi2cCompleteSem = Get_I2C_Sem(i2cCompletedSem, hi2c);
    OS_SEM_DATA semData;
    if (OSSemQuery(hi2cCompleteSem, &semData) == OS_ERR_NONE && semData.OSCnt == 0) {
        OSSemPost(hi2cCompleteSem);
    }

#endif
}


HAL_StatusTypeDef
sdk_i2c_transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;

#ifdef OS_uCOS_II_H
    if (OSRunning == OS_TRUE) {
        OSSchedLock();
        status = HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, Timeout);
        OSSchedUnlock();
    } else {
        status = HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, Timeout);
    }
#else
    status = HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, Timeout);
#endif

    return status;
}


HAL_StatusTypeDef
sdk_i2c_transmit_dma(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout,
                     uint8_t *perr) {
#ifdef OS_uCOS_II_H
    // 使用了ucos


#if I2C_DMA_EN == 0
    *perr = 0;
    return sdk_i2c_transmit(hi2c, DevAddress, pData, Size, Timeout > 0 ? Timeout : 100);
#else

    if (OSRunning == OS_FALSE || hi2c->hdmatx == NULL) {
        //当前没在跑操作系统或发送端没开DMA功能，不能调用DMA功能
        return sdk_i2c_transmit(hi2c, DevAddress, pData, Size, Timeout > 0 ? Timeout : 100);
    }

    Timeout = 0;

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
    OS_EVENT *hi2cFreeSem = Get_I2C_Sem(i2cFreeSem, hi2c);
    OSSemPend(hi2cFreeSem, Timeout, perr);
#endif


    OS_EVENT *hi2cCompleteSem = Get_I2C_Sem(i2cCompletedSem, hi2c);
    OSSemSet(hi2cCompleteSem, 0, perr);


    //    I2C状态异常报error
    //    if (hi2c->State != HAL_I2C_STATE_READY || hi2c->hdmatx->State != HAL_DMA_STATE_READY) {
    //        SEGGER_SYSVIEW_Error("not ready");
    //    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress, pData, Size);
    if (status != HAL_OK) {
        //DMA发送异常
        *perr = 0;
#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
        OSSemPost(hi2cFreeSem);
#endif
        return status;
    }


    OSSemPend(hi2cCompleteSem, Timeout, perr);
    if (*perr != OS_ERR_NONE) {
        //申请信号量异常
    }

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
    OSSemPost(hi2cFreeSem);
#endif
    return HAL_OK;

#endif  //I2C_DMA_EN

#else   //OS_uCOS_II_H

    *perr = 0;
    return sdk_i2c_transmit(hi2c, DevAddress, pData, Size, Timeout > 0 ? Timeout : 100);

#endif  //OS_uCOS_II_H

}

HAL_StatusTypeDef
sdk_i2c_receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;

#ifdef OS_uCOS_II_H
    if (OSRunning == OS_TRUE) {
        OSSchedLock();
        status = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, Size, Timeout);
        OSSchedUnlock();
    } else {
        status = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, Size, Timeout);
    }
#else
    status = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, Size, Timeout);
#endif

    return status;
}


HAL_StatusTypeDef
sdk_i2c_receive_dma(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout,
                    uint8_t *perr) {
#ifdef OS_uCOS_II_H
    // 使用了ucos

#if I2C_DMA_EN == 0
    *perr = 0;
    return sdk_i2c_receive(hi2c, DevAddress, pData, Size, Timeout > 0 ? Timeout : 100);
#else

    if (OSRunning == OS_FALSE || hi2c->hdmarx == NULL) {
        //当前没在跑操作系统或接收没开DMA功能，不能调用DMA功能
        *perr = 0;
        return sdk_i2c_receive(hi2c, DevAddress, pData, Size, Timeout > 0 ? Timeout : 100);
    }

    Timeout = 0;

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
    OS_EVENT *hi2cFreeSem = Get_I2C_Sem(i2cFreeSem, hi2c);
    OSSemPend(hi2cFreeSem, Timeout, perr);
#endif

    OS_EVENT *hi2cCompleteSem = Get_I2C_Sem(i2cCompletedSem, hi2c);
    OSSemSet(hi2cCompleteSem, 0, perr);

    //    I2C状态异常报error
    //    if (hi2c->State != HAL_I2C_STATE_READY || hi2c->hdmatx->State != HAL_DMA_STATE_READY) {
    //        SEGGER_SYSVIEW_Error("not ready");
    //    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(hi2c, DevAddress, pData, Size);
    if (status != HAL_OK) {
        //DMA接收异常
        *perr = 0;
#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
        OSSemPost(hi2cFreeSem);
#endif
        return status;
    }


    OSSemPend(hi2cCompleteSem, Timeout, perr);
    if (*perr != OS_ERR_NONE) {
        //申请信号量异常
    }

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
    OSSemPost(hi2cFreeSem);
#endif

    return HAL_OK;


#endif  //I2C_DMA_EN

#else   //OS_uCOS_II_H

    *perr = 0;
    return sdk_i2c_receive(hi2c, DevAddress, pData, Size, Timeout > 0 ? Timeout : 100);

#endif  //OS_uCOS_II_H
}

HAL_StatusTypeDef
sdk_i2c_memory_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                    uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;

#ifdef OS_uCOS_II_H
    if (OSRunning == OS_TRUE) {
        OSSchedLock();
        status = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
        OSSchedUnlock();
    } else {
        status = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
    }
#else
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
#endif

    return status;
}

HAL_StatusTypeDef
sdk_i2c_memory_read_dma(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                        uint8_t *pData, uint16_t Size, uint32_t Timeout, uint8_t *perr) {
#ifdef OS_uCOS_II_H


#if I2C_DMA_EN == 0
    *perr = 0;
    return sdk_i2c_memory_read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout > 0 ? Timeout : 100);
#else

    if (OSRunning == OS_FALSE || hi2c->hdmarx == NULL) {
        //当前没在跑操作系统或接收没开DMA，不能调用DMA功能
        *perr = 0;
        return sdk_i2c_memory_read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout > 0 ? Timeout : 100);
    }

    Timeout = 0;

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
    OS_EVENT *hi2cFreeSem = Get_I2C_Sem(i2cFreeSem, hi2c);
    OSSemPend(hi2cFreeSem, Timeout, perr);
#endif


    OS_EVENT *hi2cCompleteSem = Get_I2C_Sem(i2cCompletedSem, hi2c);
    OSSemSet(hi2cCompleteSem, 0, perr);


//    I2C状态异常报error
//    if (hi2c->State != HAL_I2C_STATE_READY || hi2c->hdmatx->State != HAL_DMA_STATE_READY) {
//        SEGGER_SYSVIEW_Error("not ready");
//    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
    //运行结束会调用HAL_I2C_MemRxCpltCallback

    if (status != HAL_OK) {
        *perr = 0;
#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
        OSSemPost(hi2cFreeSem);
#endif
        return status;
    }

    OSSemPend(hi2cCompleteSem, Timeout, perr);
    if (*perr != OS_ERR_NONE) {
    }

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
    OSSemPost(hi2cFreeSem);
#endif

    return HAL_OK;

#endif  //I2C_DMA_EN

#else   //OS_uCOS_II_H

    *perr = 0;
    return sdk_i2c_memory_read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout > 0 ? Timeout : 100);

#endif  //OS_uCOS_II_H
}

HAL_StatusTypeDef
sdk_i2c_memory_write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                     uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;

#ifdef OS_uCOS_II_H
    if (OSRunning == OS_TRUE) {
        OSSchedLock();
        status = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
        OSSchedUnlock();
    } else {
        status = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
    }
#else
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
#endif

    return status;
}

HAL_StatusTypeDef
sdk_i2c_memory_write_dma(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                         uint8_t *pData, uint16_t Size, uint32_t Timeout, uint8_t *perr) {
#ifdef OS_uCOS_II_H


#if I2C_DMA_EN == 0
    *perr = 0;
    return sdk_i2c_memory_write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout > 0 ? Timeout : 100);
#else

    if (OSRunning == OS_FALSE || hi2c->hdmarx == NULL) {
        //当前没在跑操作系统或接收没开DMA，不能调用DMA功能
        *perr = 0;
        return sdk_i2c_memory_write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout > 0 ? Timeout : 100);
    }

    Timeout = 0;

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
    OS_EVENT *hi2cFreeSem = Get_I2C_Sem(i2cFreeSem, hi2c);
    OSSemPend(hi2cFreeSem, Timeout, perr);
#endif


    OS_EVENT *hi2cCompleteSem = Get_I2C_Sem(i2cCompletedSem, hi2c);
    OSSemSet(hi2cCompleteSem, 0, perr);


//    I2C状态异常报error
//    if (hi2c->State != HAL_I2C_STATE_READY || hi2c->hdmatx->State != HAL_DMA_STATE_READY) {
//        SEGGER_SYSVIEW_Error("not ready");
//    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
    //运行结束会调用HAL_I2C_MemRxCpltCallback

    if (status != HAL_OK) {
        *perr = 0;
#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
        OSSemPost(hi2cFreeSem);
#endif
        return status;
    }

    OSSemPend(hi2cCompleteSem, Timeout, perr);
    if (*perr != OS_ERR_NONE) {
    }

#if MULTITASK_OCCUPIES_THE_SAME_I2C > 0
    OSSemPost(hi2cFreeSem);
#endif

    return HAL_OK;

#endif  //I2C_DMA_EN

#else   //OS_uCOS_II_H

    *perr = 0;
    return sdk_i2c_memory_write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout > 0 ? Timeout : 100);

#endif  //OS_uCOS_II_H
}


void sdk_i2c_recovery(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_DeInit(hi2c);

    GPIO_TypeDef *sda_GPIO = NULL, *scl_GPIO = NULL;
    uint32_t sda_Pin, scl_Pin;

#ifdef I2C1_SCL_GPIO_Port
    if (hi2c->Instance == I2C1) {
        sda_GPIO = I2C1_SDA_GPIO_Port;
        sda_Pin = I2C1_SDA_Pin;
        scl_GPIO = I2C1_SCL_GPIO_Port;
        scl_Pin = I2C1_SCL_Pin;
    }
#endif

#ifdef I2C2_SCL_GPIO_Port
    if (hi2c->Instance == I2C2) {
        sda_GPIO = I2C2_SDA_GPIO_Port;
        sda_Pin = I2C2_SDA_Pin;
        scl_GPIO = I2C2_SCL_GPIO_Port;
        scl_Pin = I2C2_SCL_Pin;
    }
#endif

#ifdef I2C3_SCL_GPIO_Port
    if (hi2c->Instance == I2C3) {
        sda_GPIO = I2C3_SDA_GPIO_Port;
        sda_Pin = I2C3_SDA_Pin;
        scl_GPIO = I2C3_SCL_GPIO_Port;
        scl_Pin = I2C3_SCL_Pin;
    }
#endif

    if (sda_GPIO == NULL) {
        HAL_I2C_Init(hi2c);
        return;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Pin = sda_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(sda_GPIO, &GPIO_InitStruct);
    if (HAL_GPIO_ReadPin(sda_GPIO, sda_Pin) == GPIO_PIN_RESET) {
        printf("[warning] i2C bus is busy!!!\n");
        GPIO_InitStruct.Pin = scl_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(scl_GPIO, &GPIO_InitStruct);

        uint8_t i = 0;
        for (; i < 9; ++i) {
            HAL_GPIO_WritePin(scl_GPIO, scl_Pin, GPIO_PIN_RESET);
            delay_us(4);
            if (HAL_GPIO_ReadPin(sda_GPIO, sda_Pin) == GPIO_PIN_SET) {
                break;
            }
            HAL_GPIO_WritePin(scl_GPIO, scl_Pin, GPIO_PIN_SET);
            delay_us(4);
        }
        if (i < 9 || HAL_GPIO_ReadPin(sda_GPIO, sda_Pin) == GPIO_PIN_SET) {
            GPIO_InitStruct.Pin = sda_Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            HAL_GPIO_Init(sda_GPIO, &GPIO_InitStruct);
            HAL_GPIO_WritePin(sda_GPIO, sda_Pin, GPIO_PIN_RESET);
            delay_us(4);
            HAL_GPIO_WritePin(scl_GPIO, scl_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(sda_GPIO, sda_Pin, GPIO_PIN_SET);
        }
    }
    HAL_I2C_Init(hi2c);
}
