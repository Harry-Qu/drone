/** 
 *  @brief	    TFMini-S雷达模组驱动
 *  @details    支持通过DMA方式接收数据帧并解析
 *  @author     Harry-Qu
 *  @date       2022/11/6
 *  @version    1.0
 *  @par        日志
*/

#include "driver_tfmini.h"
#include "usart.h"
#include "sdk_usart.h"
#include "sdk_ano.h"


uint32_t tfmini_lastUpdateTime; //最近一次数据更新时间
uint8_t tfmini_dataBuffer[100]; //DMA接收数据缓冲区
uint8_t tfmini_raw_data[6]; //最新的有效原始数据
tfmini_t tfmini_data; //解析后数据


void driver_tfmini_transmitHeightData(void) {
    struct {
        int32_t fu;
        int32_t add;
        uint8_t sta;
    } sendData;

    sendData.fu = 0;
    sendData.add = tfmini_data.distance;
    sendData.sta = 0;
//    sdk_ano_transmit_height(&sendData);
//    printf("%d\n", tfmini_data.distance);
}

HAL_StatusTypeDef driver_tfmini_analyse(void) {
    if (HAL_GetTick() - tfmini_lastUpdateTime > TFMINI_TIMEOUT){
        // 上次更新时间超过TFMINI_TIMEOUT，视为数据超时
        return HAL_TIMEOUT;
    }

    SDK_UART_PAUSE_IDLE_IT(&TFMINI_UART);

    tfmini_data.distance = (int16_t) (tfmini_raw_data[1] << 8) | tfmini_raw_data[0];
    tfmini_data.strength = (uint16_t) (tfmini_raw_data[3] << 8) | tfmini_raw_data[2];
    tfmini_data.temp = (uint16_t) (tfmini_raw_data[5] << 8) | tfmini_raw_data[4];

    SDK_UART_RESUME_IDLE_IT(&TFMINI_UART);

    driver_tfmini_transmitHeightData();

    return HAL_OK;

}


void driver_tfmini_it_callback(UART_HandleTypeDef *huart, uint16_t length) {
    uint8_t checkSum = 0xB2;


    if (length == 9) {
        if (tfmini_dataBuffer[0] == 0x59 && tfmini_dataBuffer[1] == 0x59) {
            // 不使用循环可利用流水线特性加速计算
            checkSum += tfmini_dataBuffer[2];
            checkSum += tfmini_dataBuffer[3];
            checkSum += tfmini_dataBuffer[4];
            checkSum += tfmini_dataBuffer[5];
            checkSum += tfmini_dataBuffer[6];
            checkSum += tfmini_dataBuffer[7];
            if (checkSum == tfmini_dataBuffer[8]) {
                memcpy(tfmini_raw_data, tfmini_dataBuffer + 2, 6);
                tfmini_lastUpdateTime = HAL_GetTick();
            }
        }
    }
}

void driver_tfmini_init(void) {
    sdk_uart_register_receive_dma_callback(driver_tfmini_it_callback, &TFMINI_UART, tfmini_dataBuffer, 100);
}