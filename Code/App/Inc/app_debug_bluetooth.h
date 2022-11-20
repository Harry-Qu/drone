/** 
 *  @brief	    调试任务-蓝牙部分
 *  @details    实现接收上位机调参指令功能，需使用DMA
 *  @author     Harry-Qu
 *  @date       2022/10/16
 *  @version    1.0
 *  @par        日志
*/

#ifndef GY86_APP_BLUETOOTH_H
#define GY86_APP_BLUETOOTH_H

#include "main.h"
#include "usart.h"

#define BTUart huart6

void app_bluetooth_init(void);


#endif //GY86_APP_BLUETOOTH_H
