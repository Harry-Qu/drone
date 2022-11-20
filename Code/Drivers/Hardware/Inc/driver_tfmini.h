/** 
 *  @brief	    TFMini-S雷达模组驱动
 *  @details    支持通过DMA方式接收数据帧并解析
 *  @author     Harry-Qu
 *  @date       2022/11/6
 *  @version    1.0
 *  @par        日志
*/

#ifndef TFMINI_DRIVER_TFMINI_H
#define TFMINI_DRIVER_TFMINI_H

#include "main.h"

#define TFMINI_UART huart6 //TFMINI模块使用的uart口
#define TFMINI_TIMEOUT 2000    //超时时间，毫秒（该时间段内未接收到新数据视为超时）

typedef struct{
    int16_t distance;   //距离，单位为cm，范围为0~2000，当测量值不可信时，输出-1
    uint16_t strength;  //信号强度，0~65535。测距越远，信号强度越低;目标物反射率越低，信号强度越低。
    uint16_t temp;  //摄氏度 = Temp/8-256
}tfmini_t;

/**
 * TFMINI向匿名上位机发送高度数据
 */
void driver_tfmini_transmitHeightData(void);

/**
 * TFMINI解析接收到的原始数据
 * @return 解析状态
 */
HAL_StatusTypeDef driver_tfmini_analyse(void);

/**
 * TFMINI模块初始化，注册DMA接收回调函数
 */
void driver_tfmini_init(void);

#endif //TFMINI_DRIVER_TFMINI_H
