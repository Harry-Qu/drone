/** 
 *  @brief	    调试任务-匿名上位机部分
 *  @details    该文件内函数会被周期性调用，用于向匿名上位机发送数据
 *  @author     Harry-Qu
 *  @date       2022/11/2
 *  @version    1.0
 *  @par        日志
*/

#ifndef APP_DEBUG_ANO_H
#define APP_DEBUG_ANO_H

#include "main.h"

/**
 * 向匿名上位机发送日志
 * @details 调用该函数后，会先保存待发送的内容，在匿名上位机发送数据时再统一发送，主要用于在中断中和其他任务中使用防止串口资源冲突
 * @note 发送的日志信息长度需小于39
 * @param msg 日志消息
 */
void app_debug_ano_log(char *msg);

/**
 * 发送调试信息到匿名上位机
 */
void app_debug_ano(void);


#endif //APP_DEBUG_ANO_H
