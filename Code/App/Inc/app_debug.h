/** 
 *  @brief	    调试相关任务
 *  @details    包含OLED显示参数，向匿名上位机发送数据，接收上位机调参指令功能
 *  @author     Harry-Qu
 *  @date       2022/11/2
 *  @version    1.1
 *  @par        日志
 *              1.0     |       实现调试相关任务
 *              1.1     |       新增PID调试数据存储功能
*/

#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include "main.h"

#if DEBUG_OLED_EN > 0u
#include "app_debug_oled.h"
#endif

#if DEBUG_ANO_EN > 0u
#include "app_debug_ano.h"
#endif


#if DEBUG_BT_EN > 0u
#include "app_debug_bluetooth.h"
#endif

#if DEBUG_PID_STORAGE_EN > 0
#include "app_debug_pid_storage.h"
#endif

void app_debug_init(void);

void app_debug_task(void *args);

#endif //APP_DEBUG_H
