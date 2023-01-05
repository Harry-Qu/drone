/** 
 *  @brief	    调试任务——PID参数读写
 *  @details    将PID参数保存至EEPROM中，从EEPROM中读取PID参数
 *  @author     Harry-Qu
 *  @date       2023/1/5
 *  @version    1.0
 *  @par        日志
 *              1.0     |       实现读写PID参数功能
*/

#ifndef APP_DEBUG_PID_STORAGE_H
#define APP_DEBUG_PID_STORAGE_H

#include "main.h"
#include "pid.h"

extern pid_type pidRollInner, pidRollOuter, pidPitchInner, pidPitchOuter, pidYaw, pidHeight;

/**
 * 保存PID参数
 * @note 本函数支持在中断中调用，将在任务中执行实际操作
 * @param pid pid地址
 * @param argId 参数ID。0:Kp,1:Ki,2:Kd,3:iL
 * @param data 具体数据
 */
void app_debug_save_pid_data(pid_type *pid, uint8_t argId, float data);

/**
 * 读取PID参数
 * @note 本函数支持在中断中调用，将在任务中执行实际操作
 * @param pid pid地址
 */
void app_debug_read_pid_data(pid_type *pid);

/**
 * 任务中执行函数
 */
void app_debug_pid_storage();


#endif //APP_DEBUG_PID_STORAGE_H
