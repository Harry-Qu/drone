/** 
 *  @brief	    输出重定向
 *  @details    重定向输出为uart输出,RTT输出,SystemVIew调试输出
 *  @author     Harry-Qu
 *  @date       2022/7/8
 *  @version    1.1
 *  @par        日志
 *              1.0     |       实现重定向输出到串口,systemview,RTT的功能
 *              1.1     |       支持通过注释头文件的方式对输出功能进行自动裁剪，降低与其他代码的耦合性
*/

#ifndef SDK_IO_H
#define SDK_IO_H

#include "stm32f4xx_hal.h"
#include <sys/stat.h>
#include <stdio.h>
//#include "app_cfg.h"
//#include "SEGGER_RTT.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "usart.h"

#endif

#define FD_NONE 1

#ifdef HAL_UART_MODULE_ENABLED
#define FD_UART 3
#endif

#if OS_TRACE_EN > 0
#define FD_SYSTEMVIEW 5
#define FD_SYSTEMVIEW_ERR 6
#endif

#ifdef SEGGER_RTT_H
#define FD_RTT 7
#endif

#define FD_MAX 8


void sdk_io_init(int fd);

#ifdef HAL_UART_MODULE_ENABLED

void sdk_io_setUart(UART_HandleTypeDef *huart);

#endif

int _isatty(int fd);

int _write(int fd, char *ptr, int len);

int _close(int fd);

int _lseek(int fd, int ptr, int dir);

int _read(int fd, char *ptr, int len);

int _fstat(int fd, struct stat *st);

#endif //SDK_IO_H
