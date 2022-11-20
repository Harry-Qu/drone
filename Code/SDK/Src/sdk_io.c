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

#include "sdk_io.h"
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <stdint.h>
#include <sys/fcntl.h>




#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

#ifdef HAL_UART_MODULE_ENABLED

UART_HandleTypeDef *gHuart;

#endif

int usrFd = 0;

int timesCal = 0;

void sdk_io_init(int fd) {
    usrFd = fd;
    setvbuf(stdout, NULL, _IONBF, 0);

#ifdef HAL_UART_MODULE_ENABLED
    if (fd == FD_UART) {
        sdk_io_setUart(&huart2);
    }
#endif

}

#ifdef HAL_UART_MODULE_ENABLED

void sdk_io_setUart(UART_HandleTypeDef *huart) {
    gHuart = huart;
}

#endif

int _isatty(int fd) {
    if (fd >= 0 && fd <= FD_MAX)
        return 1;
    errno = EBADF;
    return 0;
}


int _write(int fd, char *ptr, int len) {
    HAL_StatusTypeDef hstatus;
    fd = usrFd;
    ptr[len] = '\0';
    if (fd == FD_NONE) {
        return EIO;

#ifdef HAL_UART_MODULE_ENABLED
    } else if (fd == FD_UART) {
#ifdef OS_uCOS_II_H
        OSSchedLock();
#endif
        hstatus = HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
#ifdef OS_uCOS_II_H
        OSSchedUnlock();
#endif
        if (hstatus == HAL_OK)
            return len;
        else
            return EIO;
#endif  //HAL_UART_MODULE_ENABLED

#if OS_TRACE_EN > 0
    } else if (fd == FD_SYSTEMVIEW) {
        SEGGER_SYSVIEW_Print(ptr);
        return len;
    } else if (fd == FD_SYSTEMVIEW_ERR) {
        SEGGER_SYSVIEW_Error(ptr);
        return len;
#endif

#ifdef SEGGER_RTT_H
    } else if (fd == FD_RTT) {
        SEGGER_RTT_Write(0, ptr, len);
        return len;
#endif

    }
    errno = EBADF;
    return -1;
}

int _close(int fd) {
    if (fd >= 0 && fd <= FD_MAX)
        return 0;
    errno = EBADF;
    return -1;
}


int _lseek(int fd, int ptr, int dir) {
    (void) fd;
    (void) ptr;
    (void) dir;

    errno = EBADF;
    return -1;
}

int _read(int fd, char *ptr, int len) {
    HAL_StatusTypeDef hstatus;
    fd = usrFd;

    if (fd == FD_NONE) {
        return EIO;
#ifdef HAL_UART_MODULE_ENABLED
    } else if (fd == FD_UART) {
        hstatus = HAL_UART_Receive(gHuart, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
        if (hstatus == HAL_OK)
            return 1;
        else
            return EIO;
#endif
    }
    errno = EBADF;
    return -1;
}

int _fstat(int fd, struct stat *st) {
    if (fd >= 0 && fd <= FD_MAX) {
        st->st_mode = S_IFCHR;
        return 0;
    }
    errno = EBADF;
    return 0;
}
