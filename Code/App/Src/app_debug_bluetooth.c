/** 
 *  @brief	    调试任务-蓝牙部分
 *  @details    实现接收上位机调参指令功能，需使用DMA
 *  @author     Harry-Qu
 *  @date       2022/10/16
 *  @version    1.1
 *  @par        日志
 *              1.0     |       实现蓝牙调试基础功能
 *              1.1     |       新增存储PID参数，读取PID参数功能
*/

/**
 * 蓝牙调试PID发送消息格式
 * pid名称,参数名=两位小数
 * 数据长度[8,10]
 * 样例:
 *      pi,i=0.25       ;pitch inner, ki=0.25
 *      po,p=-0.41      ;pitch outer, kp=-0.41
 *      ri,d=1.01       ;roll inner, kd=1.01
 *      ro,p=0.40       ;roll outer, kp=0.40
 *      y,p=1.00        ;yaw, kp=1.00
 *      h,p=-0.50       ;height, kp=-0.50
 *
 * 蓝牙读取PID历史参数功能
 * 样例:
 *      pi=?
 *      y=?
 *
 */

#include "app_debug_bluetooth.h"
#include "sdk_usart.h"
#include "sdk_math.h"
#include "app_control.h"
#include "app_debug_ano.h"
#include "pid.h"
#include "AHRS.h"
#include <stdlib.h>
#include "app_debug_pid_storage.h"


uint8_t bluetooth_dataBuffer[100];


/**
 * pid参数调参，根据上位机指令修改PID参数，调试用
 * @param dataBuffer 数据缓冲区
 * @param length 数据长度
 */
static void app_bluetooth_solvePid(char *dataBuffer, uint16_t length) {


    pid_type *pid = NULL;  //调的是哪个pid
    int pos;                //参数位置
    int8_t argId = -1;    //哪个参数 0=Kp,1=Ki,2=Kd


    switch (dataBuffer[0]) {
        case 'P':
        case 'p': {
            switch (dataBuffer[1]) {
                case 'I':
                case 'i':
                    pid = &pidPitchInner;
                    pos = 3;
                    break;
                case 'O':
                case 'o':
                    pid = &pidPitchOuter;
                    pos = 3;
                    break;
            }
            break;
        }
        case 'R':
        case 'r': {
            switch (dataBuffer[1]) {
                case 'I':
                case 'i':
                    pid = &pidRollInner;
                    pos = 3;
                    break;
                case 'O':
                case 'o':
                    pid = &pidRollOuter;
                    pos = 3;
                    break;
            }
            break;
        }
        case 'Y':
        case 'y':
            pid = &pidYaw;
            pos = 2;
            break;
        case 'H':
        case 'h':
            pid = &pidHeight;
            pos = 2;
            break;
    }

    if (pid == NULL) {
        return;
    }

    if (dataBuffer[pos - 1] == '=' && dataBuffer[pos] == '?') {
#if DEBUG_PID_STORAGE_EN > 0
        app_debug_read_pid_data(pid);
#endif
    } else {
        switch (dataBuffer[pos]) {
            case 'p':
                argId = 0;
                break;
            case 'i':
                argId = 1;
                break;
            case 'd':
                argId = 2;
                break;
            case 'l':
                argId = 3;
                break;
        }
        if (argId == -1 || dataBuffer[pos + 1] != '=') {
            return;
        }
        float value = convertString2Float((char *) dataBuffer + (pos + 2));

        PID_setArg(pid, argId, value);
#if DEBUG_PID_STORAGE_EN > 0
        app_debug_save_pid_data(pid, argId, value);
#endif
        char replyString[80];
        PID_getArgString(pid, replyString);
        app_debug_ano_log(replyString);
    }


}

/**
 * Mahony参数修改，接收上位机指令并对Mahony相关参数进行修改，调试用
 * @param dataBuffer 数据缓冲区
 * @param length 接收的数据长度
 */
static void app_bluetooth_solveMahony(char *dataBuffer, uint16_t length) {

    if (length < 3) {
        return;
    }
    float value = convertString2Float(dataBuffer + 2);
    if (isnanf(value)) {
        return;
    }
    switch (dataBuffer[0]) {
        case 'p':
            twoKp = value;
            break;
        case 'i':
            twoKi = value;
            break;
        case 'b':
            betaMax = value;
            break;
        case 's':
            stepMax = value;
            break;
        case 'k':
            betaK = value;
            break;
    }

    char replyMsg[10];
    sprintf(replyMsg, "%c %.4f", dataBuffer[0], value);
    sdk_ano_transmit_log_string(0, replyMsg);
}

/**
 * 蓝牙回调处理函数
 * @param huart 有DMA接收的huart
 * @param length 数据长度
 */
void app_bluetooth_callback(UART_HandleTypeDef *huart, uint16_t length) {
    app_bluetooth_solvePid(bluetooth_dataBuffer, length);
//    app_bluetooth_solveMahony(bluetooth_dataBuffer, length);
    printf("%s\n", bluetooth_dataBuffer);
    printf("%d\n", length);

}

void app_bluetooth_init(void) {
    sdk_uart_register_receive_dma_callback(app_bluetooth_callback, &BTUart, bluetooth_dataBuffer, 100);
//    sdk_uart_register_receive_dma_callback(app_bluetooth_callback, &huart2, bluetooth_dataBuffer, 100);
}