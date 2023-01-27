/** 
 *  @brief	    调试任务——PID参数读写
 *  @details    将PID参数保存至EEPROM中，从EEPROM中读取PID参数
 *  @author     Harry-Qu
 *  @date       2023/1/5
 *  @version    1.0
 *  @par        日志
 *              1.0     |       实现读写PID参数功能
*/

#include "app_debug_pid_storage.h"
#include "app_debug_ano.h"
#include "pid.h"
#include "driver_at24c0x.h"

static uint8_t readTask, writeTask;
static uint8_t opAddress;
static float writeData;

static uint8_t get_pid_address(pid_type *pid) {
    uint8_t address = 0x00;
    if (pid == &pidRollInner) {
        address = 0x10;
    } else if (pid == &pidRollOuter) {
        address = 0x20;
    } else if (pid == &pidPitchInner) {
        address = 0x30;
    } else if (pid == &pidPitchOuter) {
        address = 0x40;
    } else if (pid == &pidYaw) {
        address = 0x50;
    } else if (pid == &pidHeight) {
        address = 0x60;
    }
    return address;
}

void app_debug_save_pid_data(pid_type *pid, uint8_t argId, float data) {
    uint8_t address = get_pid_address(pid) | (argId << 2);
    opAddress = address;
    writeTask = 1;
    writeData = data;
}

void app_debug_read_pid_data(pid_type *pid){
    uint8_t address = get_pid_address(pid);
    opAddress = address;
    readTask = 1;
}

void app_debug_pid_storage() {
    if (readTask) {
        struct {
            float p,i,d,l;
        }pidData;
        driver_at24c0x_readData_page(3, opAddress, &pidData, 16);
        char msg[40];
        sprintf(msg, "*p=%.2f i=%.2f d=%.2f iL=%.2f", pidData.p, pidData.i, pidData.d, pidData.l);
        app_debug_ano_log(msg);
        readTask = 0;
    }

    if (writeTask) {
        driver_at24c0x_writeData_page(3, opAddress, &writeData, 4);
        writeTask = 0;
    }
}
