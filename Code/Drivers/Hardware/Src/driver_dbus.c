/**
 *  @brief      Dbus遥控器接收机驱动
 *  @details    Dbus数据接收，解析
 *  @author     Harry-Qu
 *  @date       2021.10
 *  @version    1.1
 *  @par        Copyright (c):  四轴小组
 *  @par    版本变更:
 *  			1.0		|		实现基本功能，移植到ucos上，支持信号量触发
 *  			1.1     |       降低对ucosII的耦合，可在未移植操作系统时使用
*/

#include "driver_dbus.h"

#include "app_cfg.h"
#include "usart.h"
#include "string.h"

rc_data_t rc_raw_data; //dbus原始数据
rc_data_t rc_data; //Dbus解算数据
uint8_t Dbus_Rxbuffer[DRIVER_DBUS_BUFFER_SIZE] = {0}; //Dbus接收数据缓存数组
uint8_t Dbus_buffer[18];    //有用数据缓存
static uint32_t last_update_timestamp = 0;

#ifdef OS_uCOS_II_H
OS_EVENT *sem_dbus_analysis;
#endif

uint16_t cnt = 0;


void driver_dbus_Init(void) {
    __HAL_UART_CLEAR_IDLEFLAG(&DRIVER_DBUS_UART);
    __HAL_UART_ENABLE_IT(&DRIVER_DBUS_UART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DRIVER_DBUS_UART, Dbus_Rxbuffer, DRIVER_DBUS_BUFFER_SIZE);
    last_update_timestamp = HAL_GetTick();

#ifdef DRIVER_DBUS_USE_SIGNAL
    sem_dbus_analysis = OSSemCreate(0);
#endif

}

uint32_t driver_dbus_Get_Status(void) {
    if (HAL_GetTick() - last_update_timestamp >= DRIVER_DBUS_OFFLINE_TIMEOUT) {
        return DBUS_OFFLINE;
    } else {
        return DBUS_OK;
    }
}

void driver_dbus_Get_Error_Message(uint32_t status, char *errMessage) {
    if (status == 0) {
        strcpy(errMessage, "OK!");
    } else if (status & DBUS_OFFLINE) {
        strcpy(errMessage, "RC OFFLINE!");
    }
}

void driver_dbus_Analysis(void) {


    rc_raw_data.ch0 = ((int16_t) Dbus_buffer[0] | (((int16_t) Dbus_buffer[1] << 8))) & 0x07FF;
    rc_raw_data.ch1 = ((int16_t) Dbus_buffer[1] >> 3 | (((int16_t) Dbus_buffer[2] << 5))) & 0x07FF;
    rc_raw_data.ch2 = ((int16_t) Dbus_buffer[2] >> 6 | (((int16_t) Dbus_buffer[3] << 2)) |
                       ((int16_t) Dbus_buffer[4]) << 10) & 0x07FF;
    rc_raw_data.ch3 = ((int16_t) Dbus_buffer[4] >> 1 | (((int16_t) Dbus_buffer[5] << 7))) & 0x07FF;
    rc_raw_data.S1 = ((Dbus_buffer[5] >> 4) & 0X000C) >> 2;
    rc_raw_data.S2 = (Dbus_buffer[5] >> 4) & 0X0003;
    rc_raw_data.Dial = ((int16_t) Dbus_buffer[16] | ((int16_t) Dbus_buffer[17] << 8)) & 0x07FF;        //拨码盘数据，需要更新遥控器固件


#if DRIVER_DBUS_USE_KEY_AND_MOUSE > 0 //键鼠控制部分
    rc_data.Mouse.X = ((int16_t) Dbus_buffer[6]) | ((int16_t) Dbus_buffer[7] << 8);
    rc_data.Mouse.Y = ((int16_t) Dbus_buffer[8]) | ((int16_t) Dbus_buffer[9] << 8);
    rc_data.Mouse.Z = ((int16_t) Dbus_buffer[10]) | ((int16_t) Dbus_buffer[11] << 8);
    rc_data.Mouse.Leftkey = Dbus_buffer[12];
    rc_data.Mouse.Rightkey = Dbus_buffer[13];
    rc_data.Keys = ((int16_t) Dbus_buffer[14] | (int16_t) Dbus_buffer[15] << 8);
#endif




    //遥控器数据归一化
    rc_raw_data.ch0 -= DRIVER_DBUS_RC_CH_MID;
    rc_raw_data.ch1 -= DRIVER_DBUS_RC_CH_MID;
    rc_raw_data.ch2 -= DRIVER_DBUS_RC_CH_MID;
    rc_raw_data.ch3 -= DRIVER_DBUS_RC_CH_MID;
    rc_raw_data.Dial -= DRIVER_DBUS_RC_CH_MID;

    memcpy(&rc_data, &rc_raw_data, sizeof(rc_data_t));
}


void driver_dbus_it(void) {
    if (__HAL_UART_GET_FLAG(&DRIVER_DBUS_UART, UART_FLAG_IDLE) != RESET) {    //如果产生了空闲中断
        HAL_UART_DMAStop(&DRIVER_DBUS_UART);
        int len = DRIVER_DBUS_BUFFER_SIZE - (DRIVER_DBUS_UART.hdmarx->Instance->NDTR);
        if (len == 18) {
            memcpy(Dbus_buffer, Dbus_Rxbuffer, 18);
#ifdef DRIVER_DBUS_USE_SIGNAL  //如果使用信号量
            OSSemPost(sem_dbus_analysis);
#else   //没有使用信号量

#ifdef DRIVER_DBUS_ANALYSE_IN_IT
            driver_dbus_Analysis(); //直接在中断里面解析数据
#endif

#endif
        }
        last_update_timestamp = HAL_GetTick();  //更新获取数据的时间戳
        __HAL_UART_CLEAR_IDLEFLAG(&DRIVER_DBUS_UART);
        memset(Dbus_Rxbuffer, 0, sizeof(Dbus_Rxbuffer));
        HAL_UART_Receive_DMA(&DRIVER_DBUS_UART, Dbus_Rxbuffer, DRIVER_DBUS_BUFFER_SIZE);
    }

}