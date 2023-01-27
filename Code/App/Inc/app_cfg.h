#ifndef _APP_CFG_H
#define _APP_CFG_H

#define OS_TRACE_EN 1u          //是否开启SystemView功能


#define USART_DMA_EN 1u       //是否开启异步USART发送
#define MULTITASK_OCCUPIES_THE_SAME_UART 1u //是否允许多任务占用同一串口功能（多任务都会调用串口通信时，需开启此功能）

#define I2C_DMA_EN 1u         //是否开启异步I2C发送接收
#define MULTITASK_OCCUPIES_THE_SAME_I2C 0u  //是否开启多任务占用同一I2C功能(多个任务有可能同时使用同一I2C通道时，需开启此功能)


#define TASK_DEBUG_EN 1u

#define DEBUG_OLED_EN 0u    //是否使用OLED显示调试数据
#define DEBUG_ANO_EN 1u     //是否发送匿名上位机数据
#define DEBUG_BT_EN 1u      //是否接收上位机调试信息
#define DEBUG_PID_STORAGE_EN 1u //是否使用ROM记录PID数据

#define DEVICE_RGB_EN 1u


#if (((DEBUG_OLED_EN>0u) || (DEBUG_ANO_EN>0u) || (DEBUG_BT_EN>0u)) && (TASK_DEBUG_EN == 0u))
#error "if DEBUG is enabled, TASK_DEBUG_EN must be enabled."
#endif


#endif
