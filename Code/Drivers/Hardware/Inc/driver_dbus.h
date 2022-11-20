/**
* @brief    Dbus遥控器接收机驱动
* @details  Dbus数据接收，解析
* @author   Evan-GH, Harry-Qu
* @date      2019.10
* @version  2.0
* @par Copyright (c):  RM2020电控
* @par 	具体使用方法见Readme.md
				版本变更:
				1.0		|		调通Dbus总线相关中断处理函数,数据解析函数
				1.1		|		添加对接收数据的有效性和正确性的校验
				1.2		|		加入条件编译,分为使用信号量和普通中断两种
				1.3		|		增加对接收缓存数组处理之后的清0操作，确保数据校验准确
				1.4		|		根据代码规范修改了一些函数名称
				1.5		|		修改一些小bug
				1.6		|		修改头文件包含关系，减少.h文件包含过多头文件
				2.0     |       移植到ucos上，支持信号量触发
*/

/**
 * 遥控器摇杆定义
 *
 *      ------------------------------------
 *      |   S1                       S2    |
 *      |                                  |
 *      |       |              |           |
 *      |     -----2         -----0        |
 *      |       |              |           |
 *      |       3              1           |
 *      |                                  |
 *      ------------------------------------
 *
 *
 */

#ifndef __DRIVER_DBUS_H
#define __DRIVER_DBUS_H


#include "app_cfg.h"
#include "main.h"


#ifdef OS_uCOS_II_H
//#define DRIVER_DBUS_USE_SIGNAL    //是否在中断中释放信号量
#endif
//#define DRIVER_DBUS_ANALYSE_IN_IT   //是否在中断中进行数据解析



#define DRIVER_DBUS_USE_KEY_AND_MOUSE 0u //是否使用键鼠功能

#define DRIVER_DBUS_UART           huart1  //遥控器使用的串口编号
#define DRIVER_DBUS_BUFFER_SIZE    100      //Dbus接收缓存的数组大小，有需要请在这里修改,使用空闲中断要求这个数至少要大于18

//遥控器一些相关的宏定义,从官方手册看过来的，如果不需要就注释了吧
#define DRIVER_DBUS_RC_CH_MIN            (-660)     //中点归0后数据
#define DRIVER_DBUS_RC_CH_MID            (1024)
#define DRIVER_DBUS_RC_CH_MAX            (660)      //中点归0后数据
#define DRIVER_DBUS_RC_SW_UP                ((uint8_t)1)
#define DRIVER_DBUS_RC_SW_MID            ((uint8_t)3)
#define DRIVER_DBUS_RC_SW_DOWN            ((uint8_t)2)
//参照了一下上届代码，此处按键定义可能和官方手册有差异，具体可以自己测试一下
#define DRIVER_DBUS_KEY_W                    ((uint16_t)0x01<<0)
#define DRIVER_DBUS_KEY_S                    ((uint16_t)0x01<<1)
#define DRIVER_DBUS_KEY_A                    ((uint16_t)0x01<<2)
#define DRIVER_DBUS_KEY_D                    ((uint16_t)0x01<<3)
#define DRIVER_DBUS_KEY_Q                    ((uint16_t)0x01<<7)
#define DRIVER_DBUS_KEY_E                    ((uint16_t)0x01<<8)
#define DRIVER_DBUS_KEY_CTRL                ((uint16_t)0x01<<6)
#define DRIVER_DBUS_KEY_SHIFT               ((uint16_t)0x01<<5)
#define DRIVER_DBUS_KEY_R                    ((uint16_t)0x01<<9)
#define DRIVER_DBUS_KEY_F                    ((uint16_t)0x01<<10)
#define DRIVER_DBUS_KEY_G                    ((uint16_t)0x01<<11)
#define DRIVER_DBUS_KEY_Z                    ((uint16_t)0x01<<12)
#define DRIVER_DBUS_KEY_X                    ((uint16_t)0x01<<13)
#define DRIVER_DBUS_KEY_C                    ((uint16_t)0x01<<14)
#define DRIVER_DBUS_KEY_V                    ((uint16_t)0x01<<15)
#define DRIVER_DBUS_KEY_B                    ((uint16_t)0x01<<16)

#define DRIVER_DBUS_RC_CH_TOLERANCE_VALUE   10

#define DRIVER_DBUS_OFFLINE_TIMEOUT 100

/*遥控数据结构体*/
typedef struct rc_rec {
    int16_t ch0;        //通道0
    int16_t ch1;        //通道1
    int16_t ch2;        //通道2
    int16_t ch3;        //通道3
    uint8_t S1;            //左拨码开关
    uint8_t S2;            //右拨码开关
    int16_t Dial;        //拨码盘
#if DRIVER_DBUS_USE_KEY_AND_MOUSE > 0
    struct {
        int16_t X;                //X轴
        int16_t Y;                //Y轴
        int16_t Z;                //Z轴
        uint8_t Leftkey;    //右键
        uint8_t Rightkey;    //右键
    } Mouse;                    //鼠标信息
    uint16_t Keys;    //按键信息
#endif
} rc_data_t;

extern rc_data_t rc_data;   //遥控器数据


/**
 * @brief 解析遥控器数据
 */
void driver_dbus_Analysis(void);

/**
 * @brief Dbus总线数据校验
 * @details 校验接收数据的有效性和正确性
 * @retval HAL_OK 数据正常  HAL_ERROR 数据有误
 */
static HAL_StatusTypeDef _DBUS_DataCheck(void);

/**
 * @brief dbus中断处理函数
 */
void driver_dbus_it(void);

/**
 * @brief dbus初始化函数
 */
void driver_dbus_Init(void);


enum DBUS_STATUS_CODE {
    DBUS_OK = 0,
    DBUS_OFFLINE = (1 << 0),
};

/**
 * @brief 获取dbus目前状态的函数
 * @return DBUS_STATUS_CODE
 */
uint32_t driver_dbus_Get_Status(void);

/**
 * @brief 获取dbus的错误详细信息
 * @param status 状态码
 * @param errMessage 返回的错误详细信息
 */
void driver_dbus_Get_Error_Message(uint32_t status, char *errMessage);

#ifdef DRIVER_DBUS_USE_SIGNAL
//extern OS_EVENT *sem_dbus_analysis;
#endif

#endif  //__DRIVER_DBUS_H
