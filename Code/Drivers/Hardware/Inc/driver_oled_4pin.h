/** 
 *  @brief	    oled驱动
 *  @details    基于SSD1306开发
 *  @author     Asn，Evan-GH, Harry-Qu
 *  @date       2019.11.5
 *  @version    3.4
 *  @par        Copyright (c): Asn，Evan-GH, Harry-Qu
 *  @par        日志
                V1.0 移植中景园oled例程成功
>>>>>>>>>>>>>>>Evan-GH的开发分界线>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
				V1.1b 根据代码规范做了一些修改，移植一部分Evasn-GH的旧oled库到这里来，现在可以显示浮点数了呢
				V2.0 修改逻辑层，将app_oled文件函数转移到这里，OLED现在只有driver这一个层的文件
				V2.1 为函数增加了注释
				V2.2 修改文件依赖关系，现在不再调用app层文件
>>>>>>>>>>>>>>>Harry-Qu的开发分界线>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
				V3.0移植代码至4pin OLED
				V3.1重构渲染逻辑，构建全部显示内容再统一发送，提高指令发送效率，将单独发送控制指令修改为批量发送控制指令。
				V3.2提高数字、字符串等显示效率，修改显示器刷新顺序，改为行刷新模式，显示时先生成待显示像素矩阵，一次性写入OLED内存中。
				V3.3新增溢出判断，当待显示数字、字符串超出屏幕宽度时，溢出部分以省略号表示。
				V3.4修复部分整数，浮点数显示的bug
*/

#ifndef __DRIVER_OLED_4PIN_H
#define __DRIVER_OLED_4PIN_H

#include "i2c.h"
#include <math.h>


#define DRIVER_OLED_I2C                    hi2c1
//-----------------OLED定义----------------  					   
#define DRIVER_OLED_CMD  0    //写命令
#define DRIVER_OLED_DATA 1    //写数据

#define DRIVER_OLED_ADDRESS 0x3C

enum COMMAND_AND_DATA {
    COMMAND = 0,
    DATA = 1
};

enum ALIGN {
    LEFT = 0,
    CENTER,
    RIGHT
};


/********************功能函数********************/

void driver_oled_Init(void);    //OLED初始化
void driver_oled_Display_On(void);    //开启OLED显示
void driver_oled_Display_Off(void);    //关闭OLED显示
void driver_oled_Display_Fill(uint8_t Fill_Data);    //全屏填充
void driver_oled_Clear(void);    //清屏函数
void driver_oled_Clear_Part(uint8_t x1, uint8_t x2, uint8_t page1, uint8_t page2);    //部分清屏函数

/********************显示函数********************/

/**
 * @brief  在oled上显示整数
 * @details  在屏幕某个位置显示整数，整数过长时会省略
 * @param  x 列，取值范围为[0,127]
 * @param  page 页，取值范围为[0,7]
 * @param  number 要显示的整数
 * @param  fontSize 字体大小
 */
void driver_oled_Show_Num(uint8_t x, uint8_t page, int32_t number, uint8_t fontSize);

/**
 * @brief  在oled上显示整数，并在末尾补齐空格
 * @param x 列，取值范围为[0,127]
 * @param page 页，取值范围为[0,7]
 * @param number 要显示的整数
 * @param fontSize 字体大小
 * @param length 显示的总长度
 */
void driver_oled_Show_Num_With_Blank(uint8_t x, uint8_t page, int32_t number, uint8_t fontSize, uint8_t length);

/**
 * @brief  在oled上显示整数，并在整数前补0
 * @param x 列，取值范围为[0,127]
 * @param page 页，取值范围为[0,7]
 * @param number 要显示的整数
 * @param fontSize 字体大小
 * @param length 显示的总长度
 */
void driver_oled_Show_Num_With_Zero(uint8_t x, uint8_t page, int32_t number, uint8_t fontSize, uint8_t length);

/**
	* @brief  字符显示函数
	* @details  在指定位置显示一个字符,包括部分字符
	* @param  x:列
	* @param  page:页
	* @param  chr:要显示的字符
	* @param  fontSize:字体大小
	* @retval  NULL
	*/
void driver_oled_Show_Char(uint8_t x, uint8_t page, uint8_t chr, uint8_t fontSize);

/**
	* @brief  显示浮点数
	* @details  在屏幕某个位置显示浮点数，浮点数过长时会省略
	* @param  x:列
	* @param  page:页
	* @param  number:要显示的整数
	* @param  precision:精确到小数点后几位
	* @param  fontSize:字体大小
	* @retval  NULL
	*/
void driver_oled_Show_Float(uint8_t x, uint8_t page, float number, uint8_t precision, uint8_t fontSize);

/**
	* @brief  显示字符串
	* @details  在屏幕某个位置显示字符串，字符串过长时会省略
	* @param  x:列
	* @param  page:页
	* @param  chr:要显示的字符串
	* @param  fontSize:字体大小
	* @retval  NULL
	*/
void driver_oled_Show_String(uint8_t x, uint8_t page, char *chr, uint8_t fontSize);

/**
	* @brief  显示图片
	* @details  在屏幕某个位置显示字符串
	* @param  x1,x2:图片显示列范围
	* @param  page1,page2:图片显示页范围
	* @param  BMP:图片像素矩阵
	* @retval  NULL
	*/
void driver_oled_Draw_Bmp(uint8_t x1, uint8_t x2, uint8_t page1, uint8_t page2, const unsigned char BMP[]);


#endif
