/** 
 *  @brief	    oled驱动
 *  @details    基于SSD1306开发
 *  @author     Asn，Evan-GH, Harry-Qu
 *  @date       2019.11.5
 *  @version    3.4.1
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
				V3.4.1取消驱动代码的裁剪性
*/


#include <memory.h>
#include <stdio.h>
#include "driver_oled_4pin.h"
#include "driver_oled_font.h"
#include "sdk_i2c.h"
#include "app_cfg.h"


//OLED相关宏定义

#define MAX_COLUMN    128
#define MAX_ROW        64
#define MAX_PAGE 8
#define    BRIGHTNESS    0xEF
#define BLOCK_SIZE 16
#define    ABS(x)   ((x)>0?(x):-(x))
#define MIN(a, b) (((a)<(b))?(a):(b))

#define Horizontal
//OLED的显存
//存放格式如下
//       [col]
//[page0]0 1 2 3 ... 127	
//[page1]0 1 2 3 ... 127	
//[page2]0 1 2 3 ... 127	
//[page3]0 1 2 3 ... 127	
//[page4]0 1 2 3 ... 127	
//[page5]0 1 2 3 ... 127	
//[page6]0 1 2 3 ... 127	
//[page7]0 1 2 3 ... 127


/********************内部函数，发送数据、指令********************/
static void
driver_oled_Write_CDs(const unsigned char *datas, uint16_t length, enum COMMAND_AND_DATA cd);    //向OLED批量发送指令或数据
static void driver_oled_Write_CD(const unsigned char data, enum COMMAND_AND_DATA cd);    //向OLED单独发送指令或数据
static void driver_oled_Write_Cmds(const unsigned char *cmds, uint16_t length);    //向OLED批量发送指令
static void driver_oled_Write_Cmd(const unsigned char cmd);    //向OLED单独发送指令
static void driver_oled_Write_Datas(const unsigned char *datas, uint16_t length);    //向OLED批量发送数据
static void driver_oled_Write_Data(const unsigned char data);    //向OLED单独发送数据

/********************内部函数，设置显示范围********************/
static void driver_oled_Set_Page(uint8_t page);    //设置需写入像素数据的页数
static void driver_oled_Set_Row(uint8_t row);    //设置需写入像素数据的行数
static void driver_oled_Set_Column(uint8_t column);    //设置需写入像素数据的列数
static void driver_oled_Set_Pos(uint8_t x, uint8_t page);    //设置需写入像素数据的坐标
static void driver_oled_Set_Column_Area(uint8_t x1, uint8_t x2);    //设置需写入像素数据的指针移动列范围
static void driver_oled_Set_Page_Area(uint8_t page1, uint8_t page2);    //设置需写入像素数据的指针移动页范围
static void driver_oled_Set_Area(uint8_t x1, uint8_t x2, uint8_t page1, uint8_t page2);    //设置需写入像素数据的指针移动范围

/********************内部函数********************/
static void driver_oled_Show_Ellipsis(uint8_t x1, uint8_t x2, uint8_t page, uint8_t fontSize);    //添加省略号
static void driver_oled_Add_Char_Data(uint8_t *dest, uint8_t chr, uint8_t fontSize, uint8_t width);    //复制字符像素矩阵函数

/********************内部计算函数********************/
static uint8_t driver_oled_Cal_Font_Size(uint8_t fontSize);    //字体大小计算
static uint8_t driver_oled_Cal_Word_Width(uint8_t fontSize);    //字宽计算
static uint32_t driver_oled_pow(uint8_t m, uint8_t n);    //计算m^n
static uint8_t driver_oled_Get_Unsigned_Integer_Length(uint32_t number);    //获取整数位数

/**
	* @brief  向OLED批量发送指令或数据
	* @details  NULL
	* @param  datas 指令或数据串
	* @param  length 指令或数据串长度
	* @param  cd 发送类型（指令或数据），可选值:COMMAND,DATA
	* @retval  NULL
  **/
void driver_oled_Write_CDs(const unsigned char *datas, uint16_t length, enum COMMAND_AND_DATA cd) {
    /*	分块出锅时调试用
    uint16_t a=0;
    for (a=0;a<length;++a) _driver_oled_Write_CD(datas[a], cd);
    return;
    */

    uint8_t sendData[BLOCK_SIZE * 2];
    memset(sendData, 0x80 | (cd << 6), sizeof(sendData));
    uint16_t i = 0, k = 0, j = 0;

    for (k = 0; k <= length / BLOCK_SIZE; ++k) {    //分块
        j = BLOCK_SIZE > length - k * BLOCK_SIZE ? length - k * BLOCK_SIZE : BLOCK_SIZE;
        for (i = 0; i < j; ++i) {
            sendData[2 * i + 1] = datas[i + k * BLOCK_SIZE];
        }

#if I2C_DMA_EN > 0
        uint8_t err;
//        OS_TRACE_MARKER_START(5);
        sdk_i2c_transmit_dma(&DRIVER_OLED_I2C, DRIVER_OLED_ADDRESS << 1, sendData, j * 2, 100, &err);
//        OS_TRACE_MARKER_STOP(5);
#else
        OSSchedLock();
//        OS_TRACE_MARKER_START(5);
        HAL_StatusTypeDef status;
        status = HAL_I2C_Master_Transmit(&DRIVER_OLED_I2C, DRIVER_OLED_ADDRESS << 1, sendData, j * 2, 100);
//        printf("%d status:%d\n", HAL_GetTick(), status);
//        OS_TRACE_MARKER_STOP(5);
        OSSchedUnlock();
#endif
    }


}

/**
	* @brief  向OLED单独发送指令或数据
	* @details  NULL
	* @param  datas:指令或数据
	* @param  cd:发送类型（指令或数据），可选值:COMMAND,DATA
	* @retval  NULL
  **/
void driver_oled_Write_CD(const unsigned char data, enum COMMAND_AND_DATA cd) {
    uint8_t sendData[2] = {0x00 | (cd << 6), data};
#if I2C_DMA_EN > 0
    uint8_t err;
//    OS_TRACE_MARKER_START(5);
    sdk_i2c_transmit_dma(&DRIVER_OLED_I2C, DRIVER_OLED_ADDRESS << 1, sendData, 2, 100, &err);
//    OS_TRACE_MARKER_STOP(5);
#else
    OSSchedLock();
    HAL_I2C_Master_Transmit(&DRIVER_OLED_I2C, DRIVER_OLED_ADDRESS << 1, sendData, 2, 100);
    OSSchedUnlock();
#endif


}

/**
	* @brief  向OLED批量发送指令
	* @details  NULL
	* @param  cmds:指令串
	* @param  length:指令串长度
	* @retval  NULL
  **/
void driver_oled_Write_Cmds(const unsigned char *cmds, uint16_t length) {
    driver_oled_Write_CDs(cmds, length, COMMAND);
}

/**
	* @brief  向OLED单独发送指令
	* @details  NULL
	* @param  cmd:指令
	* @retval  NULL
  **/
void driver_oled_Write_Cmd(const unsigned char cmd) {
    driver_oled_Write_CD(cmd, COMMAND);
}

/**
	* @brief  向OLED批量发送数据
	* @details  NULL
	* @param  datas:数据串
	* @param  length:数据串长度
	* @retval  NULL
  **/
void driver_oled_Write_Datas(const unsigned char *datas, uint16_t length) {
    driver_oled_Write_CDs(datas, length, DATA);
}

/**
	* @brief  向OLED单独发送数据
	* @details  NULL
	* @param  data:数据
	* @retval  NULL
  **/
void driver_oled_Write_Data(const unsigned char data) {
    driver_oled_Write_CD(data, DATA);
}

/**
	* @brief  设置需写入像素数据的页数
	* @details  仅Page addressing mode可用，V3.2后不再使用此函数
	* @param  page:页数，取值0~7
	* @retval  NULL
  **/
void driver_oled_Set_Page(uint8_t page) {
    driver_oled_Write_Cmd(0xb0 + page);
}

/**
	* @brief  设置需写入像素数据的行数
	* @details  仅Page addressing mode可用，V3.2后不再使用此函数
	* @param  row:行数，取值0~63
	* @retval  NULL
  **/
void driver_oled_Set_Row(uint8_t row) {
    driver_oled_Write_Cmd(0x40 + row);
}

/**
	* @brief  设置需写入像素数据的列数
	* @details  仅Page addressing mode可用，V3.2后不再使用此函数
	* @param  column:列，取值0~127
	* @retval  NULL
  **/
void driver_oled_Set_Column(uint8_t column) {
    driver_oled_Write_Cmd(0x10 | (column >> 4));    //设置列地址高位
    driver_oled_Write_Cmd(0x00 | (column & 0x0f));   //设置列地址低位
}

/**
	* @brief  设置需写入像素数据的坐标
	* @details  仅Page addressing mode可用，V3.2后不再使用此函数
	* @param  x:列，取值0~127
	* @param  page:页，取值0~7
	* @retval  NULL
  **/
void driver_oled_Set_Pos(uint8_t x, uint8_t page) {
    driver_oled_Set_Column(x);
    driver_oled_Set_Page(page);
}

/**
	* @brief  设置需写入像素数据的指针移动列范围
	* @details  设置后，OLED的数据指针仅在列范围内移动
	* @param  x1:最低列，取值0~127
	* @param  x2:最高列，取值0~127，x1<=x2
	* @retval  NULL
  **/
void driver_oled_Set_Column_Area(uint8_t x1, uint8_t x2) {
    if (x1 > x2 || x2 >= MAX_COLUMN) {
        return;
    }
    uint8_t cmds[3] = {0x21, x1, x2};
    driver_oled_Write_Cmds(cmds, 3);
}

/**
	* @brief  设置需写入像素数据的指针移动页范围
	* @details  设置后，OLED的数据指针仅在页范围内移动
	* @param  page1:最小页，取值0~7
	* @param  page2:最大页，取值0~7，page1<=page2
	* @retval  NULL
  **/
void driver_oled_Set_Page_Area(uint8_t page1, uint8_t page2) {
    if (page1 > page2 || page2 >= MAX_PAGE) {
        return;
    }
    uint8_t cmds[3] = {0x22, page1, page2};
    driver_oled_Write_Cmds(cmds, 3);
}

/**
	* @brief  设置需写入像素数据的指针移动范围
	* @details  设置后，OLED的数据指针仅在范围内移动
	* @param  x1:最低列，取值0~127
	* @param  x2:最高列，取值0~127，x1<=x2
	* @param  page1:最小页，取值0~7
	* @param  page2:最大页，取值0~7，page1<=page2
	* @retval  NULL
  **/
void driver_oled_Set_Area(uint8_t x1, uint8_t x2, uint8_t page1, uint8_t page2) {
    if (x1 > x2 || x2 >= MAX_COLUMN || page1 > page2 || page2 >= MAX_PAGE) {
        return;
    }

    uint8_t cmds[6] = {0x21, x1, x2, 0x22, page1, page2};
    driver_oled_Write_Cmds(cmds, 6);
}

/**
	* @brief  开启OLED显示  
	* @details  打开屏幕的显示功能
	* @param  NULL
	* @retval  NULL
	**/
void driver_oled_Display_On(void) {
    uint8_t cmds[3] = {
            0X8D,    //SET DC-DC命令
            0X14,    //DC-DC ON
            0XAF    //DISPLAY ON
    };
    driver_oled_Write_Cmds(cmds, 3);
}

/**
	* @brief  关闭OLED显示  
	* @details  关闭屏幕的显示功能
	* @param  NULL
	* @retval  NULL
	**/
void driver_oled_Display_Off(void) {
    uint8_t cmds[3] = {
            0X8D,    //SET DC-DC命令
            0X10,    //DC-DC OFF
            0XAE    //DISPLAY OFF
    };
    driver_oled_Write_Cmds(cmds, 3);
}


/**
	* @brief  全屏填充
	* @details  用于对整个屏幕填充数据
	* @param  Fill_Data:你要填充的数据
	* @retval  NULL
	**/
void driver_oled_Display_Fill(uint8_t Fill_Data) {
    uint8_t fill_Data[MAX_COLUMN * MAX_PAGE];
    memset(fill_Data, Fill_Data, sizeof(fill_Data));

    driver_oled_Set_Area(0, MAX_COLUMN - 1, 0, MAX_PAGE - 1);
    driver_oled_Write_Datas(fill_Data, sizeof(fill_Data));
}

/**
	* @brief  清屏函数
	* @details  对屏幕填充0X00，整个屏幕是黑色的!和没点亮一样!!!	
	* @param NULL 
	* @retval  NULL
	**/
void driver_oled_Clear(void) {
    driver_oled_Display_Fill(0X00);
}

/**
	* @brief  部分清屏函数
	* @details  用于对屏幕某一部分进行清除操作，使视觉效果变好
	* @param  x1:最低列，取值0~127
	* @param  x2:最高列，取值0~127，x1<=x2
	* @param  page1:最小页，取值0~7
	* @param  page2:最大页，取值0~7，page1<=page2
	* @retval  NULL
	**/
void driver_oled_Clear_Part(uint8_t x1, uint8_t x2, uint8_t page1, uint8_t page2) {
    if (x1 > x2 || x2 >= MAX_COLUMN || page1 > page2 || page2 >= MAX_PAGE) {
        return;
    }
    uint8_t datas[(x2 - x1 + 1) * (page2 - page1 + 1)];
    memset(datas, 0x00, sizeof(datas));
    driver_oled_Set_Area(x1, x2, page1, page2);
    driver_oled_Write_Datas(datas, sizeof(datas));
}

/**
	* @brief  字体大小计算
	* @details  给定用户定义的字体大小，返回本程序支持的字体大小
	* @param fontSize:预期字体大小
	* @retval  实际支持的字体大小，支持值:8,16
	**/
uint8_t driver_oled_Cal_Font_Size(uint8_t fontSize) {
    if (fontSize <= 11) {
        return 8;
    } else {
        return 16;
    }
}

/**
	* @brief  字宽计算
	* @details  给定字体大小，返回显示字宽度
	* @param fontSize:字体大小
	* @retval  显示字宽度
	*/
uint8_t driver_oled_Cal_Word_Width(uint8_t fontSize) {
    if (fontSize <= 11) {
        return 6;
    } else {
        return 8;
    }
}

/**
	* @brief  添加省略号
	* @details  对于数据显示溢出限定范围的情况，在给定范围内添加省略号
	* @details  fontSize=8时，在第page页显示；fontSize=16时，在第page+1页显示；
	* @param  x1:开始显示位置的偏移地址（相对dest）
	* @param  x2:结束显示位置的偏移地址
	* @param  page:页数，取值0~7
	* @param  fontSize:字体大小，取值8,16
	* @retval  NULL
	**/
void
driver_oled_Add_Ellipsis_Data(uint8_t *dest, uint8_t x1, uint8_t x2, uint8_t page, uint8_t fontSize, uint8_t width) {
    x2 = (x2 >= MAX_COLUMN) ? MAX_COLUMN - 1 : x2;
    page = (page >= MAX_PAGE) ? MAX_PAGE - 1 : page;
    if (x1 > x2) {
        return;
    }
    uint8_t char_spacing = 2;    //设置几列显示一个省略号
    uint8_t i = 0;
    if (fontSize == 16 && page < MAX_PAGE - 1) {
        dest += width;
    }
    for (i = x1; i < x2; i += char_spacing) {
        dest[i] = 0x40;
    }
}

/**
	* @brief  显示省略号
	* @details  对于数据显示溢出限定范围的情况，在给定范围内显示省略号
	* @details  fontSize=8时，在第page页显示；fontSize=16时，在第page+1页显示；
	* @param  x1:最低列，取值0~127
	* @param  x2:最高列，取值0~127，x1<=x2
	* @param  page:页数，取值0~7
	* @param  fontSize:字体大小，取值8,16
	* @retval  NULL
	**/
void driver_oled_Show_Ellipsis(uint8_t x1, uint8_t x2, uint8_t page, uint8_t fontSize) {
    x2 = (x2 >= MAX_COLUMN) ? MAX_COLUMN - 1 : x2;
    page = (page >= MAX_PAGE) ? MAX_PAGE - 1 : page;
    if (x1 > x2) {
        return;
    }
    uint8_t char_spacing = 2;    //设置几列显示一个省略号
    uint8_t i = 0;
    page = (fontSize == 8 && page < 7) ? page : page + 1;

    uint8_t width = (x2 + 1 - x1) / char_spacing * char_spacing;//计算省略号的宽度，取2的倍数
    uint8_t datas[width];
    memset(datas, 0x00, sizeof(datas));
    for (i = 0; i < width; i += char_spacing) {
        datas[i] = 0x40;
    }

    driver_oled_Set_Area(x1, x1 + width - 1, page, page);
    driver_oled_Write_Datas(datas, width);
}

/**
	* @brief  复制字符像素矩阵函数
	* @details  将给定字符的像素数据复制到dest数组中
	* @param  dest:目标数组
	* @param  chr:待复制字符
	* @param  fontSize:字体大小
	* @param  width:dest宽度，对于size=16时用于换行
	* @retval  NULL
	**/
void driver_oled_Add_Char_Data(uint8_t *dest, uint8_t chr, uint8_t fontSize, uint8_t width) {
    uint8_t c = 0;
    c = chr - ' ';//得到偏移后的值
    switch (fontSize) {
        case 8:
            memcpy(dest, F6x8 + c * 6, 6);
            break;
        case 16:
            memcpy(dest, F8X16 + c * 16, 8);
            memcpy(dest + width, F8X16 + c * 16 + 8, 8);
            break;
    }
}

/**
	* @brief  获取整数位数
	* @details  计算所给整数的位数
	* @param  number:整数
	* @retval  整数位数
	*/
uint8_t driver_oled_Get_Unsigned_Integer_Length(uint32_t number) {
    if (number == 0) {
        return 1;
    }
    uint8_t length = 0;
    while (number > 0) {
        length++;
        number /= 10;
    }
    return length;
}

uint8_t driver_oled_Get_Integer_Length(int32_t number) {
    if (number < 0) {
        return driver_oled_Get_Unsigned_Integer_Length(-number) + 1;
    }
    return driver_oled_Get_Unsigned_Integer_Length(number);
}

/**
	* @brief  复制无符号整数像素矩阵函数
	* @details  将给定无符号整数的像素数据复制到dest数组中，需确保数据不会溢出dest
	* @param  dest:目标数组
	* @param  number:待复制整数
	* @param  fontSize:字体大小
	* @param  width:dest宽度，对于size=16时用于换行
	* @param  min_length:最小显示整数长度，当整数长度大于最小小时长度时，将前补0，常规值为1
	* @param  max_length:最大显示整数长度，当证书长度大于最大显示长度时，将显示高位，舍弃低位
	* @retval  length:显示整数的实际长度
	**/
uint16_t
driver_oled_Add_Unsigned_Integer_Data(uint8_t *dest, uint32_t number, uint8_t fontSize, uint8_t width,
                                      uint8_t min_length,
                                      uint8_t max_length) {
    uint8_t word_spacing = driver_oled_Cal_Word_Width(fontSize);
    int8_t length = 0;
    uint8_t num[11];
    while (number > 0) {
        num[length++] = number % 10;
        number /= 10;
    }
    if (max_length <= 0) return length;

    if (length < min_length) {
        //用于前补0
        uint8_t i = min_length - length;
        for (; i; --i) {
            driver_oled_Add_Char_Data(dest, '0', fontSize, width);
            dest += word_spacing;
        }
    }
    int8_t j = length - 1;
    while (j >= length - max_length && j >= 0) {
        driver_oled_Add_Char_Data(dest, num[j] + '0', fontSize, width);
        dest += word_spacing;
        j--;
    }
    return length;
}

/**
	* @brief  复制整数像素矩阵函数
	* @details  将给定整数的像素数据复制到dest数组中，需确保数据不会溢出dest
	* @param  dest:目标数组
	* @param  number:待复制整数
	* @param  fontSize:字体大小
	* @param  width:dest宽度，对于size=16时用于换行
	* @param  min_length:最小显示整数长度，当整数长度大于最小小时长度时，将前补0
	* @param  max_length:最大显示整数长度，当证书长度大于最大显示长度时，将显示高位，舍弃低位
	* @retval  NULL
	**/
uint16_t
driver_oled_Add_integer_Data(uint8_t *dest, int32_t number, uint8_t fontSize, uint8_t width, uint8_t min_length,
                             uint8_t max_length) {
    if (max_length <= 0) return driver_oled_Get_Integer_Length(number);
    uint8_t word_spacing = driver_oled_Cal_Word_Width(fontSize);
    if (number < 0) {
        driver_oled_Add_Char_Data(dest, '-', fontSize, width);
        return 1 + driver_oled_Add_Unsigned_Integer_Data(dest + word_spacing, -number, fontSize, width,
                                                         min_length > 0 ? min_length - 1 : 0, max_length - 1);
    } else {
        return driver_oled_Add_Unsigned_Integer_Data(dest, number, fontSize, width, min_length, max_length);
    }
}

/**
	* @brief  字符显示函数
	* @details  在指定位置显示一个字符,包括部分字符
	* @param  x:列
	* @param  page:页
	* @param  chr:要显示的字符
	* @param  fontSize:字体大小
	* @retval  NULL
	*/
void driver_oled_Show_Char(uint8_t x, uint8_t page, uint8_t chr, uint8_t fontSize) {
    fontSize = driver_oled_Cal_Font_Size(fontSize);
    uint8_t word_spacing = driver_oled_Cal_Word_Width(fontSize);
    if (x + word_spacing >= MAX_COLUMN || page >= MAX_PAGE) {
        return;    //溢出屏幕，不予显示
    }

    uint8_t c = chr - ' ';//得到偏移后的值
    if (x > MAX_COLUMN - 1) {
        x = 0;
        page = page + 2;
    }
    if (fontSize == 16) {
        driver_oled_Set_Area(x, x + word_spacing - 1, page, page + 1);
        driver_oled_Write_Datas(F8X16 + c * 16, 16);
    } else {
        driver_oled_Set_Area(x, x + word_spacing - 1, page, page);
        driver_oled_Write_Datas(F6x8 + c * 6, 6);
    }
}

/**
	* @brief  m^n函数
	* @details  算一个数的幂次方
	* @param  uint8_t m 底数,uint8_t n 幂数
	* @retval  uint32_t result计算结果
	*/
uint32_t driver_oled_pow(uint8_t m, uint8_t n) {
    uint32_t result = 1;
    while (n--)result *= m;
    return result;
}


void driver_oled_Show_Num(uint8_t x, uint8_t page, int32_t number, uint8_t fontSize) {
    fontSize = driver_oled_Cal_Font_Size(fontSize);
    uint8_t word_spacing = driver_oled_Cal_Word_Width(fontSize);
    if (x + word_spacing >= MAX_COLUMN || page >= MAX_PAGE) {
        return;    //溢出屏幕，不予显示
    }

    uint8_t numLength = driver_oled_Get_Integer_Length(number);
    uint8_t charWidth = (fontSize == 8) ? 6 : 8, charHeight = (fontSize == 8) ? 1 : 2;    //单个字符宽度与高度（按页计）
    uint8_t width = numLength * charWidth, height = charHeight;    //显示数字所需的宽度和高度
    uint8_t maxLength = (MAX_COLUMN - x) / charWidth;

    if (numLength > maxLength) {
        //判断是否会溢出屏幕
        width = MAX_COLUMN - x;

    }

    uint8_t showDatas[width * height];
    memset(showDatas, 0x00, sizeof(showDatas));

    driver_oled_Add_integer_Data(showDatas, number, fontSize, width, 1, maxLength);

    if (numLength > maxLength) {
        //确认有溢出
        driver_oled_Add_Ellipsis_Data(showDatas, maxLength * charWidth, width, page, fontSize, width);    //在数字后添加省略号
    }
    driver_oled_Set_Area(x, x + width - 1, page, page + height - 1);
    driver_oled_Write_Datas(showDatas, sizeof(showDatas));

}

void driver_oled_Show_Num_With_Blank(uint8_t x, uint8_t page, int32_t number, uint8_t fontSize, uint8_t length) {
    fontSize = driver_oled_Cal_Font_Size(fontSize);
    uint8_t word_spacing = driver_oled_Cal_Word_Width(fontSize);
    if (x + word_spacing >= MAX_COLUMN || page >= MAX_PAGE) {
        return;    //溢出屏幕，不予显示
    }

    uint8_t numLength = driver_oled_Get_Integer_Length(number);
    uint8_t charWidth = (fontSize == 8) ? 6 : 8, charHeight = (fontSize == 8) ? 1 : 2;    //单个字符宽度与高度（按页计）
    uint8_t width = numLength * charWidth, height = charHeight;    //显示数字所需的宽度和高度
    uint8_t maxLength = MIN((MAX_COLUMN - x) / charWidth, length);


    if (numLength > maxLength) {
        //判断是否会溢出屏幕
        width = MIN((MAX_COLUMN - x), length * charWidth);

    }

    uint8_t showDatas[width * height];
    memset(showDatas, 0x00, sizeof(showDatas));


    driver_oled_Add_integer_Data(showDatas, number, fontSize, width, 1, maxLength);
    if (numLength > maxLength) {
        //确认有溢出
        driver_oled_Add_Ellipsis_Data(showDatas, maxLength * charWidth, width, page, fontSize, width);    //在数字后添加省略号
    }


    driver_oled_Set_Area(x, x + width - 1, page, page + height - 1);
    driver_oled_Write_Datas(showDatas, sizeof(showDatas));


    if (numLength < maxLength) {
        driver_oled_Clear_Part(x + charWidth * numLength, x + charWidth * maxLength - 1,
                               page, page + charHeight - 1);
    }

}

void driver_oled_Show_Num_With_Zero(uint8_t x, uint8_t page, int32_t number, uint8_t fontSize, uint8_t length) {
    fontSize = driver_oled_Cal_Font_Size(fontSize);
    uint8_t word_spacing = driver_oled_Cal_Word_Width(fontSize);
    if (x + word_spacing >= MAX_COLUMN || page >= MAX_PAGE) {
        return;    //溢出屏幕，不予显示
    }

    uint8_t numLength = driver_oled_Get_Integer_Length(number);
    uint8_t charWidth = (fontSize == 8) ? 6 : 8, charHeight = (fontSize == 8) ? 1 : 2;    //单个字符宽度与高度（按页计）
    uint8_t width = numLength * charWidth, height = charHeight;    //显示数字所需的宽度和高度
    uint8_t maxLength = MIN((MAX_COLUMN - x) / charWidth, length), zeroNum = 0;

    if (numLength > maxLength) {
        //判断会溢出屏幕
        width = MIN((MAX_COLUMN - x), length * charWidth);
    } else if (numLength < length) {
        //需要前补0
        width = length * charWidth;
        zeroNum = MIN(length - numLength, maxLength - numLength);
    }

    uint8_t showDatas[width * height];
    uint8_t charPos = 0;
    memset(showDatas, 0x00, sizeof(showDatas));

    if (number < 0) {
        driver_oled_Add_Char_Data(showDatas, '-', fontSize, width);
        charPos++;
        number = -number;
    }

    if (zeroNum) {
        //需要前补0
        uint8_t i = 0;
        for (; i < zeroNum; ++i) {
            driver_oled_Add_Char_Data(showDatas + charWidth * (charPos + i), '0', fontSize, width);
        }
        charPos += zeroNum;
    }

    driver_oled_Add_integer_Data(showDatas + charWidth * charPos, number, fontSize, width, 0, maxLength);
    if (numLength > maxLength) {
        //确认有溢出
        driver_oled_Add_Ellipsis_Data(showDatas, maxLength * charWidth, width, page, fontSize, width);    //在数字后添加省略号
    }

    driver_oled_Set_Area(x, x + width - 1, page, page + charHeight-1);
    driver_oled_Write_Datas(showDatas, sizeof(showDatas));

    //driver_oled_Show_Num()
}

void driver_oled_Show_Float(uint8_t x, uint8_t page, float number, uint8_t precision, uint8_t fontSize) {

    fontSize = driver_oled_Cal_Font_Size(fontSize);
    uint8_t word_spacing = driver_oled_Cal_Word_Width(fontSize);
    if (x + word_spacing >= MAX_COLUMN || page >= MAX_PAGE) {
        return;    //溢出屏幕，不予显示
    }

    if (number < 0) {
        driver_oled_Show_Char(x, page, '-', fontSize);
        driver_oled_Show_Float(x + word_spacing, page, -number, precision, fontSize);
        return;
    }

    int32_t integer;
    float decimal;
    uint32_t decimal_integer;
    uint32_t pow = driver_oled_pow(10, precision);

    integer = (int32_t) floorf(number);
    decimal = number - (float) integer;
    decimal_integer = (uint32_t) roundf(decimal * pow);
    //driver_oled_Show_Num(100,page,decimal_integer, 8);

    if (decimal_integer >= pow) {
        //处理小数部分进位会影响到整数部分的情况。
        integer += decimal_integer / pow;
        decimal_integer %= pow;
    }

    uint8_t integer_length = driver_oled_Get_Unsigned_Integer_Length(integer);

    driver_oled_Show_Num(x, page, integer, fontSize);
    if (precision > 0) {
        driver_oled_Show_Char(x + word_spacing * integer_length, page, '.', fontSize);
        uint8_t zero_num = precision - driver_oled_Get_Unsigned_Integer_Length(decimal_integer);
        for (uint8_t i = 1; i <= zero_num; ++i) {
            driver_oled_Show_Char(x + word_spacing * (integer_length + i), page, '0', fontSize);
        }

        driver_oled_Show_Num(x + word_spacing * (integer_length + 1 + zero_num), page, decimal_integer, fontSize);
    }

}

void driver_oled_Show_Float_With_Blank(uint8_t x, uint8_t page, float number, uint8_t precision, uint8_t fontSize) {

    fontSize = driver_oled_Cal_Font_Size(fontSize);
    uint8_t word_spacing = driver_oled_Cal_Word_Width(fontSize);
    if (x + word_spacing >= MAX_COLUMN || page >= MAX_PAGE) {
        return;    //溢出屏幕，不予显示
    }

    if (number < 0) {
        driver_oled_Show_Char(x, page, '-', fontSize);
        driver_oled_Show_Float_With_Blank(x + word_spacing, page, -number, precision, fontSize);
        return;
    }

    int32_t integer;
    float decimal;
    uint32_t decimal_integer;
    uint32_t pow = driver_oled_pow(10, precision);

    integer = (int32_t) floorf(number);
    decimal = number - (float) integer;
    decimal_integer = (uint32_t) roundf(decimal * pow);
    //driver_oled_Show_Num(100,page,decimal_integer, 8);

    if (decimal_integer >= pow) {
        //处理小数部分进位会影响到整数部分的情况。
        integer += decimal_integer / pow;
        decimal_integer %= pow;
    }

    uint8_t integer_length = driver_oled_Get_Unsigned_Integer_Length(integer);

    driver_oled_Show_Num(x, page, integer, fontSize);
    if (precision > 0) {
        driver_oled_Show_Char(x + word_spacing * integer_length, page, '.', fontSize);
        uint8_t zero_num = precision - driver_oled_Get_Unsigned_Integer_Length(decimal_integer);
        for (uint8_t i = 1; i <= zero_num; ++i) {
            driver_oled_Show_Char(x + word_spacing * (integer_length + i), page, '0', fontSize);
        }

        driver_oled_Show_Num(x + word_spacing * (integer_length + 1 + zero_num), page, decimal_integer, fontSize);
    }

}

void driver_oled_Show_String(uint8_t x, uint8_t page, char *chr, uint8_t fontSize) {
    fontSize = driver_oled_Cal_Font_Size(fontSize);

    uint8_t length = strlen(chr);
    uint8_t charWidth = (fontSize == 8) ? 6 : 8, charHeight = (fontSize == 8) ? 1 : 2;
    uint8_t width = length * charWidth, height = charHeight;
    if (width + x >= MAX_COLUMN) {
        length = (MAX_COLUMN - x) / charWidth;    //如果数字过大会溢出屏幕，则只显示数字的高位。
        width = charWidth * length;    //新的待显示宽度
        driver_oled_Show_Ellipsis(x + width, MAX_COLUMN, page, fontSize);    //在数字后添加省略号
    }
    uint8_t showDatas[charWidth * charHeight * length];
    uint8_t j = 0;

    while (j < length) {
        driver_oled_Add_Char_Data(showDatas + charWidth * j, chr[j], fontSize, width);
        j++;
    }

    driver_oled_Set_Area(x, x + width - 1, page, page + height - 1);
    driver_oled_Write_Datas(showDatas, sizeof(showDatas));

}

void driver_oled_Draw_Bmp(uint8_t x1, uint8_t x2, uint8_t page1, uint8_t page2, const unsigned char BMP[]) {
    driver_oled_Set_Area(x1, x2, page1, page2);
    driver_oled_Write_Datas(BMP, (x2 - x1 + 1) * (page2 - page1 + 1));
}


/**
* @brief  OLED初始化
* @details  对整个屏幕做初始化
* @param  NULL
* @retval  NULL
*/
void driver_oled_Init(void) {
    const unsigned char OLED_init_cmd[30] =
            {

                    /*0xae,0X00,0X10,0x40,0X81,0XCF,0xff,0xa1,0xa4,
                    0xA6,0xc8,0xa8,0x3F,0xd5,0x80,0xd3,0x00,0XDA,0X12,
                    0x8d,0x14,0xdb,0x40,0X20,0X02,0xd9,0xf1,0xAF*/



                    0xAE,//关闭显示
                    0xD5,//设置时钟分频因子,震荡频率
                    0x80,  //[3:0],分频因子;[7:4],震荡频率

                    0xA8,//设置驱动路数
                    0X3F,//默认0X3F(1/64)
                    0xD3,//设置显示偏移
                    0X00,//默认为0
                    0x40,//设置显示开始行 [5:0],行数.
                    0x8D,//电荷泵设置
                    0x14,//bit2，开启/关闭
                    0x20,//设置内存地址模式
                    0x00,//[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
                    0xA1,//段重定义设置,bit0:0,0->0;1,0->127;!!!
                    0xC8,//设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
                    //0x29,
                    //0x00,0x00,0x01,0x07,0x00,

                    0x26,
                    0x00, 0x00, 0x01, 0x07,


                    0xDA,//设置COM硬件引脚配置
                    0x12,//[5:4]配置
                    0x81,//对比度设置
                    BRIGHTNESS,//1~255;默认0X7F (亮度设置,越大越亮)
                    0xD9,//设置预充电周期
                    0xf1,//[3:0],PHASE 1;[7:4],PHASE 2;
                    0xDB,//设置VCOMH 电压倍率
                    0x30,//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
                    0xA4,//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
                    0xA6,//设置显示方式;bit0:1,反相显示;0,正常显示
                    0xAF,//开启显示
            };
    driver_oled_Write_Cmds(OLED_init_cmd, sizeof(OLED_init_cmd));
    driver_oled_Clear();
//    printf("oled init.\n");
}



//卸载宏定义

#undef MAX_COLUMN
#undef MAX_ROW
#undef MAX_PAGE
#undef BRIGHTNESS
#undef ABS
#undef BLOCK_SIZE
#undef Horizontal

