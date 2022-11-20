/** 
 *  @brief	    AT24C0x驱动
 *  @details    支持读取固定某页的(256*8K)的不限类型不限长度的数据
 *  @author     Harry-Qu
 *  @date       2022/11/5
 *  @version    1.0
 *  @par        日志
*/

#ifndef DRIVER_AT24C0X_H
#define DRIVER_AT24C0X_H

#include "main.h"
#include "i2c.h"

#define AT24C0X_I2C hi2c1
#define AT24C0X_READ_ADDRESS 0xA1
#define AT24C0X_WRITE_ADDRESS 0xA0

/**
 * 从AT24C0X中读取数据
 * @param address 数据地址
 * @param data 读出的数据指针，单数据/数组
 * @param dataSize 数据大小，单数据大小*数据长度
 * @return 读取状态
 */
HAL_StatusTypeDef driver_at24c0x_readData(uint8_t address, void *data, uint32_t dataSize);

/**
 * 向AT24C0X中写入数据
 * @param address 数据地址
 * @param data 写入的数据指针，单数据/数组
 * @param dataSize 数据大小，单数据大小*数据长度
 * @return 写入状态
 */
HAL_StatusTypeDef driver_at24c0x_writeData(uint8_t address, void *data, uint32_t dataSize);

#endif //DRIVER_AT24C0X_H
