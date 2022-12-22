/** 
 *  @brief	    AT24C0x驱动
 *  @details    支持从AT24C0X指定页读写的不限类型不限长度的数据
 *  @author     Harry-Qu
 *  @date       2022/11/5
 *  @version    1.1.1
 *  @par        日志
 *              1.0     |   实现AT24C0x基本读写功能
 *              1.1     |   新增从指定页中读写数据的接口driver_at24c0x_writeData_page, driver_at24c0x_readData_page
 *                          修改部分接口返回类型为uint8_t
 *              1.1.1   |   为ROM读写增加数据切片功能，保证读写大量数据时的准确性
 *                          修改dataSize参数类型为uint8_t
*/

#ifndef DRIVER_AT24C0X_H
#define DRIVER_AT24C0X_H

#include "main.h"
#include "i2c.h"

#define AT24C0X_I2C hi2c3
#define AT24C0X_READ_ADDRESS 0xA1
#define AT24C0X_WRITE_ADDRESS 0xA0


/**
 * 向AT24C0X中写入数据
 * @param address 数据地址
 * @param data 写入的数据指针，单数据/数组
 * @param dataSize 数据大小，单数据大小*数据长度
 * @return 写入状态
 *         0: success
 *         1: fail
 */
uint8_t driver_at24c0x_writeData(uint8_t address, void *data, uint8_t dataSize);

/**
 * 从AT24C0X中读取数据
 * @param address 数据地址
 * @param data 读出的数据指针，单数据/数组
 * @param dataSize 数据大小，单数据大小*数据长度
 * @return 读取状态
 *         0: success
 *         1: fail
 */
uint8_t driver_at24c0x_readData(uint8_t address, void *data, uint8_t dataSize);

/**
 * 向AT24C0X指定页中写入数据
 * @param page 要写入的ROM分页
 * @param address 要写入的ROM开始地址
 * @param data 写入的数据指针，单数据/数组
 * @param dataSize 数据大小，单数据大小*数据长度
 * @return 写入状态
 *         0: success
 *         1: fail
 */
uint8_t driver_at24c0x_writeData_page(uint8_t page, uint8_t address, void *data, uint8_t dataSize);

/**
 * 从AT24C0X指定页中读取数据
 * @param page 要读取的ROM分页
 * @param address 要读取的ROM开始地址
 * @param data 读出的数据指针，单数据/数组
 * @param dataSize 数据大小，单数据大小*数据长度
 * @return 写入状态
 *         0: success
 *         1: fail
 */
uint8_t driver_at24c0x_readData_page(uint8_t page, uint8_t address, void *data, uint8_t dataSize);

#endif //DRIVER_AT24C0X_H
