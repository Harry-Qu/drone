/** 
 *  @brief	    AT24C0x驱动
 *  @details    支持从AT24C0X指定页读写的不限类型不限长度的数据
 *  @author     Harry-Qu
 *  @date       2022/11/5
 *  @version    1.1
 *  @par        日志
 *              1.0     |   实现AT24C0x基本读写功能
 *              1.1     |   新增从指定页中读写数据的接口driver_at24c0x_writeData_page, driver_at24c0x_readData_page
 *                          修改部分接口返回类型为uint8_t
 *
 * @code 代码示例:
 * {
 *      uint8_t data = 0x18;
 *      driver_at24c0x_writeData(0x00, &data, sizeof(data));    //向0x00地址写入1字节数据0x18
 * }
 *
 * {
 *      uint8_t data;
 *      driver_at24c0x_readData(0x00, &data, sizeof(data));    //从0x00地址读取1字节数据
 * }
 *
 * {
 *      float data[3] = {-1.0f, 2.5f, 3.14f};
 *      driver_at24c0x_writeData_page(1, 0x00, &data, sizeof(data));    //向0x00地址写入3个float数据，共12字节
 * }
 *
 * {
 *      float data[3];
 *      driver_at24c0x_readData_page(1, 0x00, &data, sizeof(data));    //从0x00地址读取3个float数据，共12字节
 * }
 */

#include "driver_at24c0x.h"

static uint32_t at24c0x_LastVisitTick = 0;

/**
 * 读写之间需间隔5ms
 */
static void driver_at24c0x_checkDeltaTime(void) {
    if (at24c0x_LastVisitTick) {
        while (HAL_GetTick() - at24c0x_LastVisitTick < 5);
        //注意，此处等待时间可能会影响程序执行效率!!!!
    }
}

/**
 * 记录上一次读写时间
 */
static void driver_at24c0x_RecordVisitTime(void) {
    at24c0x_LastVisitTick = HAL_GetTick();
}

/**
 * 向指定内存地址写入数据
 * @param address 内存地址
 * @param pData 要写入的数据
 * @param Size 数据大小(字节)
 * @param Timeout 超时时间
 * @return 写入状态
 *         0: success
 *         1: fail
 */
static uint8_t
driver_at24c0x_memWrite(uint8_t page, uint8_t address, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
    uint8_t devAddress = AT24C0X_WRITE_ADDRESS | (page << 1);

    driver_at24c0x_checkDeltaTime();

    status = HAL_I2C_Mem_Write(&AT24C0X_I2C, devAddress, address, I2C_MEMADD_SIZE_8BIT, pData, Size,
                               Timeout);
    driver_at24c0x_RecordVisitTime();
    if (status > 0) {
        return 1;
    }
    return 0;
}

/**
 * 从指定内存地址读取数据
 * @param address 内存地址
 * @param pData 读出的数据
 * @param Size 数据大小(字节)
 * @param Timeout 超时时间
 * @return 读取状态
 *         0: success
 *         1: fail
 */
static uint8_t
driver_at24c0x_memRead(uint8_t page, uint8_t address, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status;
    uint8_t devAddress = AT24C0X_WRITE_ADDRESS | (page << 1);

    driver_at24c0x_checkDeltaTime();

    status = HAL_I2C_Mem_Read(&AT24C0X_I2C, devAddress, address, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
    driver_at24c0x_RecordVisitTime();
    if (status > 0) {
        return 1;
    }
    return 0;
}

uint8_t driver_at24c0x_writeData(uint8_t address, void *data, uint32_t dataSize) {
    HAL_StatusTypeDef status;
    status = driver_at24c0x_memWrite(0, address, data, dataSize, 10);
    return status;
}

uint8_t driver_at24c0x_readData(uint8_t address, void *data, uint32_t dataSize) {
    HAL_StatusTypeDef status;
    status = driver_at24c0x_memRead(0, address, data, dataSize, 10);
    return status;
}

uint8_t driver_at24c0x_writeData_page(uint8_t page, uint8_t address, void *data, uint32_t dataSize) {
    HAL_StatusTypeDef status;
    status = driver_at24c0x_memWrite(page, address, data, dataSize, 10);
    return status;
}

uint8_t driver_at24c0x_readData_page(uint8_t page, uint8_t address, void *data, uint32_t dataSize) {
    HAL_StatusTypeDef status;
    status = driver_at24c0x_memRead(page, address, data, dataSize, 10);
    return status;
}
