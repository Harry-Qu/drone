/**
 *  @brief	    I2C SDK
 *  @details    普通,DMA方式发送,接收数据
 *  @author     Harry-Qu
 *  @date       2022/5/18
 *  @version    1.2.1
 *  @par        日志
 *              1.0     |       实现普通和DMA方式发送接收数据功能。
 *              1.1     |       新增从指定内存地址读取数据功能。
 *              1.2     |       降低与ucos-II的耦合，可在非操作系统环境下使用
 *              1.2.1   |       新增向I2C设备指定内存地址写入数据功能
 *                              新增接口为sdk_i2c_memory_write, sdk_i2c_memory_write_dma
*/

#ifndef SDK_I2C_H
#define SDK_I2C_H

#define SDK_I2C    //用于在其他文件中标识可以使用sdk_i2c

#include "main.h"

#define I2C_TOTAL_NUM 3

/**
 * 根据i2c编号获取对应的信号量指针
 * @param sem 信号量数组指针
 * @param hi2c I2C_HandleTypeDef结构体指针
 */
#define Get_I2C_Sem(sem, hi2c) (hi2c->Instance == I2C1)?(sem[0]):((hi2c->Instance == I2C2)?(sem[1]):(sem[2]))

/**
 * 初始化i2c dma相关变量功能
 * @details 使用SDK的DMA功能必须调用此函数
 *          必须在操作系统初始化后调用此函数
 */
void sdk_i2c_dma_init();

/**
 * 同步发送i2C数据
 * @param hi2c I2C_HandleTypeDef结构体
 * @param DevAddress 目的设备地址
 * @param pData 发送的数据
 * @param Size 发送数据大小
 * @param Timeout 超时时间
 * @return
 */
HAL_StatusTypeDef
sdk_i2c_transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);


/**
 * 异步发送i2C数据
 * @details 通过与操作系统的深入结合，在异步调用i2C发送时，操作系统将调度其他任务运行，将此任务挂起，直到发送完成后再将此任务调回就绪状态。
 * @param hi2c I2C_HandleTypeDef结构体
 * @param DevAddress 目的设备地址
 * @param pData 发送的数据
 * @param Size 发送数据大小
 * @param Timeout 超时时间(一定为0，其他情况暂未测试)
 * @param perr 错误代码
 * @return
 */
HAL_StatusTypeDef
sdk_i2c_transmit_dma(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout,
                     uint8_t *perr);

/**
 * 同步接收i2C数据
 * @param hi2c I2C_HandleTypeDef结构体
 * @param DevAddress 目的设备地址
 * @param pData 接收的数据
 * @param Size 接收数据大小
 * @param Timeout 超时时间
 * @return
 */
HAL_StatusTypeDef
sdk_i2c_receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * 异步接收i2C数据
 * @details 通过与操作系统的深入结合，在异步调用i2C接收时，操作系统将调度其他任务运行，将此任务挂起，直到接收完成后再将此任务调回就绪状态。
 * @param hi2c I2C_HandleTypeDef结构体
 * @param DevAddress 目的设备地址
 * @param pData 接收的数据
 * @param Size 接收数据大小
 * @param Timeout 超时时间(一定为0，其他情况暂未测试)
 * @param perr 错误代码
 * @return
 */
HAL_StatusTypeDef
sdk_i2c_receive_dma(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout,
                    uint8_t *perr);
/**
 * 从指定设备指定地址中读取数据信息（阻塞方式）
 * @param hi2c i2c句柄
 * @param DevAddress 设备地址
 * @param MemAddress 待读取的寄存器地址
 * @param MemAddSize 待读取的寄存器大小（一般是I2C_MEMADD_SIZE_8BIT，即1）
 * @param pData 数据地址
 * @param Size 数据大小
 * @param Timeout 超时时间
 * @return
 */
HAL_StatusTypeDef
sdk_i2c_memory_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                    uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * 从指定设备指定地址中读取数据信息（DMA方式）
 * @param hi2c i2c句柄
 * @param DevAddress 设备地址
 * @param MemAddress 待读取的寄存器地址
 * @param MemAddSize 待读取的寄存器大小（一般是I2C_MEMADD_SIZE_8BIT，即1）
 * @param pData 数据地址
 * @param Size 数据大小
 * @param Timeout 超时时间
 * @param perr 错误信息
 * @return
 */
HAL_StatusTypeDef
sdk_i2c_memory_read_dma(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                        uint8_t *pData, uint16_t Size, uint32_t Timeout,
                        uint8_t *perr);

/**
 * 向指定设备指定地址中写入数据信息（阻塞方式）
 * @param hi2c i2c句柄
 * @param DevAddress 设备地址
 * @param MemAddress 待读取的寄存器地址
 * @param MemAddSize 待读取的寄存器大小（一般是I2C_MEMADD_SIZE_8BIT，即1）
 * @param pData 数据地址
 * @param Size 数据大小
 * @param Timeout 超时时间
 * @return
 */
HAL_StatusTypeDef
sdk_i2c_memory_write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                     uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * 向指定设备指定地址中写入数据信息（DMA方式）
 * @param hi2c i2c句柄
 * @param DevAddress 设备地址
 * @param MemAddress 待读取的寄存器地址
 * @param MemAddSize 待读取的寄存器大小（一般是I2C_MEMADD_SIZE_8BIT，即1）
 * @param pData 数据地址
 * @param Size 数据大小
 * @param Timeout 超时时间
 * @param perr 错误信息
 * @return
 */
HAL_StatusTypeDef
sdk_i2c_memory_write_dma(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                         uint8_t *pData, uint16_t Size, uint32_t Timeout, uint8_t *perr);

#endif //SDK_I2C_H
