/**
 *  @brief	    串口SDK
 *  @details    普通,DMA方式发送数据，向指定串口注册DMA接收回调函数
 *  @author     Harry-Qu
 *  @date       2022/5/20
 *  @version    1.2
 *  @par        日志
 *              1.0     |       实现普通和DMA方式发送串口数据功能。
 *              1.1     |       实现向指定串口注册DMA接收回调函数功能
 *              1.2     |       降低与ucos-II的耦合，可在非操作系统环境下使用
*/

/**
 * 使用示例
 *
 * uint8_t dataBuffer[100]; //定义数据缓冲区
 *
 * //当UART发生空闲中断时，此函数将被SDK主动调用
 * void app_callback(UART_HandleTypeDef *huart, uint16_t length) {
 *      printf("%s\n",dataBuffer);
 * }
 *
 * int main(){
 *      ...
 *      OSInit();   //操作系统初始化
 *      sdk_usartdma_init();    //UART DMA相关变量初始化
 *      sdk_uart_register_receive_dma_callback(app_callback, &huart1, dataBuffer, 100); //注册回调函数
 *      while(1);
 * }
 */

#ifndef SDK_USART_H
#define SDK_USART_H

#define SDK_UART    //用于在其他文件中标识可以使用sdk_uart

#include "main.h"
#include "usart.h"

#define USART_TOTAL_NUM 6

/**
 * 根据usart编号获取对应的信号量指针
 * @param sem 信号量数组指针
 * @param huart UART_HandleTypeDef结构体指针
 */
#ifdef USART3
#define UART_ID(huart) ((huart)->Instance == USART1)?(0): \
                        ((huart)->Instance == USART2)?(1):\
                        ((huart)->Instance == USART3)?(2):(5)   //仅含有USART1,USART2,USART3,USART6情况
#else
#define UART_ID(huart) ((huart)->Instance == USART1)?(0): \
                        ((huart)->Instance == USART2)?(1):(5)
#endif

//关闭空闲中断，用于处理数据时互斥访问
#define SDK_UART_PAUSE_IDLE_IT(huart) __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE)
//清空空闲标志位并打开空闲中断，用于处理数据时互斥访问
#define SDK_UART_RESUME_IDLE_IT(huart) __HAL_UART_CLEAR_IDLEFLAG(huart);__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE)

//获取uart对应的信号量
#define Get_USART_Sem(sem, huart) ((huart)->Instance == USART1)?(sem[0]):(((huart)->Instance == USART2)?(sem[1]):(sem[2]))


typedef struct {
    UART_HandleTypeDef *huart; //Uart handle
    void (*callbackFunction)(UART_HandleTypeDef *huart, uint16_t length); //回调函数
    uint8_t *pData; //DMA接收存放数据位置
    uint8_t size; //DMA接收数据大小
} sdk_uart_receiver_dma_t; //uart DMA接收后回调函数相关信息

/**
 * UART DMA功能相关变量初始化
 * @details 使用UART DMA功能时必须进行初始化
 *          必须在操作系统初始化后再调用此函数
 */
void sdk_usartdma_init();

/**
 * sdk处理UART接收活动的回调函数。
 * @details 当UART产生空闲中断，会调用此函数，由此函数调用对应注册的回调函数进行数据处理，调用后会清空接收数据并准备接收下一次数据
 * @param huart Uart handle
 * @param Pos 该次接收的数据长度
 */
void sdk_uart_RxEventCallback(struct __UART_HandleTypeDef *huart, uint16_t Pos);

/**
 * 注册UART DMA接收信息后的处理回调函数。
 * @details 当uart开启了DMA接收消息后，通过该接口注册回调函数，并指定数据存放地址及数据长度
 * @param callbackFunction 空闲中断处理回调函数，当uart产生了空闲中断，会在中断中调用该回调函数。
 * @param huart Uart handle
 * @param pData DMA接收数据地址
 * @param size DMA接收数据最长长度
 */
void sdk_uart_register_receive_dma_callback(void (*callbackFunction)(UART_HandleTypeDef *huart, uint16_t length),
                                            UART_HandleTypeDef *huart,
                                            uint8_t *pData,
                                            uint8_t size);

/**
 * 以阻塞方式发送uart数据
 * @param huart Uart handle
 * @param pData 发送的数据
 * @param Size 发送的数据大小
 * @param Timeout 超时时间
 * @return 发送状态
 */
HAL_StatusTypeDef sdk_uart_transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * 通过DMA发送uart数据，必须调用sdk_usartdma_init进行初始化，发送后会进行任务切换
 * @param huart Uart handle
 * @param pData 发送的数据
 * @param Size 数据大小
 * @param Timeout 超时时间
 * @param perr 错误码
 * @return 发送状态
 */
HAL_StatusTypeDef
sdk_uart_transmit_dma(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout, uint8_t *perr);

#endif //SDK_USART_H
