/** 
 *  @brief	    pid
 *  @details    提供PID相关操作
 *  @author     Harry-Qu
 *  @date       2022/10/27
 *  @version    1.1
 *  @par        日志
 *              1.0     |       实现PID基础功能
 *              1.1     |       新增PID积分限幅参数（调试）修改功能
 *
*/

#ifndef GY86_PID_H
#define GY86_PID_H

#include "main.h"


typedef struct {
    float kp, ki, kd;
    float integralLimit;    //积分限幅
    float error;    //误差值
    float current;  //当前值
    float target;   //期望值
    float lastError;    //上次的误差值
    float integral; //积分值
    float out;  //输出值

    float pOut,iOut, dOut;
} pid_type;

#define pid_setTarget(pid, value) pid.target = (value)
#define pid_setCurrent(pid, value) pid.current = (value)

/**
 * PID数据初始化
 * @param kp 比例值kp
 * @param ki 积分比例ki
 * @param kd 微分比例kd
 * @param integralLimit 积分限幅（为0则不使用）
 * @return 初始化参数后的PID结构体
 */
pid_type PID_Init(float kp, float ki, float kd, float integralLimit);

/**
 * PID计算输出值
 * @param pid pid结构体
 * @return 输出值
 */
float PID_calculate(pid_type *pid);

/**
 * 设置PID参数，调试用
 * @param pid pid结构体
 * @param argId 参数Id
 *              0=kp; 1=ki; 2=kd
 * @param value
 */
void PID_setArg(pid_type *pid, int8_t argId, float value);

/**
 * 获取pid的参数（字符串形式）,调试用
 * @param pid pid结构体
 * @param strs 输出字符串
 */
void PID_getArgString(pid_type *pid, char* strs);

#endif //GY86_PID_H
