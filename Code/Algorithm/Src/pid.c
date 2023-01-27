/** 
 *  @brief	    pid
 *  @details    提供PID相关操作
 *  @author     Harry-Qu
 *  @date       2022/10/27
 *  @version    1.1
 *  @par        日志
 *              1.0     |       实现PID基础功能
 *              1.1     |       新增PID积分限幅参数（调试）修改功能
*/



#include "pid.h"
#include "sdk_math.h"


pid_type PID_Init(float kp, float ki, float kd, float integralLimit) {
    pid_type pid;
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.integralLimit = integralLimit;
    return pid;
}

float PID_calculate(pid_type *pid) {
    pid->error = pid->target - pid->current;
    pid->integral += pid->error;

    if (pid->integralLimit > 0) {
        LIMIT(pid->integral, -pid->integralLimit, pid->integralLimit);
    }
    pid->pOut = pid->kp * pid->error;
    pid->iOut = pid->ki * pid->integral;
    pid->dOut = pid->kd * (pid->error - pid->lastError);
    pid->out = pid->pOut + pid->iOut + pid->dOut;
    pid->lastError = pid->error;

    return pid->out;
}



void PID_setArg(pid_type *pid, int8_t argId, float value) {
    if (pid == NULL){
        return;
    }
    if (isnanf(value) || isinff(value)){
        return;
    }
    switch (argId) {
        case 0:
            pid->kp = value;
            break;
        case 1:
            pid->ki = value;
            break;
        case 2:
            pid->kd = value;
            break;
        case 3:
            pid->integralLimit = value;
            break;
        default:
            break;
    }
}



void PID_getArgString(pid_type *pid, char *strs) {
    if (pid == NULL){
        return;
    }
    sprintf(strs, "p=%.2f i=%.2f d=%.2f iL=%.2f", pid->kp, pid->ki, pid->kd, pid->integralLimit);
}

