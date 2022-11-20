#ifndef OS_CPU_H
#define OS_CPU_H


typedef unsigned char BOOLEAN;
typedef unsigned char INT8U;
typedef signed char INT8S;
typedef unsigned short INT16U;
typedef signed short INT16S;
typedef unsigned int INT32U;
typedef signed int INT32S;
typedef float FP32;
typedef double FP64;
typedef unsigned int OS_CPU_SR;
typedef unsigned int OS_STK;

#define ALIGNED8 __attribute__ ((aligned(8)))

void OS_TASK_SW(void);

int OSTaskIdleHook2(void);

void OSStartHighRdy(void);

void OSIntCtxSw(void);

void OSCtxSw(void);

#endif
