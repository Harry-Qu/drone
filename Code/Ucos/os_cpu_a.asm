    .EXTERN  OSRunning  //; External references
    .EXTERN  OSPrioCur
    .EXTERN  OSPrioHighRdy
    .EXTERN  OSTCBCur
    .EXTERN  OSTCBHighRdy
    .EXTERN  OSIntExit
    .EXTERN  OSTaskSwHook
    .EXTERN  OS_CPU_ExceptStkBase 
    .EXTERN  OS_KA_BASEPRI_Boundary

#include "app_cfg.h"

#if (defined OS_TRACE_EN) && (OS_TRACE_EN>0u)
	.EXTERN  SYSVIEW_TaskSwitchedIn
#endif



    .global OSCtxSw
    .global OSIntCtxSw
    .global PendSV_Handler
	.global	OS_CPU_SR_Save
	.global	OS_CPU_RestoreSR
	.global OSStartHighRdy



#define NVIC_INT_CTRL 0xE000ED04
#define NVIC_SYSPRI14 0xE000ED22
#define NVIC_PENDSV_PRI 0xFFFF
#define NVIC_PENDSVSET 0x10000000
#define EXC_RETURN 0xFFFFFFFD
#define BASEPRI_VALUE 0x00000010


    .section  .text.CODE,"ax",%progbits
    .syntax unified
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .thumb



.thumb_func
PendSV_Handler:
	CPSID I

	MRS r0,PSP
	CBZ	r0,PendSV_Handler_Nosave    //first task

    AND R1, lr, #0x10
    CBNZ R1,NO_STR_FP
    VSTMDB.32  r0!, {S16-S31}
NO_STR_FP:
    STMFD r0!,{r4-r11, lr}

	LDR r1,=OSTCBCur
	LDR	r2,[r1]
	STR r0,[r2]

	B PendSV_Handler_Nosave

.thumb_func
PendSV_Handler_Nosave:

	LDR	R0,=OSPrioCur
	LDR	R1,=OSPrioHighRdy
	LDRB	R2,[R1]
	STRB	R2,[R0]

	LDR	R2,=OSTCBCur
	LDR	R1,=OSTCBHighRdy
	LDR	R0,[R1]
	STR	R0,[R2]

	LDR	R0,[R0]

	LDMFD	r0!,{r4-r11, lr}

    AND R1, lr, #0x10
    CBNZ R1,NO_LDR_FP
    VLDMIA.32  r0!, {S16-S31}
NO_LDR_FP:
	MSR		PSP,r0

    PUSH {LR}
	BL OSTaskSwHook
    POP {LR}

	CPSIE I
	BX LR
	NOP
	




OSStartHighRdy:
    LDR R0,=OSRunning
    MOV R1,#1
    STRB R1,[R0]

    LDR R0,=OSTCBCur
    MOV R1,#0
    STR R1,[R0]



OSCtxSw:
OSIntCtxSw:
    LDR     R0, =NVIC_INT_CTRL
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR



OS_CPU_SR_Save:
	MRS	R0,PRIMASK
	CPSID I
	BX	LR
	NOP



OS_CPU_RestoreSR:
	MSR	PRIMASK,R0
	BX	LR
	NOP



.END