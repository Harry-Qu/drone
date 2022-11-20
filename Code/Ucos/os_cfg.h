#ifndef OS_CFG_H
#define OS_CFG_H

#include "os_cpu.h"
#include "app_cfg.h"

#define OS_TIME_CHAIN     //是否使用差分时间链
#define OS_TICKS_PER_SEC 1000u

#define OS_TICKS(ms) (OS_TICKS_PER_SEC * ((INT32U) ms + 500uL / OS_TICKS_PER_SEC) / 1000uL)


#define OS_LOWEST_PRIO    63u
#define OS_TASK_IDLE_STK_SIZE    256u
#define OS_MAX_TASKS    57u

#define OS_FLAG_EN 0u
#define OS_MAX_FLAGS 50u

#define OS_FLAGS_NBITS 32u

#define OS_FLAG_WAIT_CLR_EN 0u
#define OS_FLAG_ACCEPT_EN  0u
#define OS_FLAG_DEL_EN    0u
#define OS_FLAG_NAME_EN   0u
#define OS_FLAG_QUERY_EN  0u
#define OS_MBOX_EN 0u
#define OS_MBOX_ACCEPT_EN 0u
#define OS_MBOX_DEL_EN 0u
#define OS_MBOX_PEND_ABORT_EN 0u
#define OS_MBOX_POST_EN 0u
#define OS_MBOX_POST_OPT_EN 0u
#define OS_MBOX_QUERY_EN 0u

#define OS_MEM_EN 1u
#define OS_MAX_MEM_PART 100u
#define OS_MEM_NAME_EN 0u
#define OS_MEM_QUERY_EN 0u

#define OS_MUTEX_EN 1u
#define OS_MUTEX_ACCEPT_EN 1u
#define OS_MUTEX_DEL_EN 0u
#define OS_MUTEX_QUERY_EN 1u

#define OS_Q_EN 0u
#define OS_Q_POST_EN 0u
#define OS_MAX_QS 100u
#define OS_Q_ACCEPT_EN 0u
#define OS_Q_DEL_EN 0u
#define OS_Q_PEND_ABORT_EN 0u
#define OS_Q_FLUSH_EN 0u
#define OS_Q_POST_FRONT_EN 0u
#define OS_Q_POST_OPT_EN 0u
#define OS_Q_QUERY_EN 0u

#define OS_SEM_EN 1u //Semaphore management is required (set OS_SEM_EN to 1) when enabling Timer Management
#define OS_SEM_ACCEPT_EN 0u
#define OS_SEM_DEL_EN 0u
#define OS_SEM_PEND_ABORT_EN 0u
#define OS_SEM_QUERY_EN 1u
#define OS_SEM_SET_EN 1u
#define OS_TASK_STAT_EN 0u
#define OS_TASK_STAT_STK_CHK_EN 0u
#define OS_TASK_CHANGE_PRIO_EN 0u
#define OS_TASK_CREATE_EN 1u
#define OS_TASK_CREATE_EXT_EN 1u
#define OS_TASK_DEL_EN 0u
#define OS_TASK_NAME_EN 1u
#define OS_TASK_SUSPEND_EN 0u
#define OS_TASK_QUERY_EN 0u

#define OS_TASK_STAT_STK_SIZE 100u
#define OS_TASK_REG_TBL_SIZE 0u



#define OS_TIME_DLY_RESUME_EN 1u
#define OS_TIME_GET_SET_EN 1u
#define OS_TIME_DLY_HMSM_EN 1u

#define OS_TMR_EN 1u                 //OS_CFG.H, Missing OS_TMR_EN: When (1) enables code generation for Timer Management"


#define OS_TMR_CFG_MAX  30u           //Determines the total number of timers in an application (2 .. 65500)
#define OS_TMR_CFG_WHEEL_SIZE 100u      //Sets the size of the timer wheel (1 .. 1023), OS_CFG.H, OS_TMR_CFG_WHEEL_SIZE should be between 2 and 1024

#define OS_TMR_CFG_NAME_EN 1u
#define OS_TMR_CFG_TICKS_PER_SEC 1000u  //Determines the rate at which the timer management task will run (Hz)
#define OS_TASK_TMR_STK_SIZE  2048u      //the size of the Timer Task's stack

#define OS_ARG_CHK_EN 1u   //Enable (1) or Disable (0) argument checking
#define OS_CPU_HOOKS_EN 1  //uC/OS-II hooks are found in the processor port files when 1
#define OS_APP_HOOKS_EN 0u //
#define OS_DEBUG_EN 0u
#define OS_MAX_EVENTS 63  // OS_MAX_EVENTS must be <= 65500
#define OS_SCHED_LOCK_EN 1u //Include code for OSSchedLock() and OSSchedUnlock()

#define OS_EVENT_MULTI_EN 0
#define OS_TASK_PROFILE_EN 0
#define OS_TASK_SW_HOOK_EN 0
#define OS_TICK_STEP_EN 0
#define OS_TIME_TICK_HOOK_EN 0

#define OS_TASK_TMR_PRIO 5u
#define OS_STK_GROWTH 1u
#define OS_CRITICAL_METHOD 3u

extern OS_CPU_SR OS_CPU_SR_Save(void);

extern void OS_CPU_RestoreSR(OS_CPU_SR cpu_sr);

#define OS_ENTER_CRITICAL() {cpu_sr = OS_CPU_SR_Save();}
#define OS_EXIT_CRITICAL() {OS_CPU_RestoreSR(cpu_sr);}

#define OS_EVENT_NAME_EN 1u

#define SAFETY_CRITICAL_RELEASE



#if OS_TICKS_PER_SEC > 1000u
#error "OS_TICKS_PER_SEC must be smaller than 1000 in this system."
#endif

#if (defined(OS_TRACE_EN)) && OS_TRACE_EN > 0
#define OS_TRACE_MARKER_START(MarkerId)     SEGGER_SYSVIEW_MarkStart(MarkerId)
#define OS_TRACE_MARKER_STOP(MarkerId)      SEGGER_SYSVIEW_MarkStop(MarkerId)
#define OS_TRACE_MARKER(MarkerId)           SEGGER_SYSVIEW_Mark(MarkerId)
#else
#define OS_TRACE_MARKER_START(MarkerId)
#define OS_TRACE_MARKER_STOP(MarkerId)
#define OS_TRACE_MARKER(MarkerId)
#endif

#endif
