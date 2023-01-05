/** 
 *  @brief	    调试相关任务
 *  @details    包含OLED显示参数，向匿名上位机发送数据，接收上位机调参指令功能
 *  @author     Harry-Qu
 *  @date       2022/11/2
 *  @version    1.1
 *  @par        日志
 *              1.0     |       实现调试相关任务
 *              1.1     |       新增PID调试数据存储功能
*/

#include "app_debug.h"

void app_debug_init(void) {
#if DEBUG_OLED_EN > 0
    app_oled_Init();
#endif

#if DEBUG_ANO_EN > 0

#endif


#if DEBUG_BT_EN > 0

    app_bluetooth_init();

#endif
}

void app_debug_task(void *args) {
    (void) args;

    while (1) {
        OS_TRACE_MARKER_START(3);

#if DEBUG_OLED_EN > 0
        app_oled_Show_Data();
#endif

#if DEBUG_PID_STORAGE_EN > 0
        app_debug_pid_storage();
#endif

#if DEBUG_ANO_EN > 0
        app_debug_ano();
#endif
        OS_TRACE_MARKER_STOP(3);

//        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        OSTimeDly(OS_TICKS(8));
    }
}