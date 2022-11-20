# 四轴飞行器设计

## 项目介绍

一个基于STM32F4的四轴飞行器实现

## 开发环境及所需工具

### 软件环境

- MacOS 12 Monterey
- STM32CubeMX
- Clion 2021.3.4
- Makefile
- arm-none-eabi-gcc
- jlinkExe
- SystemView（可选）

### 硬件环境

- STM32 NUCLEO-F401RE

## 目录树结构

```
Code
├── Algorithm
│   ├── Inc
│   │   ├── AHRS.h
│   │   ├── filter.h
│   │   └── pid.h
│   └── Src
│       ├── AHRS.c
│       ├── filter.c
│       └── pid.c
├── App
│   ├── Inc
│   │   ├── app_attitude.h
│   │   ├── app_cfg.h
│   │   ├── app_control.h
│   │   ├── app_debug.h
│   │   ├── app_debug_ano.h
│   │   ├── app_debug_bluetooth.h
│   │   ├── app_debug_oled.h
│   │   └── app_manage.h
│   └── Src
│       ├── app_attitude.c
│       ├── app_control.c
│       ├── app_debug.c
│       ├── app_debug_ano.c
│       ├── app_debug_bluetooth.c
│       ├── app_debug_oled.c
│       └── app_manage.c
├── Bsp
│   ├── Inc
│   │   └── bsp_tim.h
│   └── Src
│       └── bsp_tim.c
├── Core
│   ├── Inc
│   │   ├── dataType.h
│   │   ├── dma.h
│   │   ├── gpio.h
│   │   ├── i2c.h
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   ├── stm32f4xx_it.h
│   │   ├── tim.h
│   │   └── usart.h
│   └── Src
│       ├── dma.c
│       ├── gpio.c
│       ├── i2c.c
│       ├── main.c
│       ├── stm32f4xx_hal_msp.c
│       ├── stm32f4xx_it.c
│       ├── system_stm32f4xx.c
│       ├── tim.c
│       └── usart.c
├── Drivers
│   ├── CMSIS
│   │   ├── DSP
│   │   │   ├── ...
│   │   ├── Device
│   │   │   └── ST
│   │   │       └── STM32F4xx
│   │   │           ├── Include
│   │   │           │   ├── stm32f401xe.h
│   │   │           │   ├── stm32f4xx.h
│   │   │           │   └── system_stm32f4xx.h
│   │   │           └── Source
│   │   │               └── Templates
│   │   └── Include
│   │       ├── cmsis_armcc.h
│   │       ├── ...
│   ├── Hardware
│   │   ├── Inc
│   │   │   ├── driver_at24c0x.h
│   │   │   ├── driver_dbus.h
│   │   │   ├── driver_gy86.h
│   │   │   ├── driver_motor.h
│   │   │   ├── driver_oled_4pin.h
│   │   │   ├── driver_oled_font.h
│   │   │   ├── driver_rgb.h
│   │   │   ├── driver_tfmini.h
│   │   │   └── driver_ultrasound.h
│   │   └── Src
│   │       ├── driver_at24c0x.c
│   │       ├── driver_dbus.c
│   │       ├── driver_gy86.c
│   │       ├── driver_motor.c
│   │       ├── driver_oled_4pin.c
│   │       ├── driver_rgb.c
│   │       ├── driver_tfmini.c
│   │       └── driver_ultrasound.c
│   └── STM32F4xx_HAL_Driver
│       ├── Inc
│       │   ├── Legacy
│       │   │   └── stm32_hal_legacy.h
│       │   ├── stm32f4xx_hal.h
│       │   ├── ...
│       └── Src
│           ├── stm32f4xx_hal.c
│           ├── ...
├── Makefile
├── SDK
│   ├── Inc
│   │   ├── sdk_ano.h
│   │   ├── sdk_i2c.h
│   │   ├── sdk_io.h
│   │   ├── sdk_math.h
│   │   ├── sdk_time.h
│   │   └── sdk_usart.h
│   └── Src
│       ├── sdk_ano.c
│       ├── sdk_i2c.c
│       ├── sdk_io.c
│       ├── sdk_math.c
│       ├── sdk_time.c
│       └── sdk_usart.c
├── STM32F401RETx_FLASH.ld
├── Systemview
│   ├── Config
│   │   ├── Cortex-M
│   │   │   └── SEGGER_SYSVIEW_Config_uCOSII.c
│   │   ├── Global.h
│   │   ├── SEGGER_RTT_Conf.h
│   │   ├── SEGGER_SYSVIEW_Conf.h
│   │   └── os_cfg_trace.h
│   ├── SEGGER
│   │   ├── SEGGER.h
│   │   ├── SEGGER_RTT.c
│   │   ├── SEGGER_RTT.h
│   │   ├── SEGGER_RTT_ASM_ARMv7M.s
│   │   ├── SEGGER_RTT_printf.c
│   │   ├── SEGGER_SYSVIEW.c
│   │   ├── SEGGER_SYSVIEW.h
│   │   ├── SEGGER_SYSVIEW_ConfDefaults.h
│   │   ├── SEGGER_SYSVIEW_Int.h
│   │   └── Syscalls
│   │       └── SEGGER_RTT_Syscalls_GCC.c
│   ├── SEGGER_SYSVIEW_uCOSII.c
│   ├── cpu_core.h
│   └── os_trace_events.h
├── Ucos
│   ├── os.h
│   ├── os_cfg.h
│   ├── os_core.c
│   ├── os_cpu.h
│   ├── os_cpu_a.asm
│   ├── os_cpu_c.c
│   ├── os_dbg_r.c
│   ├── os_flag.c
│   ├── os_mbox.c
│   ├── os_mem.c
│   ├── os_mutex.c
│   ├── os_q.c
│   ├── os_sem.c
│   ├── os_task.c
│   ├── os_time.c
│   ├── os_tmr.c
│   ├── os_trace.h
│   ├── ucos_ii.c
│   └── ucos_ii.h
├── drone-iii-401.ioc
├── startup_stm32f401xe.s
├── stm32f4_config.gdbinit
├── stm32f4_config.jlink
Doc
├── 飞行状态转换.md
├── 四轴飞行器总结报告-i.pdf
└── 四轴飞行器总结报告-ii.pdf

```

## 项目进度

### V1 基础软硬件

#### 硬件：

- [x] 配件选型
- [x] 转接板PCB绘制
- [x] PCB打样与组装
- [ ] 一体板绘制

#### 软件：

- [x] PWM脉冲输出
- [x] 配置并获取MPU6050数据
- [x] 配置并获取HMC5883L数据
- [x] 配置并获取MS5611数据
- [x] 串口数据收发
- [x] 获取并解析遥控器数据
- [x] OLED数据显示
- [x] 雷达测距模块
- [x] 模块集成

### V2 操作系统

- [x] 移植UcOS/II 操作系统
- [x] 移植SystemView任务级调试工具
- [x] 使用DMA进行数据收发
- [x] 重构工程，支持以任务级方式运行
- [x] 使用Makefile重构工程

### V3 飞控集成

- [x] 加速度、角速度、磁力计数据解析
- [x] 姿态解算-MadgWick
- [x] 姿态解算-Mahony
- [x] 低通滤波
- [x] PID控制算法
- [x] 上位机调参协议
- [x] DSP库数学优化
- [ ] 重构整套工程
- [ ] PID调参

