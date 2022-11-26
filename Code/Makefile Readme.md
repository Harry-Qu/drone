# Makefile介绍

本Makefile对CubeMX生成的Makefile做了修改，主要修改如下：

```makefile
DEVICE = STM32F401RE
# 用于指定芯片型号，用于JlinkExe烧录命令 -device $(DEVICE)

JLINK_CONFIG_FILENAME = stm32f4_config.jlink
# 用于指定Jlink配置文件，执行make download时会自动生成此文件

C_SOURCES_PATH = App/Src
# 该变量用于指定需要编译的C程序路径，该路径下所有.c程序都会被添加到C_SOURCES中，无需手动逐一添加

ASM_SOURCES = Ucos/os_cpu_a.asm
# 该变量用于指定需要编译的asm程序路径

LDFLAGS += -u _printf_float
# 添加该指令用于printf/sprintf格式化浮点数

CLEAR_OUTPUT = $(shell rm build/output;)
CHECK_COMPILE = $(shell if [ -f build/output ];then if [ "`cat build/output | grep OK`" != "" ];then echo "\033[32m编译成功\033[0m"; else echo "\033[31m编译失败\033[0m";fi;else echo "文件未更新";fi)
CHECK_DOWNLOAD = $(shell if [ "`cat build/output | grep 'Contents already match'`" != "" ];then echo "\033[32m已烧写\033[0m"; else if [ "`cat build/output | grep O.K.`" != "" ];then echo "\033[32m烧写成功\033[0m";else echo "\033[31m烧写失败\033[0m";fi;fi)

# 这三条指令用于在编译/烧写时以中文输出执行结果

all: clearoutput compiledo
	@echo $(CHECK_COMPILE)
# 新增显示编译结果功能
#	     编译成功
#      编译失败
#      文件未更新

clearoutput:
	$(CLEAR_OUTPUT)

compiledo: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin force


OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(S_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(S_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.asm=.o)))
vpath %.asm $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@ | tee -a build/output

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@ | tee -a build/output

$(BUILD_DIR)/%.o: %.asm Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@ | tee -a build/output

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@
	@echo "OK" >> build/output
	
$(BUILD_DIR):
	mkdir $@	

gdbserver:
	jlinkgdbserver -if swd -device $(DEVICE)
# 运行jlink gdb server功能

gs:
	jlinkgdbserver -if swd -device $(DEVICE)
# 运行jlink gdb server功能

gdb:
	arm-none-eabi-gdb -x stm32f4_config.gdbinit --nh ./build/$(TARGET).elf
# 运行gdb debug

download: jlinkfile downloaddo
	@echo $(CHECK_DOWNLOAD)
# 新增显示烧写结果功能
#	     烧写成功
#      烧写失败
#      已烧写

downloaddo:
	$(CLEAR_OUTPUT)
	jlinkExe -device $(DEVICE) -if SWD -speed auto -autoconnect 1 -commandfile "$(JLINK_CONFIG_FILENAME)" | tee -a build/output

jlinkfile: Makefile
	echo "h \n\
LoadFile build/$(TARGET).bin 0x8000000 \n\
VerifyBin build/$(TARGET).bin 0x8000000 \n\
r \n\
g \n\
q" > $(JLINK_CONFIG_FILENAME)
#生成烧写相关的jlink命令

view:
	open -a Systemview.app
# 在电脑上打开SystemView软件
```

JLink命令解释

```
h 	halt:Halt CPU.
LoadFile build/drone-iii-401.bin 0x8000000 	Load data file into target memory.
VerifyBin build/drone-iii-401.bin 0x8000000 		Verfy if specified .bin file is at the specified target memory location.
r 	reset:Reset CPU.
g 	go:Start CPU if halted.
q		quit:Close J-Link connection and quit.

```

