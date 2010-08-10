CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump

CORE_CFLAGS = -I./ -I$(CORE_SRC) -I$(DEVICE_SRC) -ILibraries/STM32F10x_StdPeriph_Driver/inc -ILibraries/CMSIS/CM3/CoreSupport -ILibraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x -fno-common -O0 -g -mcpu=cortex-m3 -mthumb 
CFLAGS  = $(CORE_CFLAGS) -c 
CFLAGS_LINK = -Wl,-Tmain.ld -nostartfiles $(CORE_CFLAGS)
ASFLAGS = -mcpu=cortex-m3 -mthumb -g
LDFLAGS = -Tmain.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

SRC		= $(realpath .)
CORE_SRC = $(SRC)/Libraries/CMSIS/CM3/CoreSupport
DEVICE_SRC = $(SRC)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
STD_PERIPH_SRC = $(SRC)/Libraries/STM32F10x_StdPeriph_Driver/src

# Directory where QEMU ARM executable is located - used for running
# program in emulator.
#QEMU_ARM_DIR ?= ../qemu/arm-softmmu/

# OpenOCD interface file used for programming/debugging the micronctroller
OPENOCD_INTERFACE ?= interface/olimex-arm-usb-tiny-h.cfg

SYS_FILES = $(CORE_SRC)/core_cm3.c
SYS_FILES += $(DEVICE_SRC)/system_stm32f10x.c
SYS_FILES += $(DEVICE_SRC)/startup/gcc_ride7/startup_stm32f10x_md.s

SRC_FOLDERS += blink_flash
SRC_FOLDERS =  blink_flash_asm
SRC_FOLDERS += button
SRC_FOLDERS += button_int
SRC_FOLDERS += button_int_infinite
SRC_FOLDERS += c_mem_model
SRC_FOLDERS += freertos
SRC_FOLDERS += software_int
SRC_FOLDERS += stkalign
SRC_FOLDERS += systick
SRC_FOLDERS += uart_echo
SRC_FOLDERS += uart_echo_int
SRC_FOLDERS += uart_repeat_write
SRC_FOLDERS += uart_repeat_write_int

FREE_RTOS_SRC = $(SRC)/Libraries/FreeRTOS
FREE_RTOS_SRC_FILES = $(FREE_RTOS_SRC)/croutine.c $(FREE_RTOS_SRC)/list.c $(FREE_RTOS_SRC)/queue.c $(FREE_RTOS_SRC)/tasks.c $(FREE_RTOS_SRC)/croutine.c $(FREE_RTOS_SRC)/portable/GCC/ARM_CM3/port.c
FREE_RTOS_INC = $(FREE_RTOS_SRC)/include/
FREE_RTOS_PORT_INC = $(FREE_RTOS_SRC)/portable/GCC/ARM_CM3/

MAIN_FILES = $(addsuffix /main,$(SRC_FOLDERS))

ELF_FILES	= $(addsuffix .elf,$(MAIN_FILES))
LIST_FILES	= $(addsuffix .list,$(MAIN_FILES))
BIN_FILES	= $(addsuffix .bin,$(MAIN_FILES))

ALL_TARGETS = $(addsuffix _ALL,$(SRC_FOLDERS))
PROG_TARGETS = $(addsuffix _PROG,$(SRC_FOLDERS))

QEMU_RUN_TARGETS = $(addsuffix _QEMURUN,$(SRC_FOLDERS))
QEMU_RUN_PTY_TARGETS = $(addsuffix _QEMURUN_PTY,$(SRC_FOLDERS))
QEMU_RUN_TEL_TARGETS = $(addsuffix _QEMURUN_TEL,$(SRC_FOLDERS))
QEMU_DBG_TARGETS = $(addsuffix _QEMUDBG,$(SRC_FOLDERS))
QEMU_DBG_PTY_TARGETS = $(addsuffix _QEMUDBG_PTY,$(SRC_FOLDERS))
QEMU_DBG_TEL_TARGETS = $(addsuffix _QEMUDBG_TEL,$(SRC_FOLDERS))

LIB_FILES =  stm32f10x_rcc
LIB_FILES += stm32f10x_gpio
#LIB_FULLFILES = $(addprefix $(LIB_SRC)/,$(LIB_FILES))
LIB_OBJ = $(addsuffix .o,$(LIB_FILES))




.PHONY: clean $(ALL_TARGETS) $(PROG_TARGETS) openocd_dbg

all: $(ALL_TARGETS)


clean:
	find . -type f -name "*.o" -exec rm {} \;
	find . -type f -name "*.elf" -exec rm {} \;
	find . -type f -name "*.bin" -exec rm {} \;
	find . -type f -name "*.list" -exec rm {} \; 



$(ALL_TARGETS): %_ALL : %/main.elf %/main.bin %/main.list

$(PROG_TARGETS): %_PROG : %_ALL
	-killall -q openocd
	openocd -f $(OPENOCD_INTERFACE) -f openocd/openocd_stm32_p103.cfg -c "program_flash $*/main.bin"

DBG:
	-killall -q openocd
	openocd -f $(OPENOCD_INTERFACE) -f openocd/openocd_stm32_p103_dbg.cfg
	
	
$(QEMU_RUN_TARGETS): %_QEMURUN : %_ALL
	-killall -q qemu-system-arm
	$(QEMU_ARM_DIR)qemu-system-arm -M stm32-p103 -kernel $*/main.bin

$(QEMU_RUN_PTY_TARGETS): %_QEMURUN_PTY : %_ALL
	-killall -q qemu-system-arm
	$(QEMU_ARM_DIR)qemu-system-arm -M stm32-p103 -kernel $*/main.bin -serial pty
	
$(QEMU_RUN_TEL_TARGETS): %_QEMURUN_TEL : %_ALL
	-killall -q qemu-system-arm
	$(QEMU_ARM_DIR)qemu-system-arm -M stm32-p103 -kernel $*/main.bin -serial tcp::7777,server


$(QEMU_DBG_TARGETS): %_QEMUDBG : %_ALL
	-killall -q qemu-system-arm
	$(QEMU_ARM_DIR)qemu-system-arm -M stm32-p103 -gdb tcp::3333 -S -kernel $*/main.bin

$(QEMU_DBG_PTY_TARGETS): %_QEMUDBG_PTY : %_ALL
	-killall -q qemu-system-arm
	$(QEMU_ARM_DIR)qemu-system-arm -M stm32-p103 -gdb tcp::3333 -S -kernel $*/main.bin -serial pty
	
$(QEMU_DBG_TEL_TARGETS): %_QEMUDBG_TEL : %_ALL
	-killall -q qemu-system-arm
	$(QEMU_ARM_DIR)qemu-system-arm -M stm32-p103 -gdb tcp::3333 -S -kernel $*/main.bin -serial tcp::7777,server




$(LIST_FILES): %.list : %.elf
	$(OD) $(ODFLAGS) $< > $@

$(BIN_FILES): %.bin : %.elf
	$(CP) $(CPFLAGS) $< $@

	





blink_flash_asm/main.elf: blink_flash_asm/main.o
	$(LD) $(LDFLAGS) -nostartfiles -o $@ $<
	
blink_flash_asm/main.o: blink_flash_asm/main.s
	 $(AS) $(ASFLAGS) -o $@ $<


c_mem_model/main.o: c_mem_model/main.c
	$(CC) $(CFLAGS_LINK) -Ic_mem_model/ -c -o $@ $^

c_mem_model/main.elf: c_mem_model/main.o
c_mem_model/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Ic_mem_model/ -o $@ $^


blink_flash/main.elf: blink_flash/main.c
blink_flash/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
blink_flash/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
blink_flash/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Iblink_flash/ -o $@ $^



button/main.elf: button/main.c
button/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
button/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
button/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Ibutton/ -o $@ $^
	
	
	
button_int/main.elf: button_int/main.c
button_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
button_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
button_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_exti.c
button_int/main.elf: $(STD_PERIPH_SRC)/misc.c
button_int/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Ibutton_int/ -o $@ $^
	
	
button_int_infinite/main.elf: button_int_infinite/main.c
button_int_infinite/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
button_int_infinite/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
button_int_infinite/main.elf: $(STD_PERIPH_SRC)/stm32f10x_exti.c
button_int_infinite/main.elf: $(STD_PERIPH_SRC)/misc.c
button_int_infinite/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Ibutton_int_infinite/ -o $@ $^
	


software_int/main.elf: software_int/main.c
software_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
software_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
software_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_exti.c
software_int/main.elf: $(STD_PERIPH_SRC)/misc.c
software_int/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Isoftware_int/ -o $@ $^



uart_echo/main.elf: uart_echo/main.c
uart_echo/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
uart_echo/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
uart_echo/main.elf: $(STD_PERIPH_SRC)/stm32f10x_usart.c
uart_echo/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Iuart_echo/ -o $@ $^


uart_repeat_write/main.elf: uart_repeat_write/main.c
uart_repeat_write/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
uart_repeat_write/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
uart_repeat_write/main.elf: $(STD_PERIPH_SRC)/stm32f10x_usart.c
uart_repeat_write/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Iuart_repeat_write/ -o $@ $^
	
	
	
uart_repeat_write_int/main.elf: uart_repeat_write_int/main.c
uart_repeat_write_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
uart_repeat_write_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
uart_repeat_write_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_usart.c
uart_repeat_write_int/main.elf: $(STD_PERIPH_SRC)/misc.c
uart_repeat_write_int/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Iuart_repeat_write_int/ -o $@ $^

uart_echo_int/main.elf: uart_echo_int/main.c
uart_echo_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
uart_echo_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
uart_echo_int/main.elf: $(STD_PERIPH_SRC)/stm32f10x_usart.c
uart_echo_int/main.elf: $(STD_PERIPH_SRC)/misc.c
uart_echo_int/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Iuart_echo_int/ -o $@ $^
	
	
systick/main.elf: systick/main.c
systick/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
systick/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
systick/main.elf: $(STD_PERIPH_SRC)/misc.c
systick/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Isystick/ -o $@ $^
	
	
	
freertos/main.elf: freertos/main.c
freertos/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
freertos/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
freertos/main.elf: $(STD_PERIPH_SRC)/stm32f10x_exti.c
freertos/main.elf: $(STD_PERIPH_SRC)/stm32f10x_usart.c
freertos/main.elf: $(STD_PERIPH_SRC)/misc.c
freertos/main.elf: $(SYS_FILES)
freertos/main.elf: $(FREE_RTOS_SRC_FILES)
freertos/main.elf: $(FREE_RTOS_SRC)/portable/MemMang/heap_1.c
	$(CC) $(CFLAGS_LINK) -Ifreertos/ -I$(FREE_RTOS_INC) -I$(FREE_RTOS_PORT_INC) -o $@ $^


stkalign/main.elf: stkalign/main.c
stkalign/main.elf: $(STD_PERIPH_SRC)/stm32f10x_rcc.c
stkalign/main.elf: $(STD_PERIPH_SRC)/stm32f10x_gpio.c
stkalign/main.elf: $(STD_PERIPH_SRC)/stm32f10x_exti.c
stkalign/main.elf: $(STD_PERIPH_SRC)/misc.c
stkalign/main.elf: $(SYS_FILES)
	$(CC) $(CFLAGS_LINK) -Istkalign/ -o $@ $^
	
		 
