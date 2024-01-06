
################ Top configuration.
PRJ_TOP = ..
OBJDIR = out
TARGET = $(OBJDIR)/boot

MCU = rv32imac_zicsr
#MCU =rv64imc

BUILD_STRICT = TRUE
BUILD_PRINT = TRUE

OPTI = -O1
VER_STRING = $$(date_time)_$(OPTI)

LINK_SCRIPT = startup/link.ld

################  Define
DEFINE =
DEFINE += WCH_INT

################  Library directory.
PRJ_LIB_DIR = driver

EXT_LIB_DIR = 

LIB_FILE = ISP573 c_nano

################  Include.
# Add relative path from $(PRJ_TOP)
PRJ_INC = . \
		build \
		src \
		com \
		rvmsis \
		driver

# Add absolue path. (ex. c:/lib/inc)
EXT_INC =

################ source files ##############
# Source file들은 project TOP 에서의 위치를 나타낸다.
CSRC =	\
		src/key.c \
		src/usb_cdc.c \
		src/hal.c \
		src/main.c \
		src/sched.c \
		src/cli.c \
		com/util.c \
		rvmsis/core_riscv.c \
		driver/CH57x_usbdev.c \
		driver/CH57x_gpio.c \
		driver/CH57x_uart0.c \
		driver/CH57x_uart1.c \
		driver/CH57x_timer0.c \
		driver/CH57x_sys.c

CPPSRC =

ASRC =	startup/startup_CH573.S

DATA =

include rules.mk

test:
	@echo Add test make target.


prog: $(TARGET).elf

