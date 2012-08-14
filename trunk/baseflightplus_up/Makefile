###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the Baseflight U.P. firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
# 

###############################################################################
# Things that the user might override on the commandline
#

# The target to build, must be one of NAZE or FY90Q
TARGET		?= NAZE

# Compile-time options
OPTIONS		?=

###############################################################################
# Things that need to be maintained as the source changes
#

VALID_TARGETS	 = NAZE FY90Q

SERIAL_DEV      = /dev/cu.SLAB_USBtoUART
#SERIAL_DEV      = /dev/tty.CRIUS_BT-DevB

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)/src
CMSIS_DIR	 = $(ROOT)/lib/CMSIS
STDPERIPH_DIR	 = $(ROOT)/lib/STM32F10x_StdPeriph_Driver
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj

# Source files common to all targets
COMMON_SRC	 = startup_stm32f10x_md_gcc.S \
			main.c \
			syscalls.c \
			actuator/mixer.c \
			actuator/pid.c \
			actuator/stabilisation.c \
			core/cli.c \
			core/command.c \
			core/config.c \
			core/mavlink.c \
			core/printf_min.c \
			core/serial.c \
			core/utilities.c \
			drivers/adc.c \
			drivers/i2c.c \
			drivers/pwm_ppm.c \
			drivers/system.c \
			drivers/uart.c \
			estimator/altitude.c \
			estimator/attitude.c \
			estimator/lp_filter.c \
			estimator/MahonyAHRS.c \
			sensors/accel.c \
			sensors/battery.c \
			sensors/gyro.c \
			sensors/sensors.c \
			sensors/devices/adxl345.c \
			sensors/devices/bmp085.c \
			sensors/devices/hmc5883.c \
			sensors/mag.c \
			sensors/devices/mpu3050.c \
			sensors/devices/mpu6050.c \
			$(CMSIS_SRC) \
			$(STDPERIPH_SRC)

# Source files for the NAZE target
NAZE_SRC	 = $(COMMON_SRC)

# Source files for the FY90Q target
FY90Q_SRC	 = $(COMMON_SRC)

# Search path for baseflight sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			               $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC			= arm-none-eabi-gcc
OBJCOPY		= arm-none-eabi-objcopy
SIZE        = arm-none-eabi-size
LOADER      = support/stmloader/stmloader

#
# Tool options.
#
INCLUDE_DIRS	= $(SRC_DIR) \
				$(STDPERIPH_DIR)/inc \
				$(CMSIS_DIR)/CM3/CoreSupport \
				$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \
				$(ROOT)/lib

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
CFLAGS		 = $(ARCH_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -Os \
		   -Wall \
		   -ffunction-sections \
		   -fdata-sections \
		   -DSTM32F10X_MD \
		   -DUSE_STDPERIPH_DRIVER \
		   -D$(TARGET) \

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LD_SCRIPT	 = $(ROOT)/stm32_flash.ld
LDFLAGS		 = -lm \
		   $(ARCH_FLAGS) \
		   -static \
		   -Wl,-gc-sections \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

TARGET_HEX	 = $(BIN_DIR)/baseflight_up_$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/baseflight_up_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))

HEXSIZE = $(SIZE) -A $(TARGET_HEX) | grep -i Total
ELFSIZE = $(SIZE) -A $(TARGET_ELF) | grep -i Total

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 
$(OBJECT_DIR)/$(TARGET)/%.o): %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

clean:
	rm -f $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS)

size:
	$(HEXSIZE)

install:
	$(HEXSIZE)
	$(LOADER) -r -p $(SERIAL_DEV) -b 115200 -f $(TARGET_HEX)

help:
	@echo ""
	@echo "Makefile for the baseflight firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
