##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = main

ifndef DEVICE
	$(error need DEVICE)
endif

BUILDDIR = build_$(DEVICE)/

CFLAGS += -std=c99 -ggdb
CXXFLAGS += -std=c++11 -ggdb -Wno-unused-parameter -DSYLPHASE_GPSLOG_$(DEVICE)
LDSCRIPT = STM32F103CBT7.ld
LDFLAGS += -ggdb

LIBNAME		= opencm3_stm32f1
DEFS		= -DSTM32F1

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

################################################################################
# OpenOCD specific variables

OOCD		?= openocd
OOCD_INTERFACE	?= flossjtag
OOCD_BOARD	?= olimex_stm32_h103

################################################################################
# Black Magic Probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
BMP_PORT	?=

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242

# Be silent per default, but 'make V=1' will show all compiler calls.

###############################################################################
# Executables

PREFIX		?= arm-none-eabi

CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-g++
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb
STFLASH		= $(shell which st-flash)


###############################################################################
# Source files

OBJS		+= $(BUILDDIR)$(BINARY).o $(BUILDDIR)time.o $(BUILDDIR)hardware.o $(BUILDDIR)serial.o $(BUILDDIR)gps.o $(BUILDDIR)sdcard.o $(BUILDDIR)ff/ff.o $(BUILDDIR)ff/option/unicode.o $(BUILDDIR)coroutine.o $(BUILDDIR)sensors.o $(BUILDDIR)reactor.o $(BUILDDIR)scheduler.o $(BUILDDIR)misc.o


ifeq ($(strip $(OPENCM3_DIR)),)
# user has not specified the library path, so we try to detect it

# where we search for the library
LIBPATHS := ./libopencm3 ../../../../libopencm3 ../../../../../libopencm3

OPENCM3_DIR := $(wildcard $(LIBPATHS:=/locm3.sublime-project))
OPENCM3_DIR := $(firstword $(dir $(OPENCM3_DIR)))

ifeq ($(strip $(OPENCM3_DIR)),)
$(warning Cannot find libopencm3 library in the standard search paths.)
$(error Please specify it through OPENCM3_DIR variable!)
endif
endif

ifeq ($(V),1)
$(info Using $(OPENCM3_DIR) path to library)
endif

INCLUDE_DIR	= $(OPENCM3_DIR)/include
LIB_DIR		= $(OPENCM3_DIR)/lib
SCRIPT_DIR	= $(OPENCM3_DIR)/scripts

###############################################################################
# C flags

CFLAGS		+= -Os -g
CFLAGS		+= -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS		+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS		+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C++ flags

CXXFLAGS	+= -Os -g
CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions

###############################################################################
# C & C++ preprocessor common flags

CPPFLAGS	+= -MD
CPPFLAGS	+= -Wall -Wundef
CPPFLAGS	+= -I$(INCLUDE_DIR) $(DEFS)

###############################################################################
# Linker flags

LDFLAGS		+= --static -nostartfiles
LDFLAGS		+= -L$(LIB_DIR)
LDFLAGS		+= -T$(LDSCRIPT)
LDFLAGS		+= -Wl,-Map=$(*).map
LDFLAGS		+= -Wl,--gc-sections -fno-rtti -fno-exceptions
ifeq ($(V),99)
LDFLAGS		+= -Wl,--print-gc-sections
endif

###############################################################################
# Used libraries

LDLIBS		+= -l$(LIBNAME)
LDLIBS		+= -Wl,--start-group -lc -lnosys -lm

###############################################################################
###############################################################################
###############################################################################

.SUFFIXES: .elf .bin .hex .srec .list .map .images
.SECONDEXPANSION:
.SECONDARY:

all: elf

elf: $(BUILDDIR)$(BINARY).elf
bin: $(BUILDDIR)$(BINARY).bin
hex: $(BUILDDIR)$(BINARY).hex
srec: $(BUILDDIR)$(BINARY).srec
list: $(BUILDDIR)$(BINARY).list

images: $(BUILDDIR)$(BINARY).images
flash: $(BUILDDIR)$(BINARY).stlink-flash

$(BUILDDIR)%.images: $(BUILDDIR)%.bin $(BUILDDIR)%.hex $(BUILDDIR)%.srec $(BUILDDIR)%.list $(BUILDDIR)%.map
	@#printf "*** $* images generated ***\n"

$(BUILDDIR)%.bin: $(BUILDDIR)%.elf
	@mkdir -p $(BUILDDIR)
	@#printf "  OBJCOPY $(*).bin\n"
	$(OBJCOPY) -Obinary $(BUILDDIR)$(*).elf $(BUILDDIR)$(*).bin

$(BUILDDIR)%.hex: $(BUILDDIR)%.elf
	@mkdir -p $(BUILDDIR)
	@#printf "  OBJCOPY $(*).hex\n"
	$(OBJCOPY) -Oihex $(BUILDDIR)$(*).elf $(BUILDDIR)$(*).hex

$(BUILDDIR)%.srec: $(BUILDDIR)%.elf
	@mkdir -p $(BUILDDIR)
	@#printf "  OBJCOPY $(*).srec\n"
	$(OBJCOPY) -Osrec $(BUILDDIR)$(*).elf $(BUILDDIR)$(*).srec

$(BUILDDIR)%.list: $(BUILDDIR)%.elf
	@mkdir -p $(BUILDDIR)
	@#printf "  OBJDUMP $(*).list\n"
	$(OBJDUMP) -S $(BUILDDIR)$(*).elf > $(BUILDDIR)$(*).list

$(BUILDDIR)%.elf $(BUILDDIR)%.map: $(OBJS) $(LDSCRIPT) $(LIB_DIR)/lib$(LIBNAME).a
	@mkdir -p $(BUILDDIR)
	@#printf "  LD      $(*).elf\n"
	$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $(BUILDDIR)$(*).elf

$(BUILDDIR)%.o: src/%.c
	@mkdir -p $(dir $@)
	@#printf "  CC      $(*).c\n"
	$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(BUILDDIR)$(*).o -c src/$(*).c

$(BUILDDIR)%.o: src/%.cxx
	@mkdir -p $(dir $@)
	@#printf "  CXX     $(*).cxx\n"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(BUILDDIR)$(*).o -c src/$(*).cxx

$(BUILDDIR)%.o: src/%.cpp
	@mkdir -p $(dir $@)
	@#printf "  CXX     $(*).cpp\n"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(BUILDDIR)$(*).o -c src/$(*).cpp

clean:
	@#printf "  CLEAN\n"
	rm -fr $(BUILDDIR)


%.stlink-flash: %.bin
	@printf "  FLASH  $<\n"
	$(STFLASH) write $(*).bin 0x8000000

ifeq ($(STLINK_PORT),)
ifeq ($(BMP_PORT),)
ifeq ($(OOCD_SERIAL),)
%.flash: %.hex
	@printf "  FLASH   $<\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		    -f board/$(OOCD_BOARD).cfg \
		    -c "init" -c "reset init" \
		    -c "flash write_image erase $(*).hex" \
		    -c "reset" \
		    -c "shutdown" $(NULL)
else
%.flash: %.hex
	@printf "  FLASH   $<\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		    -f board/$(OOCD_BOARD).cfg \
		    -c "ft2232_serial $(OOCD_SERIAL)" \
		    -c "init" -c "reset init" \
		    -c "flash write_image erase $(*).hex" \
		    -c "reset" \
		    -c "shutdown" $(NULL)
endif
else
%.flash: %.elf
	@printf "  GDB   $(*).elf (flash)\n"
	$(GDB) --batch \
		   -ex 'target extended-remote $(BMP_PORT)' \
		   -x $(SCRIPT_DIR)/black_magic_probe_flash.scr \
		   $(*).elf
endif
else
%.flash: %.elf
	@printf "  GDB   $(*).elf (flash)\n"
	$(GDB) --batch \
		   -ex 'target extended-remote $(STLINK_PORT)' \
		   -x $(SCRIPT_DIR)/stlink_flash.scr \
		   $(*).elf
endif

.PHONY: images clean elf bin hex srec list

-include $(OBJS:.o=.d)
