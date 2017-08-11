##############################################################################
# Product: Makefile for EK-TM4C123GXL, QUTEST, GNU-ARM
# Last updated for version 5.9.6
# Last updated on  2017-08-03
#
#                    Q u a n t u m     L e a P s
#                    ---------------------------
#                    innovating embedded systems
#
# Copyright (C) Quantum Leaps, LLC. All rights reserved.
#
# This program is open source software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Alternatively, this program may be distributed and modified under the
# terms of Quantum Leaps commercial licenses, which expressly supersede
# the GNU General Public License and are specifically designed for
# licensees interested in retaining the proprietary status of their code.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Contact information:
# https://state-machine.com
# mailto:info@state-machine.com
##############################################################################
#
# examples of invoking this Makefile:
# make -fek-tm4c123gxl.mak  # make and run the tests in the current directory
# make -fek-tm4c123gxl.mak TESTS=philo*.tcl  # make and run the selected tests
# make -fek-tm4c123gxl.mak norun  # only make but not run the tests
# make -fek-tm4c123gxl.mak clean  # cleanup the build
#
# NOTE:
# To use this Makefile on Windows, you will need the GNU make utility, which
# is included in the Qtools collection for Windows, see:
#    http://sourceforge.net/projects/qpc/files/Qtools/
#

#-----------------------------------------------------------------------------
# project name, binary output directory
#
PROJECT := test_qutest
TARGET  := ek-tm4c123gxl

#-----------------------------------------------------------------------------
# project directories
#

# location of the QP/C++ framework (if not provided in an environemnt var.)
ifeq ($(QPCPP),)
QPCPP := ../../..
endif

# location of the QTOOLS directory (if not provided in an environemnt var.)
ifeq ($(QTOOLS),)
QTOOLS := c:/qp/qtools
endif


# QP port used in this project
QP_PORT_DIR := $(QPCPP)/ports/arm-cm/qutest

# list of all source directories used by this project
VPATH = \
	../$(TARGET) \
	$(QPCPP)/src/qf \
	$(QPCPP)/src/qs \
	$(QP_PORT_DIR) \
	$(QPCPP)/3rd_party/ek-tm4c123gxl \
	$(QPCPP)/3rd_party/ek-tm4c123gxl/gnu

# list of all include directories needed by this project
INCLUDES  = \
	-I. \
	-I.. \
	-I../$(TARGET) \
	-I$(QPCPP)/include \
	-I$(QPCPP)/src \
	-I$(QP_PORT_DIR) \
	-I$(QPCPP)/3rd_party/CMSIS/Include \
	-I$(QPCPP)/3rd_party/ek-tm4c123gxl

#-----------------------------------------------------------------------------
# files
#

# assembler source files
ASM_SRCS :=

# C source files
C_SRCS := \
	system_TM4C123GH6PM.c \
	startup_TM4C123GH6PM.c

# C++ source files
CPP_SRCS := \
	test_dpp.cpp \
	main.cpp \
	philo.cpp \
	table.cpp \
	bsp.cpp \
	qutest_port.cpp

OUTPUT    := $(PROJECT)
LD_SCRIPT := ../$(TARGET)/test.ld

QP_SRCS := \
	qep_hsm.cpp \
	qep_msm.cpp \
	qf_act.cpp \
	qf_defer.cpp \
	qf_dyn.cpp \
	qf_mem.cpp \
	qf_ps.cpp \
	qf_qact.cpp \
	qf_qeq.cpp \
	qf_qmact.cpp \
	qs.cpp \
	qs_64bit.cpp \
	qs_rx.cpp \
	qs_fp.cpp \
	qutest.cpp

QP_ASMS :=

QS_SRCS := \
	qs.cpp \
	qs_rx.cpp \
	qs_fp.cpp

LIB_DIRS  :=
LIBS      :=

# defines
DEFINES   := -DTARGET_IS_TM4C123_RB1

# ARM CPU, ARCH, FPU, and Float-ABI types...
# ARM_CPU:   [cortex-m0 | cortex-m0plus | cortex-m1 | cortex-m3 | cortex-m4]
# ARM_ARCH:  [6 | 7] (NOTE: must match ARM_CPU!)
# ARM_FPU:   [ | vfp]
# FLOAT_ABI: [ | soft | softfp | hard]
#
ARM_CPU   := -mcpu=cortex-m4
ARM_ARCH  := 7   # NOTE: must match the ARM_CPU!
ARM_FPU   := -mfpu=vfp
FLOAT_ABI := -mfloat-abi=softfp

#-----------------------------------------------------------------------------
# GNU-ARM toolset (NOTE: You need to adjust to your machine)
# see http://gnutoolchains.com/arm-eabi/
#
ifeq ($(GNU_ARM),)
GNU_ARM := $(QTOOLS)/gnu_arm-eabi
endif

# make sure that the GNU-ARM toolset exists...
ifeq ("$(wildcard $(GNU_ARM))","")
$(error GNU_ARM toolset not found. Please adjust the Makefile)
endif

CC    := $(GNU_ARM)/bin/arm-eabi-gcc
CPP   := $(GNU_ARM)/bin/arm-eabi-g++
AS    := $(GNU_ARM)/bin/arm-eabi-as
LINK  := $(GNU_ARM)/bin/arm-eabi-g++
BIN   := $(GNU_ARM)/bin/arm-eabi-objcopy

#-----------------------------------------------------------------------------
# LMFLASH toolset (NOTE: You need to adjust to your machine)
# see http://www.ti.com/tool/lmflashprogrammer
#
ifeq ($(LMFLASH),)
LMFLASH = $(QTOOLS)/../LM_Flash_Programmer/LMFlash.exe
endif

# make sure that the LMFLASH tool exists...
ifeq ("$(wildcard $(LMFLASH))","")
$(error LMFLASH tool not found. Please adjust the Makefile)
endif


##############################################################################
# Typically you should not need to change anything below this line

# basic utilities (included in Qtools for Windows), see:
#    http://sourceforge.net/projects/qpc/files/Qtools

MKDIR  := mkdir
RM     := rm
TCLSH  := tclsh
QUTEST := $(QTOOLS)/qspy/tcl/qutest.tcl

#-----------------------------------------------------------------------------
# build options
#

# combine all the soruces...
VPATH    += $(QPCPP)/3rd_party/gnu_cpp
CPP_SRCS += $(QP_SRCS) mini_cpp.cpp
ASM_SRCS += $(QP_ASMS)

# add the pre-defined symbol for ARM architecture
DEFINES += -D__ARM_ARCH=$(ARM_ARCH)
ASM_CPU += -defsym=__ARM_ARCH=$(ARM_ARCH)

# add the pre-defined symbol __FPU_PRESENT if needed...
ifneq (,$(ARM_FPU))
DEFINES += -D__FPU_PRESENT
ASM_FPU += -defsym=__FPU_PRESENT=1
endif

BIN_DIR := $(TARGET)

CPP_SRCS += $(QS_SRCS)

ASFLAGS = -g $(ARM_CPU) $(ARM_FPU) $(ASM_CPU) $(ASM_FPU)

CFLAGS = -g $(ARM_CPU) $(ARM_FPU) $(FLOAT_ABI) -mthumb -Wall \
	-ffunction-sections -fdata-sections \
	-O $(INCLUDES) $(DEFINES) -DQ_SPY -DQ_UTEST

CPPFLAGS = -g $(ARM_CPU) $(ARM_FPU) $(FLOAT_ABI) -mthumb -Wall \
	-ffunction-sections -fdata-sections -fno-rtti -fno-exceptions \
	-O $(INCLUDES) $(DEFINES) -DQ_SPY -DQ_UTEST


LINKFLAGS = -T$(LD_SCRIPT) $(ARM_CPU) $(ARM_FPU) $(FLOAT_ABI) \
	-mthumb -nostdlib \
	-Wl,-Map,$(BIN_DIR)/$(OUTPUT).map,--cref,--gc-sections $(LIB_DIRS)

ASM_OBJS     := $(patsubst %.s,%.o,  $(notdir $(ASM_SRCS)))
C_OBJS       := $(patsubst %.c,%.o,  $(notdir $(C_SRCS)))
CPP_OBJS     := $(patsubst %.cpp,%.o,$(notdir $(CPP_SRCS)))

TARGET_BIN   := $(BIN_DIR)/$(OUTPUT).bin
TARGET_ELF   := $(BIN_DIR)/$(OUTPUT).elf
ASM_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(ASM_OBJS))
C_OBJS_EXT   := $(addprefix $(BIN_DIR)/, $(C_OBJS))
C_DEPS_EXT   := $(patsubst %.o, %.d, $(C_OBJS_EXT))
CPP_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(CPP_OBJS))
CPP_DEPS_EXT := $(patsubst %.o, %.d, $(CPP_OBJS_EXT))

# create $(BIN_DIR) if it does not exist
ifeq ("$(wildcard $(BIN_DIR))","")
$(shell $(MKDIR) $(BIN_DIR))
endif

#-----------------------------------------------------------------------------
# rules
#

.PHONY : run norun flash

ifeq ($(MAKECMDGOALS),norun)
all : $(TARGET_BIN)
norun : all
else
all : $(TARGET_BIN) run
endif

ifeq (, $(TESTS))
TESTS := *.tcl
endif

$(TARGET_BIN) : $(TARGET_ELF)
	$(BIN) -O binary $< $@
	$(LMFLASH) -q ek-tm4c123gxl $(TARGET_BIN)
	echo Press RESET button on the EK-TM4C123GXL board
	@pause

$(TARGET_ELF) : $(ASM_OBJS_EXT) $(C_OBJS_EXT) $(CPP_OBJS_EXT)
	$(CPP) $(CPPFLAGS) -c $(QPCPP)/include/qstamp.cpp -o $(BIN_DIR)/qstamp.o
	$(LINK) $(LINKFLAGS) -o $@ $^ $(BIN_DIR)/qstamp.o $(LIBS)

flash :
	$(LMFLASH) -q ek-tm4c123gxl $(TARGET_BIN)
	echo Press RESET button on the EK-TM4C123GXL board

run : $(TARGET_BIN)
	$(TCLSH) $(QUTEST) $(TESTS)

$(BIN_DIR)/%.d : %.c
	$(CC) -MM -MT $(@:.d=.o) $(CFLAGS) $< > $@

$(BIN_DIR)/%.d : %.cpp
	$(CPP) -MM -MT $(@:.d=.o) $(CPPFLAGS) $< > $@

$(BIN_DIR)/%.o : %.s
	$(AS) $(ASFLAGS) $< -o $@

$(BIN_DIR)/%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN_DIR)/%.o : %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

.PHONY : clean show

# include dependency files only if our goal depends on their existence
ifneq ($(MAKECMDGOALS),clean)
  ifneq ($(MAKECMDGOALS),show)
-include $(C_DEPS_EXT) $(CPP_DEPS_EXT)
  endif
endif

clean :
	-$(RM) $(BIN_DIR)/*.o \
	$(BIN_DIR)/*.d \
	$(BIN_DIR)/*.bin \
	$(BIN_DIR)/*.elf \
	$(BIN_DIR)/*.map

show :
	@echo PROJECT      = $(PROJECT)
	@echo TESTS        = $(TESTS)
	@echo TARGET_ELF   = $(TARGET_ELF)
	@echo CONF         = $(CONF)
	@echo VPATH        = $(VPATH)
	@echo C_SRCS       = $(C_SRCS)
	@echo CPP_SRCS     = $(CPP_SRCS)
	@echo ASM_SRCS     = $(ASM_SRCS)
	@echo C_DEPS_EXT   = $(C_DEPS_EXT)
	@echo C_OBJS_EXT   = $(C_OBJS_EXT)

	@echo CPP_DEPS_EXT = $(CPP_DEPS_EXT)
	@echo CPP_OBJS_EXT = $(CPP_OBJS_EXT)

	@echo ASM_OBJS_EXT = $(ASM_OBJS_EXT)
	@echo LIB_DIRS     = $(LIB_DIRS)
	@echo LIBS         = $(LIBS)
	@echo DEFINES      = $(DEFINES)
	@echo QUTEST       = $(QUTEST)
