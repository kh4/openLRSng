##################################################################
#
# Makefile for OpenLRSng
#

#
# If your Arduino is in a weird place, you'll need to change this.
#
ARDUINO_PATH=/usr/share/arduino

#
# Board type can be one of 6 values:
# 0 - Flytron M1 TX
# 1 - Flytron M1 RX
# 2 - Flytron M2, M3 TX, Orange-TX
# 3 - Flytron V2 RX, Hawkeye RX, HK Orange-RX
# 4 - Hawkeye TX, OpenLRSng TX
# 5 - DTF 4ch RX
# 6 - Deluxe TX
#
BOARD_TYPE=3

#
# You can compile all TX as TX, and all RX as either RX or TX.
# You cannot currently compile TX as RX.
# This flag controls what primary function the board will have
#
COMPILE_TX=1

#
# No real user options below here.
##################################################################

#
# You don't want to change this unless you really know that you
# need to.  CPU clock.
#
CLOCK=16000000L

#
# Board type 6 requires a different Arduino target
#
ifeq ($(BOARD_TYPE),6)
CPU=atmega32u4
USB_VID=0x2341
USB_PID=0x8036
VARIANT=leonardo
else
CPU=atmega328p
USB_VID=null
USB_PID=null
VARIANT=standard
endif

#
# C preprocessor defines
#
ifeq ($(COMPILE_TX),1)
DEFINES=-DBOARD_TYPE=$(BOARD_TYPE) -DCOMPILE_TX
FIRMWARE_NAME=TX-$(BOARD_TYPE).hex
else
DEFINES=-DBOARD_TYPE=$(BOARD_TYPE)
FIRMWARE_NAME=RX-$(BOARD_TYPE).hex
endif

#
# AVR GCC info
#
EXEPATH=$(ARDUINO_PATH)/hardware/tools/avr/bin
EXEPREFIX=avr-

#
# AVR gcc and binutils
#
CC=$(EXEPATH)/$(EXEPREFIX)gcc
CXX=$(EXEPATH)/$(EXEPREFIX)g++
AR=$(EXEPATH)/$(EXEPREFIX)ar
SIZE=$(EXEPATH)/$(EXEPREFIX)size
OBJCOPY=$(EXEPATH)/$(EXEPREFIX)objcopy

#
# Compile flags
#
COPTFLAGS= -g -Os
CFLAGS=-Wall -ffunction-sections -fdata-sections -mmcu=$(CPU) -DF_CPU=$(CLOCK) -MMD \
	-DUSB_VID=$(USB_VID) -DUSB_PID=$(USB_PID) -DARDUINO=105 -D__PROG_TYPES_COMPAT__ $(DEFINES)
CXXFLAGS=-fno-exceptions

#
# Arduino libraries used, compilation settings.
#
ARDUINO_LIBS=EEPROM
ARDUINO_LIB_PATH=$(ARDUINO_PATH)/libraries/
ARDUINO_LIB_DIRS=$(addprefix $(ARDUINO_LIB_PATH),$(ARDUINO_LIBS))
ARDUINO_LIB_INCL=$(addsuffix $(ARDUINO_LIBS),-I$(ARDUINO_LIB_PATH))
ARDUINO_LIB_SRCS=$(addsuffix .cpp,$(addprefix $(ARDUINO_LIB_PATH),$(ARDUINO_LIBS)/$(ARDUINO_LIBS)))
ARDUINO_LIB_OBJS=$(patsubst %.cpp, libraries/%.o, $(addsuffix .cpp,$(ARDUINO_LIBS)))

#
# Arduino variant settings
#
ARDUINO_VARIANT_PATH=$(ARDUINO_PATH)/hardware/arduino/variants/$(VARIANT)

#
# Arduino library files used, compilation settings.
#
ARDUINO_CORELIB_PATH=$(ARDUINO_PATH)/hardware/arduino/cores/arduino/
ARDUINO_CORELIB_SRCS=WInterrupts.c wiring.c wiring_shift.c wiring_digital.c \
		     wiring_pulse.c wiring_analog.c \
		     CDC.cpp Print.cpp HardwareSerial.cpp WString.cpp IPAddress.cpp \
		     Stream.cpp main.cpp USBCore.cpp HID.cpp new.cpp Tone.cpp WMath.cpp
ARDUINO_CORELIB_OBJS= $(patsubst %.c, libraries/%.o, $(patsubst %.cpp, libraries/%.o, $(ARDUINO_CORELIB_SRCS)))


#
# Arduino stdc library files used, compilation settings.
#
ARDUINO_LIBC_PATH=/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc/
ARDUINO_LIBC_SRCS=malloc.c realloc.c

#
# Master include path
#
INCLUDE=-I$(ARDUINO_CORELIB_PATH) -I$(ARDUINO_VARIANT_PATH) $(ARDUINO_LIB_INCL) -I.

#
# Target object files
#
OBJS=out/tmp/openLRSng.o $(ARDUINO_LIB_OBJS) libraries/libcore.a

#
# Master target
#
all: all_firmwares

#
# Target to build one specific firmware
#
firmware: out/tmp/openLRSng.hex
	cp out/tmp/openLRSng.hex out/$(FIRMWARE_NAME)

#
# From here down are build rules
#
VPATH := $(ARDUINO_LIB_DIRS) $(ARDUINO_CORELIB_PATH) $(ARDUINO_LIBC_PATH)

define ino-command
	$(CXX) -c $(COPTFLAGS) $(CXXFLAGS) $(CFLAGS) $(INCLUDE) -o $@ -x c++ $<
endef
define cc-command
	$(CC) -c $(COPTFLAGS) $(CFLAGS) $(INCLUDE) -o $@ $<
endef
define cxx-command
	$(CXX) -c $(COPTFLAGS) $(CXXFLAGS) $(CFLAGS) $(INCLUDE) -o $@ $<
endef

.PHONY: all clean upload

dir_guard=@mkdir -p $(@D)

out/tmp/%.o: %.ino
	$(dir_guard)
	$(ino-command)

out/tmp/%.o: %.c
	$(dir_guard)
	$(cc-command)

out/tmp/%.o: %.cpp
	$(dir_guard)
	$(cxx-command)

libraries/%.o: %.c
	$(cc-command)

libraries/%.o: %.cpp
	$(cxx-command)

#
# Other targets
#
clean:
	rm -rf libraries/*.[aod] out/tmp

all_firmwares:
	$(MAKE) clean
	$(MAKE) firmware COMPILE_TX=1 BOARD_TYPE=2
	$(MAKE) clean
	$(MAKE) firmware COMPILE_TX=1 BOARD_TYPE=3
	$(MAKE) clean
	$(MAKE) firmware COMPILE_TX=1 BOARD_TYPE=4
	$(MAKE) clean
	$(MAKE) firmware COMPILE_TX=1 BOARD_TYPE=5
	$(MAKE) clean
	$(MAKE) firmware COMPILE_TX=1 BOARD_TYPE=6
	$(MAKE) clean
	$(MAKE) firmware COMPILE_TX=0 BOARD_TYPE=3
	$(MAKE) clean
	$(MAKE) firmware COMPILE_TX=0 BOARD_TYPE=5

out/tmp/openLRSng.hex: $(OBJS)
	@$(CC) -Os -Wl,--gc-sections -mmcu=atmega328p -o out/tmp/openLRSng.elf $(OBJS) -Llibraries -lm
	@$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load \
		--no-change-warnings --change-section-lma .eeprom=0 \
		out/tmp/openLRSng.elf out/tmp/openLRSng.eep
	@$(OBJCOPY) -O ihex -R .eeprom out/tmp/openLRSng.elf out/tmp/openLRSng.hex
	@echo "NOTE: Deployment size is text + data."
	@$(SIZE) out/tmp/openLRSng.elf

libraries/libcore.a: $(ARDUINO_CORELIB_OBJS)
	$(AR) rcs libraries/libcore.a $(ARDUINO_CORELIB_OBJS)

astyle : $(ASTYLE_FILES:.h=.astyle-check-stamp)
	astyle --options=etc/astyle.cfg *.h

