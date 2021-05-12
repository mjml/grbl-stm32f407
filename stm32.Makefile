#  Part of Grbl
#
#  Copyright (c) 2009-2011 Simen Svale Skogsrud
#  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
#
#  Grbl is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Grbl is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.


# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

#CLOCK      = 16000000
CLOCK       = 168000000

SOURCE    = main.c motion_control.c gcode.c spindle_control.c coolant_control.c serial.c \
             protocol.c stepper.c eeprom.c settings.c planner.c nuts_bolts.c limits.c jog.c\
             print.c probe.c report.c system.c
BUILDDIR = build
GRBL_PATH = grbl
OBJECTS = $(addprefix $(BUILDDIR)/,$(notdir $(SOURCE:.c=.o)))


OBJDUMP=arm-none-eabi-objdump
OBJCOPY=arm-none-eabi-objcopy
CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
SIZE=arm-none-eabi-size
CFLAGS=-DSTM32 -DSTM32F407xx -Wall -Os -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

# All the following needed for STM32:
# CMSIS (lowest level)
# HAL (basic uC services API)
# USB (usb middleware code)
# USB_CDC (the CDC device profile)

STM32_PATH = stm32f407
STM32_SRC = $(STM32_PATH)
STM32_INCL = $(STM32_PATH)
STM32_MODULES = stm32f4xx_hal_msp.c stm32f4xx_it.c syscalls.c sysmem.c system_stm32f4xx.c
STM32_OBJECTS = $(addprefix $(BUILDDIR)/,$(notdir $(STM32_MODULES:.c=.o)))

CMSIS_ARM_PATH = $(STM32_PATH)/cmsis5
CMSIS_ARM_INCL = $(CMSIS_ARM_PATH)/CMSIS/Core/Include

CMSIS_DEVICE_PATH = $(STM32_PATH)/device
CMSIS_DEVICE_MODULES = system_stm32f4xx.c gcc/startup_stm32f407xx.s
CMSIS_DEVICE_INCL = $(CMSIS_DEVICE_PATH)/Include
CMSIS_DEVICE_SOURCES = $(addprefix $(CMSIS_DEVICE_PATH)/Source/Templates/,$(CMSIS_DEVICE_MODULES))
CMSIS_DEVICE_OBJECTS = $(addprefix $(BUILDDIR)/device/,$(addsuffix .o,$(notdir $(basename $(CMSIS_DEVICE_SOURCES)))))

HAL_PATH = $(STM32_PATH)/hal
HAL_SRC = $(HAL_PATH)/Src
HAL_INCL = $(HAL_PATH)/Inc
HAL_MODULES = gpio.c rcc.c flash.c tim.c cortex.c
HAL_SOURCES = $(addprefix $(HAL_PATH)/stm32f4xx_hal_,$(HAL_MODULES))
HAL_OBJECTS = $(addprefix $(BUILDDIR)/hal/,$(notdir $(HAL_SOURCES:.c=.o)))

USB_PATH = $(STM32_PATH)/usb
USB_SRC = $(USB_PATH)/Core/Src
USB_INCL = $(USB_PATH)/Core/Inc
USB_MODULES = usbd_core.c usbd_ctlreq.c usbd_ioreq.c
USB_SOURCES = $(addprefix $(USB_SRC)/,$(USB_MODULES))
USB_OBJECTS = $(addprefix $(BUILDDIR)/usb/,$(notdir $(USB_SOURCES:.c=.o))) 

USBCDC_PATH = $(USB_PATH)/Class/CDC
USBCDC_SRC = $(USBCDC_PATH)/Src
USBCDC_INCL = $(USBCDC_PATH)/Inc
USBCDC_MODULES = usbd_cdc.c
USBCDC_SOURCES = $(addprefix $(USBCDC_SRC)/,$(USBCDC_MODULES))
USBCDC_OBJECTS = $(addprefix $(BUILDDIR)/usbcdc/,$(notdir $(USBCDC_MODULES:.c=.o))) 

USBIMPL_PATH = $(STM32_PATH)/usbimpl
USBIMPL_SRC = $(USBIMPL_PATH)
USBIMPL_INCL = $(USBIMPL_PATH)
USBIMPL_MODULES = usbd_conf.c usbd_desc.c usbd_cdc_if.c usb_device.c
USBIMPL_SOURCES = $(addprefix $(USBIMPL_PATH)/,$(USBCDC_MODULES))
USBIMPL_OBJECTS = $(addprefix $(BUILDDIR)/usbimpl/,$(USBIMPL_MODULES:.c=.o)) 

# Tune the lines below only if you know what you are doing:

# Compile flags for avr-gcc v4.8.1. Does not produce -flto warnings.
# COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -I. -ffunction-sections

# Compile flags for avr-gcc v4.9.2 compatible with the IDE. Or if you don't care about the warnings. 
# COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -I. -ffunction-sections -flto
COMPILE = $(CC) $(CFLAGS) -I$(STM32_INCL) -I$(CMSIS_ARM_INCL) -I$(CMSIS_DEVICE_INCL) -I$(HAL_INCL) -I$(USB_INCL) -I$(USBCDC_INCL) -I$(USBIMPL_INCL)
ASM = $(AS) -Wall

OBJECTS = $(addprefix $(BUILDDIR)/,$(notdir $(SOURCE:.c=.o))) $(CMSIS_OBJECTS) $(HAL_OBJECTS) $(USB_OBJECTS) $(USBCDC_OBJECTS) $(USBIMPL_OBJECTS)

# symbolic targets:
@default: all
	@echo USB_SOURCES are $(USB_SOURCES)
	@echo USB_OBJECTS are $(USB_OBJECTS)
	@echo USBCDC_SRC is $(USBCDC_SRC)
	@echo USBCDC_SOURCES are $(USBCDC_SOURCES)
	@echo USBCDC_OBJECTS are $(USBCDC_OBJECTS)

all:	grbl.hex

# These two CMSIS modules are special and are added explicitly. 
# These need to be chosen to reflect the ARM architecture
$(BUILDDIR)/device/system_stm32f4xx.o: $(CMSIS_PATH)/Source/Templates/system_stm32f4xx.c
	$(COMPILE) -c $< -o $@

$(BUILDDIR)/device/startup_stm32f407xx.o: $(CMSIS_PATH)/Source/Templates/gcc/startup_stm32f407xx.s
	$(ASM) -c $< -o $@

# Finally, the default rules for the stm32 imported submodules not covered by the above.
$(BUILDDIR)/hal/%.o: $(HAL_SRC)/%.c $(BUILDDIR)/hal
	$(COMPILE) -c $< -o $@

$(BUILDDIR)/hal:
	mkdir -p $@

$(BUILDDIR)/usb/%.o: $(USB_SRC)/%.c $(BUILDDIR)/usb
	$(COMPILE) -c $< -o $@

$(BUILDDIR)/usb:
	mkdir -p $@

$(BUILDDIR)/usbcdc/%.o: $(USBCDC_SRC)/%.c $(BUILDDIR)/usbcdc
	$(COMPILE) -c $< -o $@

$(BUILDDIR)/usbcdc:
	mkdir -p $@

$(BUILDDIR)/usbimpl/%.o: $(USBIMPL_SRC)/%.c $(BUILDDIR)/usbimpl
	$(COMPILE) -c $< -o $@

$(BUILDDIR)/usbimpl:
	mkdir -p $@

# This one's for stm32 core files
$(BUILDDIR)/%.o: $(STM32_PATH)/%.c
	$(COMPILE) -c $< -o $@

# And this one's for the original grbl files
$(BUILDDIR)/%.o: $(GRBL_PATH)/%.c
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $(BUILDDIR)/$@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $(BUILDDIR)/$@

clean:
	rm -f grbl.hex $(BUILDDIR)/*.o $(BUILDDIR)/*.d $(BUILDDIR)/*.elf

# file targets:
$(BUILDDIR)/main.elf: $(OBJECTS) $(CMSIS_OBJECTS) $(HAL_OBJECTS) $(USB_OBJECTS) $(USBCDC_OBJECTS)
	$(COMPILE) -o $(BUILDDIR)/main.elf $(OBJECTS) $(CMSIS_OBJECTS) $(HAL_OBJECTS) $(USB_OBJECTS) $(USBCDC_OBJECTS) -lm -Wl,--gc-sections

grbl.hex: $(BUILDDIR)/main.elf
	rm -f grbl.hex
	$(OBJCOPY) -j .text -j .data -O ihex $(BUILDDIR)/main.elf grbl.hex
	$(SIZE) --format=berkeley $(BUILDDIR)/main.elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	$(BUILDDIR)/main.elf
	$(OBJDUMP) -d $(BUILDDIR)/main.elf

cpp:
	$(COMPILE) -E $(SOURCEDIR)/main.c

# TODO: include generated header dependencies
#-include $(BUILDDIR)/$(OBJECTS:.o=.d)
