#
#  Generic and Simple Atmel AVR GNU Makefile
#
#  Desinged for the gnu-avr tool chain
#
#  (c) 2010 Oliver Kraus (olikraus@gmail.com)
#  
#  This file is part of the dogm128 library (http://code.google.com/p/dogm128/).
#
#  This Makefile is free software: you can redistribute it and/or modify
#  it under the terms of the Lesser GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This Makefile is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  Lesser GNU General Public License for more details.
#
#  You should have received a copy of the Lesser GNU General Public License
#  along with dogm128.  If not, see <http://www.gnu.org/licenses/>.
#
#
# 	Features
#		- upload
#		- create exe from library
#		- create assembler listing (.dis)
#
#	Limitations
#		- only C-files supported
#		- no automatic dependency checking (call 'make clean' if any .h files are changed)
#
#	Targets:
#		make
#			create hex file, no upload
#		make upload
#			create and upload hex file
#		make clean
#			delete all generated files
#
#  Note:
#  	Display list make database: make -p -f/dev/null | less

#================================================
# Project Information
TARGETNAME = dcmot_ctl
MCU:=attiny44
F_CPU:=8000000
#SRC = $(TARGETNAME).c usi_i2c_slave.c
SRC = $(TARGETNAME).c usi_twi_slave.c
#SRC = $(TARGETNAME).c

#================================================
# System/Environment Information
AVRTOOLSPATH:=/usr/bin/
# Type: "avrdude -c ?" to get a full listing.
AVRDUDE_PROGRAMMER := avrispmkii
# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT := usb

#================================================
# Main part of the Makefile starts here. Usually no changes are needed.

# Internal Variable Names
AVRDUDE_FLASH = -U flash:w:$(TARGETNAME).hex
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) -v -v
LIBNAME:=$(TARGETNAME).a
ELFNAME:=$(TARGETNAME).elf
HEXNAME:=$(TARGETNAME).hex
DISNAME:=$(TARGETNAME).dis
OBJ := $(SRC:.c=.o)
# Replace standard build tools by avr tools
CC = $(AVRTOOLSPATH)avr-gcc
CXX = $(AVRTOOLSPATH)avr-g++
AR  = @$(AVRTOOLSPATH)avr-ar
# AVR GNU Tools
OBJCOPY:=$(AVRTOOLSPATH)avr-objcopy
OBJDUMP:=$(AVRTOOLSPATH)avr-objdump
SIZE:=$(AVRTOOLSPATH)avr-size
AVRDUDE = avrdude
# C flags
COMMON_FLAGS = -DF_CPU=$(F_CPU) -mmcu=$(MCU) $(DOGDEFS)
COMMON_FLAGS += -g -Os -Wall -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
COMMON_FLAGS += -I.
CFLAGS = $(COMMON_FLAGS) -std=gnu99 -Wstrict-prototypes  

# Additional Suffixes
.SUFFIXES: .elf .hex .dis

# Targets
.PHONY: all
all: $(DISNAME) $(HEXNAME)
	$(SIZE) $(ELFNAME)
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_FLASH)

.PHONY: upload
upload: $(DISNAME) $(HEXNAME)
	$(SIZE) $(ELFNAME)
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_FLASH)

.PHONY: clean
clean:
	$(RM) $(HEXNAME) $(ELFNAME) $(LIBNAME) $(DISNAME) *.o

.PHONY: zip
zip:
	zip $(TARGETNAME)_`date +%F`.zip *.[hc] Makefile *.odg *.txt *.ino

# implicit rules
.elf.hex:
	@$(OBJCOPY) -O ihex -R .eeprom $< $@

# explicit rules
#$(ELFNAME): $(LIBNAME)($(OBJ)) 
#	$(LINK.o) $(COMMON_FLAGS) $(LIBNAME) $(LOADLIBES) $(LDLIBS) -o $@
$(ELFNAME): $(OBJ) 
	$(LINK.o) $(COMMON_FLAGS) $(OBJ) $(LOADLIBES) $(LDLIBS) -o $@

$(DISNAME): $(ELFNAME)
	$(OBJDUMP) -S $< > $@

	
