/*##############################################################################

	Name	: USI TWI Slave driver - I2C/TWI-EEPROM
	Version	: 1.3  - Stable
	autor	: Martin Junghans	jtronics@gmx.de
	page	: www.jtronics.de
	License	: GNU General Public License 

	Created from Atmel source files for Application Note AVR312: 
	Using the USI Module as an I2C slave like an I2C-EEPROM.

	LICENSE:    Copyright (C) 2010 Marin Junghans

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details
	
	
	
	16.11.2010 - reworked by Markus Schatzl
	Source: http://www.das-labor.org/svn/microcontroller/src-atmel/lib/i2c/trunk/usi-twi_slave/

	15 Apr 2013, several updated by olikraus@gmail.com

	
//############################################################################## 

	Description:
	
	The slave device acts like a I2C-EEPROM.
	
	Write data from master to slave (into slave rxbuffer):

		1. Master sends slave address (bit 7-1) + r/w flag (bit 0), which must be set to 0
		2. Master sends address of where to start with writing data (one byte)
		3. Master sends data to rxbuffer[buffer address]

	Read data from the slave (from slave txbuffer):

		1. Master sends slave address (bit 7-1) + r/w flag (bit 0), which must be set to 0
		2. Master sends address of where to start reading data (one byte)
		3. Master sends slave address (bit 7-1) + r/w flag (bit 0), which must be set to 1
		4. Master waits for callback, demanding the slave to send data starting with txbuffer[buffer address]

	Info:
		- You have to change the TWI_BUFFER_SIZE in the usiTwiSlave.h file
		- Buffer address is automatically incremented
		- If buffer address > buffersize -> start with buffer address 0x00
	
//############################################################################*/

#ifndef _USI_TWI_SLAVE_H_
#define _USI_TWI_SLAVE_H_

//##################################################################### includes

#include <stdbool.h>

//################################################################### prototypes

#define TWI_ADDRESS (15)
void usiTwiSlaveInit(void);	// send slave address

//#################################################################### variables

#define TWI_BUFFER_SIZE 64					//in bytes (2..254), change ONLY here!!!!!


extern volatile uint8_t usi_memory[TWI_BUFFER_SIZE];			// Buffer to write data received from the master
extern volatile uint8_t usi_mem_adr; 					// Virtual buffer address register


#if	(TWI_BUFFER_SIZE > 254)
		#error Buffer to big! Maximal 254 Bytes.
		
#elif	(TWI_BUFFER_SIZE < 2)
		#error Buffer to small! mindestens 2 Bytes!
#endif

//##############################################################################

#endif  // ifndef _USI_TWI_SLAVE_H_
