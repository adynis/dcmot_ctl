/*##############################################################################

	Name	: USI TWI Slave driver - I2C/TWI-EEPROM
	Version	: 1.3  - Stable
	autor	: Martin Junghans	jtronics@gmx.de
	page	: www.jtronics.de
	License	: GNU General Public License 

	Created from Atmel source files for Application Note AVR312:
	Using the USI Module as an I2C slave like an I2C-EEPROM.
	
	
	Source: http://www.das-labor.org/svn/microcontroller/src-atmel/lib/i2c/trunk/usi-twi_slave/
	
	15 Apr 2013, several updated by olikraus@gmail.com


//############################################################################*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "usi_twi_slave.h"

//############################################################### device defines

#if	defined( __AVR_ATtiny2313__ )
		#define DDR_USI             DDRB
		#define PORT_USI            PORTB
		#define PIN_USI             PINB
		#define PORT_USI_SDA        PB5
		#define PORT_USI_SCL        PB7
		#define PIN_USI_SDA         PINB5
		#define PIN_USI_SCL         PINB7
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
		
#elif   defined( __AVR_ATtiny24A__ ) | \
        defined( __AVR_ATtiny44A__ ) | \
        defined( __AVR_ATtiny24__ )  | \
        defined( __AVR_ATtiny44__ )  | \
        defined( __AVR_ATtiny84__ )
		#define DDR_USI             DDRA
		#define PORT_USI            PORTA
		#define PIN_USI             PINA
		#define PORT_USI_SDA        PA6
		#define PORT_USI_SCL        PA4
		#define PIN_USI_SDA         PINA6
		#define PIN_USI_SCL         PINA4
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_STR_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect  

#elif	defined( __AVR_ATtiny25__ ) | \
		defined( __AVR_ATtiny45__ ) | \
		defined( __AVR_ATtiny85__ )
		#define DDR_USI             DDRB
		#define PORT_USI            PORTB
		#define PIN_USI             PINB
		#define PORT_USI_SDA        PB0
		#define PORT_USI_SCL        PB2
		#define PIN_USI_SDA         PINB0
		#define PIN_USI_SCL         PINB2
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect

#elif	defined( __AVR_ATtiny26__ )
		#define DDR_USI             DDRB
		#define PORT_USI            PORTB
		#define PIN_USI             PINB
		#define PORT_USI_SDA        PB0
		#define PORT_USI_SCL        PB2
		#define PIN_USI_SDA         PINB0
		#define PIN_USI_SCL         PINB2
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_STRT_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect

#elif	defined( __AVR_ATtiny261__ ) | \
		defined( __AVR_ATtiny461__ ) | \
		defined( __AVR_ATtiny861__ )
		#define DDR_USI             DDRB
		#define PORT_USI            PORTB
		#define PIN_USI             PINB
		#define PORT_USI_SDA        PB0
		#define PORT_USI_SCL        PB2
		#define PIN_USI_SDA         PINB0
		#define PIN_USI_SCL         PINB2
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVF_vect

#elif	defined( __AVR_ATmega165__ ) | \
		defined( __AVR_ATmega325__ ) | \
		defined( __AVR_ATmega3250__ ) | \
		defined( __AVR_ATmega645__ ) | \
		defined( __AVR_ATmega6450__ ) | \
		defined( __AVR_ATmega329__ ) | \
		defined( __AVR_ATmega3290__ )
		#define DDR_USI             DDRE
		#define PORT_USI            PORTE
		#define PIN_USI             PINE
		#define PORT_USI_SDA        PE5
		#define PORT_USI_SCL        PE4
		#define PIN_USI_SDA         PINE5
		#define PIN_USI_SCL         PINE4
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect

#elif	defined( __AVR_ATmega169__ )
		#define DDR_USI             DDRE
		#define PORT_USI            PORTE
		#define PIN_USI             PINE
		#define PORT_USI_SDA        PE5
		#define PORT_USI_SCL        PE4
		#define PIN_USI_SDA         PINE5
		#define PIN_USI_SCL         PINE4
		#define USI_START_COND_INT  USISIF
		#define USI_START_VECTOR    USI_START_vect
		#define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
		
#else
		#error "no USI-Slave definition for MCU available"

#endif

//############################################## functions implemented as macros

#define SET_USI_TO_SEND_ACK( ) 	{ USIDR = 0; \
								DDR_USI |= ( 1 << PORT_USI_SDA ); \
								USISR = ( 0 << USI_START_COND_INT ) | \
								( 1 << USIOIF ) | ( 1 << USIPF ) | \
								( 1 << USIDC )| \
								( 0x0E << USICNT0 );}

								// Prepare ACK 
								// Set SDA as output 
								// Clear all interrupt flags, except Start Cond 
								// Set USI counter to shift 1 bit

#define SET_USI_TO_READ_ACK( ) 	{ USIDR = 0; \
								DDR_USI &= ~( 1 << PORT_USI_SDA ); \
								USISR = ( 0 << USI_START_COND_INT ) | \
								( 1 << USIOIF) | \
								( 1 << USIPF ) | \
								( 1 << USIDC ) | \
								( 0x0E << USICNT0 );}

								// Set SDA as input 
								// Prepare ACK 
								// Clear all interrupt flags, except Start Cond 
								// Set USI counter to shift 1 bit 

#define SET_USI_TO_TWI_START_CONDITION_MODE( ) { \
								USICR = ( 1 << USISIE ) | ( 0 << USIOIE ) | \
								( 1 << USIWM1 ) | ( 0 << USIWM0 ) | \
								( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
								( 0 << USITC ); \
								USISR = ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
								( 1 << USIDC ) | ( 0x0 << USICNT0 ); }

								// Enable Start Condition Interrupt, disable Overflow Interrupt 
								// Set USI in Two-wire mode, no USI Counter overflow hold 
								// Shift Register Clock Source = External, positive edge 
								// 4-Bit Counter Source = external, both edges 
								// No toggle clock-port pin 
								// Clear all interrupt flags, except Start Cond 

#define SET_USI_TO_SEND_DATA( ) { DDR_USI |=  ( 1 << PORT_USI_SDA ); \
								USISR = ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
								( 1 << USIDC) | \
								( 0x0 << USICNT0 ); \
								}

								// Set SDA as output 
								// Clear all interrupt flags, except Start Cond 
								// Set USI to shift out 8 bits 

#define SET_USI_TO_READ_DATA( ) { DDR_USI &= ~( 1 << PORT_USI_SDA ); \
								USISR =	( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | \
								( 1 << USIPF ) | ( 1 << USIDC ) | \
								( 0x0 << USICNT0 ); \
								}

								// Set SDA as input 
								// Clear all interrupt flags, except Start Cond 
								// Set USI to shift out 8 bits 

#define cbi(ADDRESS,BIT) 	((ADDRESS) &= ~(1<<(BIT)))	// clear Bit

//##################################################################### typedefs


#define USI_SLAVE_CHECK_ADDRESS				0x00
#define USI_SLAVE_SEND_DATA					0x01
#define USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA	0x02
#define USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA	0x03
#define USI_SLAVE_REQUEST_DATA					0x04
#define USI_SLAVE_GET_DATA_AND_SEND_ACK			0x05


//############################################################## local variables

volatile uint8_t usi_overflow_state;

volatile uint8_t usi_memory[TWI_BUFFER_SIZE];			// Buffer to write/read data received from the master
volatile uint8_t usi_mem_adr = 0; 					// Virtual buffer address register


//############################################ initialize USI for TWI slave mode

void usi_twi_slave_init(void)
{
	// In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
	// low when a start condition is detected or a counter overflow (only
	// for USIWM1, USIWM0 = 11).  This inserts a wait state. SCL is released
	// by the ISRs (USI_START_vect and USI_OVERFLOW_vect).
	DDR_USI |= (1 << PORT_USI_SCL) | (1 << PORT_USI_SDA);	// Set SCL and SDA as output
	PORT_USI |= (1 << PORT_USI_SCL);	// Set SCL high
	PORT_USI |= (1 << PORT_USI_SDA);	// Set SDA high
	DDR_USI &= ~(1 << PORT_USI_SDA);	// Set SDA as input
	USICR =
		(1 << USISIE) |					// Enable Start Condition Interrupt
		(0 << USIOIE) |					// Disable Overflow Interrupt
		(1 << USIWM1) | (0 << USIWM0) |	// Set USI in Two-wire mode, no USI Counter overflow hold
		(1 << USICS1) | (0 << USICS0) | (0 << USICLK) |// Shift Register Clock Source = external, positive edge 4-Bit Counter Source = external, both edges
		(0 << USITC);					// No toggle clock-port pin
	USISR = (1 << USI_START_COND_INT) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);  // clear all interrupt flags and reset overflow counter
}

//###################################################### USI Start Condition ISR

ISR(USI_START_VECTOR)
{
  usi_overflow_state = USI_SLAVE_CHECK_ADDRESS;			// Set default starting conditions for new TWI package
  DDR_USI &= ~(1 << PORT_USI_SDA);				// Set SDA as input

  // Wait for SCL to go low to ensure the Start Condition has completed (the
  // Start detector will hold SCL low ) - if a Stop Condition arises then leave
  // The interrupt to prevent waiting forever - don't use USISR to test for Stop
  // Condition as in Application Note AVR312 because the Stop Condition Flag is
  // going to be set from the last TWI sequence

  while ((PIN_USI & (1 << PIN_USI_SCL)) && !((PIN_USI & (1 << PIN_USI_SDA))))
    ;// SCL is high and SDA is low

  if (!(PIN_USI & (1 << PIN_USI_SDA)))
  {	// A Stop Condition did not occur
    USICR =
      (1 << USISIE) |								// Keep Start Condition Interrupt enabled to detect RESTART
      (1 << USIOIE) |								// Enable Overflow Interrupt
      (1 << USIWM1) | (1 << USIWM0) |			// Set USI in Two-wire mode, hold SCL low on USI Counter overflow
      (1 << USICS1) | (0 << USICS0) | (0 << USICLK) |	// 4-Bit Counter Source = external, both edges; Clock Source = External, positive edge	
      (0 << USITC);									// No toggle clock-port pin
  }
  else 
  {	// A Stop Condition did occur
    USICR =
      (1 << USISIE) |								// Enable Start Condition Interrupt
      (0 << USIOIE) |								// Disable Overflow Interrupt
      (1 << USIWM1) | (0 << USIWM0) |				// Set USI in Two-wire mode, no USI Counter overflow hold
      (1 << USICS1) | (0 << USICS0) | (0 << USICLK) |		// 4-Bit Counter Source = external, both edges; Clock Source = external, positive edge
      (0 << USITC);									// No toggle clock-port pin
  } 

  USISR =
    (1 << USI_START_COND_INT) | (1 << USIOIF) |	// Clear interrupt flags - resetting the Start Condition Flag will release SCL
    (1 << USIPF) |(1 << USIDC) |
    (0x0 << USICNT0);								// Set USI to sample 8 bits (count 16 external SCL pin toggles)
}

//################################################### ISR( USI_OVERFLOW_VECTOR )

ISR(USI_OVERFLOW_VECTOR)	// Handles all the communication. Only disabled when waiting for a new Start Condition.
{
  if (USISR & _BV(USIPF)) 
  {
    // stop conditions detected 
    usi_twi_slave_init();
  }
  else 
  {
    uint8_t data = 0;
    switch (usi_overflow_state)
    {
      
      //###### Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, else reset USI
      case USI_SLAVE_CHECK_ADDRESS:
	data = USIDR;
	if (data == 0 || (data>>1) == TWI_ADDRESS)		// If adress is either 0 or own address
	{
	  if (data & 0x01)
	  {
	    usi_overflow_state = USI_SLAVE_SEND_DATA;		// Master Write Data Mode - Slave transmit
	  } 
	  else
	  {
	    usi_overflow_state = USI_SLAVE_REQUEST_DATA;		// Master Read Data Mode - Slave receive
	    usi_mem_adr = 0xFF; 			// Buffer position undefined, ok: also indicates, that the first data byte contains the address
	  } // end if
	  SET_USI_TO_SEND_ACK();
	} 
	else
	{
	  SET_USI_TO_TWI_START_CONDITION_MODE();
	}
	break;

	//###################################### Master Write Data Mode - Slave transmit
	    // Check reply and goto USI_SLAVE_SEND_DATA if OK, 
	    // else reset USI
      case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
	      if (USIDR)
	      {
		      SET_USI_TO_TWI_START_CONDITION_MODE();	// If NACK, the master does not want more data
		      return;
	      }

	    // From here we just drop straight into USI_SLAVE_SEND_DATA if the master sent an ACK
      case USI_SLAVE_SEND_DATA:
	if (usi_mem_adr == 0xFF)		// No buffer position given, set buffer address to 0
	{
		usi_mem_adr = 0;
	}
	USIDR = usi_memory[usi_mem_adr];	// Send data byte
	if (usi_mem_adr < TWI_BUFFER_SIZE)
		usi_mem_adr++;			// Increment buffer address for next byte

	usi_overflow_state = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
	SET_USI_TO_SEND_DATA();
	break;

      // Set USI to sample reply from master
      // Next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
      case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
	usi_overflow_state = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
	SET_USI_TO_READ_ACK();
	break;




    //######################################## Master Read Data Mode - Slave receive

      // Set USI to sample data from master,
      // Next USI_SLAVE_GET_DATA_AND_SEND_ACK
      case USI_SLAVE_REQUEST_DATA:
	      usi_overflow_state = USI_SLAVE_GET_DATA_AND_SEND_ACK;
	      SET_USI_TO_READ_DATA();
	      break;

      // Copy data from USIDR and send ACK
      // Next USI_SLAVE_REQUEST_DATA
      case USI_SLAVE_GET_DATA_AND_SEND_ACK:
	data = USIDR;					// Read data received
	if (usi_mem_adr == 0xFF)		// First access, read buffer position
	{
		if (data < TWI_BUFFER_SIZE)		// Check if address within buffer size
		{
			usi_mem_adr = data;		// Set position as received
		} 
		else
		{
			usi_mem_adr = 0;			// Set address to 0
		}				
	} 
	else 							// Ongoing access, receive data
	{
		usi_memory[usi_mem_adr] = data;				// Write data to buffer
		if (usi_mem_adr < TWI_BUFFER_SIZE)
		  usi_mem_adr++;							// Increment buffer address for next write access
	}
	usi_overflow_state = USI_SLAVE_REQUEST_DATA;	// Next USI_SLAVE_REQUEST_DATA
	SET_USI_TO_SEND_ACK();
	break;
    }// End switch
  }
}// End ISR( USI_OVERFLOW_VECTOR )

