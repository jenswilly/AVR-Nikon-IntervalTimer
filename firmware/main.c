/*	NEXT STEPS:
 *
 *	1)	UART comm -> FTDI
 *		keep writing test string, then delay
 *
 *	√	Sleep
 *		Go to sleep/idle/power-down after USB disconnect
 */

/*
 *  main.c
 *  5110_test
 *
 *  Created by Jens Willy Johannsen on 25-07-11.
 *  Copyright Greener Pastures 2011. All rights reserved.
 *
 *
 *	Hardware connections:
 *	PD2/pin4/dig2	-> 5110 SCE (SS)
 *	PD3/pin5/dig3	-> 5110 D/C
 *	PD7/pin13/dig7	-> 5110 RESET
 *	PB5/pin19/dig13	-> 5110 SCLK
 *	PB4/pin18/dig12	-> MISO
 *	PB3/pin17/dig11	-> 5110 SDIN
 *	PB2/pin16		-> 5110 LED (direct)
 *	PB1/pin15		-> FTDI PWREN#
 *	+ PWR, GND and 3V3 -> VLED
 */


// Define to 1 to use PRR power reduction before sleep; 0 to not power-down stuff
#define POWERREDU 0


/* Precompiler stuff for calculating USART baud rate value 
 * Otherwise, use this page: http://www.wormfood.net/avrbaudcalc.php?postbitrate=9600&postclock=12&bit_rate_table=on
 */
#define USART_BAUDRATE 57600	// error 0.2%
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "5110LCD.h"
#include "images.h"

// Prototypes
void enable_spi();
void sleep();
void setup_ports();
void setup_interrupts();
void enable_serial();
void serialWriteByte( char data );
void serialWriteString( const char* string );

// Commands
typedef enum
{
	CMD_NOP,
	CMD_STATUS
} CommandMode;

// Global vars
unsigned char SLEEP_ALLOWED = 0;
volatile unsigned char SLEEPING = 0;
unsigned char usartBuffer[20];				// USART receive buffer
volatile unsigned int usartPtr=0;			// USART buffer pointer
volatile CommandMode nextCommand = CMD_NOP;	// Next command mode

// Interrupt handler for USART receive complete
ISR( USART_RX_vect ) 
{ 
	// Grab the data and but it into the buffer and increase pointer
	usartBuffer[ usartPtr++ ] = UDR0;

	// TMP: Echo
	serialWriteByte( usartBuffer[ usartPtr-1 ] );

	// Check for "terminate command" bute (0x0D, \r, CR, ^M)
	if( usartBuffer[ usartPtr-1 ] == 0x0D )
	{
		// Complete command received: parse it
		switch( usartBuffer[0] )
		{
			// Status
			case 's':
				nextCommand = CMD_STATUS;
				break;
				
			// Unknown command
			default:
				break;
		}
		
		// Reset buffer
		usartPtr = 0;
		
		// Receipt
		serialWriteString( "<\r\n" );
	}
	else if( usartBuffer[ usartPtr-1 ] == 27 )
	{
		// Escape: clear buffer
		usartPtr = 0;

		// Receipt
		serialWriteString( "*\r\n" );
	}
	
}

// Interrupt handler for PCINT1
// This is used for PCINT12: PWREN# from USB connect/disconnect
ISR(PCINT1_vect)
{
	// Re-enable SPI if we were sleeping
	if( SLEEPING )
	{
		enable_spi();
		enable_serial();
		
		LCD_init();

		SLEEPING = 0;
	}
	
	// Debounce
	_delay_ms( 20 );
	
	// Is PWREN# low?
	if( !(PINC & (1<<PC3)) )
	{
		// Yes: USB connected
		SLEEP_ALLOWED = 0;
		
		// Power up 5110 LED
		PORTB |= (1<<PB2);
		
		// Clear LCD and draw image
		LCD_clear();
		LCD_drawImage( nuke );
	}
	else
	{
		// No: USB disconnected
		SLEEP_ALLOWED = 1;
		
		// Clear and set text
		LCD_clear();
		LCD_writeString_F( "USB disconn." );
		
		// Wait a sec
		_delay_ms( 1000 );
		
		// Switch off LED
		PORTB &= ~(1<<PB2);
		
		// Go to sleep
		// ...
	}
}

void setup_ports()
{
	// Initialize ports for SPI
	DDRB |= _BV( PB2 ) | _BV( PB3 ) | _BV( PB5 );	// SS, MOSI and SCLK -> output
	DDRB &= ~(1<<PB4);	// MISO input

	// 5110 ports
	DDRD |= _BV( PD2 ) | _BV( PD3 ) | _BV( PD7 );	// SCE, RESET and D/C -> output
	DDRB |= (1<<PB2);								// 5110_LED output
	PORTB &= ~(1<<PB2);								// LED off
	
	// USB connected interrupt
	PORTC |= _BV(PC3);		// pull-up PC4/PCINT12
}

void setup_interrupts()
{
	// Enable pin-change interrupt on USB PWREN# (PC3/PCINT11)
	PCICR |= _BV(PCIE1);	//Enable PCINT1
	PCMSK1 |= _BV(PCINT11);	//Trigger on change of PCINT11 (PC3)
	
	// Enable USART RX interrupt
	UCSR0B |= (1<< RXCIE0);	// Enable USART RX interrupt
}

// Enables USART comm
// Remember to call this method after waking up
void enable_serial()
{
#if POWERREDU
	PRR &= ~(1<< PRUSART0);		// Power to the USART
#endif
	UCSR0B |= (1<< RXEN0) | (1<< TXEN0);			// Enable TX and RX
//	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);		// 8N1 serial format. Set by default
	UBRR0H = (BAUD_PRESCALE >> 8) & 0x0F;			// High part of baud rate masked so bits 15:12 are writted as 0 as per the datasheet (19.10.5)
	UBRR0L = BAUD_PRESCALE;							// Low part of baud rate. Writing to UBBRnL updates baud prescaler

}

// Call this method to initialize SPI modules.
// Remember to call this method after waking up
void enable_spi()
{
	// Init SPI
#if POWERREDU
	PRR &= ~(1<< PRSPI);	// Power to the SPI module
#endif
	SPCR = 0x52;			// SPI enable, master, sample on leading edge, rising, rate=Fosc/64
	SPSR = 0;				// not 2x data rate
}

void serialWriteByte( char data )
{
	// Wait until we're ready to send
	while( !(UCSR0A & (1<< UDRE0)))
		;
	
	// Send byte
	UDR0 = data;
}

void serialWriteString( const char* string )
{
	while( *string )
		serialWriteByte( *string++ );
}

// Call this method to go to sleep
void sleep()
{
	// Set "sleep" display
	LCD_clear();
	LCD_writeString_F( "     --     " );
	
	SMCR |= (1<< SM1) | (1<< SE);	// sleep-mode=power-down, enable sleep
#if POWERREDU
	// No, don't power off stuff. It won't matter (in tens of µAmps, at least) but it *will* mess up SPI comm
	PRR = 0xEF;						// turn everything off (actually, we don't need to switch everything off since power-down will turn off lots of stuff by itself)
#endif
	
	SLEEPING = 1;
	asm("sleep");					// nighty, night.
}

int main(void)
{
	setup_ports();
	setup_interrupts();
	
	// Init SPI
	enable_spi();
	
	// Initialize LCD
	LCD_init();

	// Enable interrupts
	sei();

	// OK to go to sleep
	SLEEP_ALLOWED = 1;
	
	// Main loop
	for( ;; )
	{
		// wait 6 secs, then sleep again
		_delay_ms( 2000 );
		if( SLEEP_ALLOWED )
			sleep();
		
		// We are awake again. 
		// Should we handle a command?
		if( nextCommand == CMD_STATUS )
		{
			serialWriteString( "STATUS command received.\r\n" );
			nextCommand = CMD_NOP;	// we're done
		}
	}

    return 0;
}
