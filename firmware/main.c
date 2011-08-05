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
//#define USART_BAUDRATE 2400	// error 0.2% for 1.5 MHz
#define USART_BAUDRATE 9600	// error 0.2%
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) 

#define TWI_CLKRATE 100000	// 100 kHz
#define TWI_BITRATE ((F_CPU/TWI_CLKRATE)-16)/2

/* 16-bit timer output compare value for timeout delay of approx. 5 seconds
 */
#define TIMER_COMPARE_VALUE 0xe4e1	// ~5 secs: 12000000/1024/0.2 - 1 = 58593
// #define TIMER_COMPARE_VALUE 1500000/1024/0.2 - 1 = 7323 for 1.5 MHz

// Macros
#define RESET_TIMEOUT TCNT1 = 0	// Reset timeout counter macro

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "5110LCD.h"
#include"PCF8563RTC.h"
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
#define	CMD_NOP 0
#define	CMD_SLEEP 1
#define	CMD_STATUS 2		// s
#define	CMD_CLOCKSET 3		// zYYMMDDW-HHMMSS
#define	CMD_CLOCKREAD 4		// q
#define CMD_ALARMSET 5		// aDDHHMM

// Global vars
#define RX_BUF_SIZE	20
unsigned char sleepAllowed;
unsigned char usartBuffer[RX_BUF_SIZE];				// USART receive buffer
volatile unsigned char usartPtr;			// USART buffer pointer
volatile unsigned char nextCommand;			// Next command mode

// Timer1 compare match interrupt used for timeout to sleep
ISR( TIMER1_COMPA_vect )
{
	// Timeout reached: go to sleep if we're allowed to do so
	if( sleepAllowed )
		nextCommand = CMD_SLEEP;
}

// Interrupt handler for USART receive complete
ISR( USART_RX_vect ) 
{ 
	// Prevent buffer overflow
	if( usartPtr >= RX_BUF_SIZE-1 )
		usartPtr = 0;
	
	// (We needn't worry about sleeping here, since we're in an interrupt handler)
	
	// Grab the data and but it into the buffer and increase pointer
	usartBuffer[ usartPtr++ ] = UDR0;

	// TMP: Echo
	serialWriteByte( usartBuffer[ usartPtr-1 ] );
	
	// Check for "terminate command" bute (0x0D, \r, CR, ^M)
	if( usartBuffer[ usartPtr-1 ] == 0x0D )
	{
		if( usartBuffer[0] == 's' )
		{
			nextCommand = CMD_STATUS;
			usartPtr = 0;
			return;
		}
		if( usartBuffer[0] == 'q' )
		{
			nextCommand = CMD_CLOCKREAD;
			usartPtr = 0;
			return;
		}
		if( usartBuffer[0] == 'z' )
		{
			nextCommand = CMD_CLOCKSET;
			usartPtr = 0;
			return;
		}
		if( usartBuffer[0] == 'a' )
		{
			nextCommand = CMD_ALARMSET;
			usartPtr = 0;
			return;
		}

		// Reset buffer
		usartPtr = 0; 
	}
}

// Interrupt handler for PCINT2
// This is used for PCINT22: external interrupt from RTC
ISR(PCINT2_vect)
{
	_delay_ms( 20 ); // debounce
	
	// Since this interrupt will trigger on both high->low and low->high, we need to check if it was a high->low change
	if( !(PIND & (1<<PD6)) )
	{
		// Yes: external alarm triggered
		
		// LED on
		PORTB |= (1<<PB2);
		LCD_drawImage( nuke );

		RESET_TIMEOUT;
	}
}

// Interrupt handler for PCINT1
// This is used for PCINT12: PWREN# from USB connect/disconnect
ISR(PCINT1_vect)
{
	// Debounce
	_delay_ms( 20 );
	
	// Is PWREN# low?
	if( !(PINC & (1<<PC3)) )
	{
		// Yes: USB connected
		sleepAllowed = 0;
		
		// Power up 5110 LED
		PORTB |= (1<<PB2);
		
		// Clear LCD and draw image
		LCD_clear();
		LCD_writeString_F( "USB connect" );
		
		// And clear USART buffer
		usartPtr = 0;
	}
	else
	{
		// No: USB disconnected
		sleepAllowed = 1;
		RESET_TIMEOUT;
		
		// Clear and set text
		LCD_clear();
		LCD_writeString_F( "USB disconn." );
		
		// Wait a sec
		_delay_ms( 2000 );
		
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
	
	// External interrupts -> input (set by default)
	PORTC |= (1<< PC3);		// pull-up PC3/PCINT11: interrupt on USB PWREN#
	PORTD |= (1<< PD6);		// pull-up PD6/PCINT22: interrupt on alarm from RTC
	
	// Set pull-ups on I2C pins
	PORTC |= _BV( PC4 ) | _BV( PC5 );
}

void setup_interrupts()
{
	// Enable pin-change interrupt on USB PWREN# (PC3/PCINT11)
	PCICR |= (1<< PCIE1) | (1<< PCIE2);		// Enable PCINT1 and PCINT2
	PCMSK1 = (1<< PCINT11);					// Trigger on change of PCINT11 (PC3/pin26)
	PCMSK2 = (1<< PCINT22);					// Trigger on change of PCINT22 (PD6/pin12)
	
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

void enable_i2c()
{
#if POWERREDU
	PRR &= ~(1<< PRTWI);
#endif
	// Set prescaler and bit rate for 100 kHz
	// SCL frequency = 12000000 / (16 + 2 * <52> * <1>) = 100 khz
	TWSR = 0x00;			// Select Prescaler of 1
	TWBR = TWI_BITRATE;		// Bit rate = 52 for 100 kHz
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
	// Power off LED
	PORTB &= ~(1<<PB2);
	
	// Set "sleep" display
	LCD_clear();
	LCD_writeString_F( "     --     " );
	_delay_ms( 200 );
	
	SMCR |= (1<< SM1) | (1<< SE);	// sleep-mode=power-down, enable sleep
#if POWERREDU
	PRR = 0xEF;						// turn everything off (actually, we don't need to switch everything off since power-down will turn off lots of stuff by itself)
#endif
	
	asm("sleep");					// nighty, night.
}

void setupTimeoutCounter()
{
#if POWERREDU
	PRR &= ~(1<< PRTIM1);		// Power-up timer1
#endif
	TCCR1A = 0;							// Normal mode, output compare pins disabled	
	TCCR1B = (1<< WGM12) | (1<< CS12) | (1<< CS10);	// CTC, top at OCR1A, prescaler 1024
	OCR1A = TIMER_COMPARE_VALUE;		// Set timer compare value
	TIMSK1 |= (1<< OCIE1A);				// Enable timer1 output compare match interrupt
}



int main(void)
{
	usartPtr = 0;
	nextCommand = CMD_NOP;
	
	unsigned char year, month, day, weekDay, hour, minute, second;
	
	setup_ports();
	
	// Init SPI, USART and TWI
	enable_spi();
	enable_serial();
	enable_i2c();
	
	// Initialize LCD
	LCD_init();
	_delay_ms( 200 );
	LCD_writeString_F( "Ready" );
	
	// Initialize RTC
	initRTC();
	
	// Setup timeout counter
	setupTimeoutCounter();
	
	// Enable interrupts
	setup_interrupts();
	sei();

	// OK to go to sleep
	sleepAllowed = 0;
	
	// Main loop
	for( ;; )
	{
		// Should we handle a command?

		if( nextCommand == CMD_STATUS )
		{
			serialWriteString( "\r\nSTATUS command received.\r\n" );
			nextCommand = CMD_NOP;
			RESET_TIMEOUT;
		}
		if( nextCommand == CMD_SLEEP )
		{
			nextCommand = CMD_NOP;
			sleep();
		}
		if( nextCommand == CMD_CLOCKREAD )
		{
			nextCommand = CMD_NOP;
			// read clock and output on serial
			readRTCClock( &year, &month, &day, &weekDay, &hour, &minute, &second );
			serialWriteString( "Clock:" );
			serialWriteByte( '0' + hour/10 );
			serialWriteByte( '0' + hour%10 );
			serialWriteByte( ':' );
			serialWriteByte( '0' + minute/10 );
			serialWriteByte( '0' + minute%10 );
			serialWriteByte( ':' );
			serialWriteByte( '0' + second/10 );
			serialWriteByte( '0' + second%10 );
			serialWriteString( "\r\n" );
		}
		if( nextCommand == CMD_CLOCKSET )
		{
			nextCommand = CMD_NOP;
			
			// Parse format: zYYMMDDW-HHMMSS (W=weekday, Sunday=0)
			year = ((usartBuffer[1] - '0') * 10) + (usartBuffer[2] - '0');
			month = ((usartBuffer[3] - '0') * 10) + (usartBuffer[4] - '0');
			day = ((usartBuffer[5] - '0') * 10) + (usartBuffer[6] - '0');
			weekDay = (usartBuffer[7] - '0');
			hour = ((usartBuffer[9] - '0') * 10) + (usartBuffer[10] - '0');
			minute = ((usartBuffer[11] - '0') * 10) + (usartBuffer[12] - '0');
			second = ((usartBuffer[13] - '0') * 10) + (usartBuffer[14] - '0');

			setRTCClock( year, month, day, weekDay, hour, minute, second );
			serialWriteString( "\r\nTime set\r\n" );
		}
		if( nextCommand == CMD_ALARMSET )
		{
			nextCommand = CMD_NOP;
			
			// Parse format: aDDHHMM
			day = ((usartBuffer[1] - '0') * 10) + (usartBuffer[2] - '0');
			hour = ((usartBuffer[3] - '0') * 10) + (usartBuffer[4] - '0');
			minute = ((usartBuffer[5] - '0') * 10) + (usartBuffer[6] - '0');
			
			setRTCAlarm( day, hour, minute );
			serialWriteString( "\r\nAlarm set\r\n" );
		}
		/*
		if( nextCommand == CMD_CLOCKSET )
		{
			nextCommand = CMD_NOP;
			// set clock to 2011-07-30 Saturday 20:35:00
			//setRTCClock( 11, 07, 30, 6, 20, 35, 00 );
			serialWriteString( "\r\nTime set to 2011-07-30 (Sat) 20:35:00\r\n" );
		}
		 */
		/*
		else if( nextCommand == CMD_CLOCKREAD )
		{
			nextCommand = CMD_NOP;
			// read clock and output on serial
			//readRTCClock( &year, &month, &day, &weekDay, &hour, &minute, &second );
			serialWriteString( "Clock read\r\n" );
		}
		 */
		 _delay_ms( 100 );
	}

    return 0;
}
