/*
 *  main.c
 *  5110_test
 *
 *  Created by Jens Willy Johannsen on 25-07-11.
 *  Copyright Greener Pastures 2011. All rights reserved.
 *
 */


/* Sleep timer value in seconds
 */
#define SLEEP_TIMER 20

/* Precompiler stuff for calculating USART baud rate value 
 * Otherwise, use this page: http://www.wormfood.net/avrbaudcalc.php?postbitrate=9600&postclock=12&bit_rate_table=on
 */
//#define USART_BAUDRATE 2400	// error 0.2% for 1.5 MHz
#define USART_BAUDRATE 9600	// error 0.2%
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) 

/* I2C clockrate
 */
#define TWI_CLKRATE 100000	// 100 kHz
#define TWI_BITRATE ((F_CPU/TWI_CLKRATE)-16)/2

/* Keypad polling frequency in Hz
 */
#define KEYPAD_POLL_FREQ 128
#define KEYPAD_POLL_TIMER_COMPARE_VALUE ((F_CPU/1024/KEYPAD_POLL_FREQ)-1)
//#define KEYPAD_POLL_TIMER_COMPARE_VALUE 91

/* 16-bit timer output compare value for timeout delay of approx. 1 seconds
 */
#define TIMER_COMPARE_VALUE 11718	// ~5 secs: 12000000/1024/1 - 1 = 11717.75

// Macros
#define RESET_TIMEOUT TCNT1 = 0; sleepTimer=0	// Reset timeout counter macro

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "5110LCD.h"
#include"PCF8563RTC.h"
#include "images.h"
#include "nRF24L01p.h"

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
unsigned char usartBuffer[RX_BUF_SIZE];		// USART receive buffer
volatile unsigned char usartPtr;			// USART buffer pointer
volatile unsigned char nextCommand;			// Next command mode
volatile int sleepTimer;					// Seconds timer for sleep timeout
volatile unsigned char keypad_mask;			// Bitmask for selecting MUX channel
volatile unsigned char rdoBuffer[4];		// Buffer for nRF radio
static char hex_chars[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

// Timer1 compare match interrupt used for timeout to sleep
ISR( TIMER1_COMPA_vect )
{
	// 1 second passed: increase counter
	sleepTimer++;
	
	// Timeout reached: go to sleep if we're allowed to do so
	if( sleepTimer >= SLEEP_TIMER )
	{
		if( sleepAllowed )
			nextCommand = CMD_SLEEP;
		else
			sleepTimer = 0;
	}
}

// Timer0 compare match interrupt used for keypad polling
ISR( TIMER0_COMPA_vect )
{
	// Increase keypad bitmask for selecting MUX channels 0-4
	keypad_mask++;
	if( keypad_mask > 4 )
		keypad_mask = 0;
	
	PORTC &= 0b11111000;	// clear PC0 - PC2
	PORTC |= keypad_mask;	// set current keypad_mask
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
		
		// Wake-up?
		// TODO

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
		// Disable keypad polling
		// ...
		
		// Disable keypad press interrupt
		// ...
		
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
		// Enable keypad polling
		// ...
		
		// Enable keypad press interrupt
		// ...
		
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

// Interrupt handler for pin-change on currently connected input pin from keypad
ISR( PCINT0_vect )
{
	// nRF IRQ?
	if( !(PINB & (1<< PB0)) )
	{
		// Yes: do we have data ready (we should but...)
		if( nRF24L01p_dataReady() )
		{
			nRF24L01p_readData( (uint8_t*)rdoBuffer );
			LCD_clear();
			LCD_writeChar( hex_chars[ rdoBuffer[0] >> 4 ] );
			LCD_writeChar( hex_chars[ rdoBuffer[0] & 0x0F ] );
			LCD_writeChar( '-' );
			LCD_writeChar( hex_chars[ rdoBuffer[1] >> 4 ] );
			LCD_writeChar( hex_chars[ rdoBuffer[1] & 0x0F ] );
			LCD_writeChar( '-' );
			LCD_writeChar( hex_chars[ rdoBuffer[2] >> 4 ] );
			LCD_writeChar( hex_chars[ rdoBuffer[2] & 0x0F ] );
			LCD_writeChar( '-' );
			LCD_writeChar( hex_chars[ rdoBuffer[3] >> 4 ] );
			LCD_writeChar( hex_chars[ rdoBuffer[3] & 0x0F ] );
			LCD_writeChar( '-' );
		}
		else
		{
			LCD_clear();
			LCD_writeString_F( "No RF data." );
		}
		
		// Reset sleep timer and exit
		RESET_TIMEOUT;
		return;
	}
		  
	// Key down or up?
	if( !(PINB & (1<< PB1)) )
	{
		// Key down
		// Stop keypad poll timer and reset any pending timer interrupt
		TCCR0B = 0;
		TIFR0 |= OCF0A;
		
		// Debounce
		cli();
		_delay_ms( 10 );
		sei();
		
		if( (PINB & (1<< PB1)) )
		{
			// Debounce failed: restart keypad poll timer and exit
			TCCR0B = (1<< CS02) | (1<< CS00);	// Start clock with prescaler 1024
			return;
		}
			
		// Make sure LED is on
		PORTB |= (1<<PB2);
		
		// Which button was pressed?
		switch( keypad_mask )
		{
			case 0x00:
				// Center button
				LCD_writeChar( '@' );
				break;
				
			case 0x01:
				// Up
				LCD_writeChar( '=' );
				break;
				
			case 0x02:
				// Down
				LCD_writeChar( '>' );
				break;
				
			case 0x03:
				// Left
				LCD_writeChar( '<' );
				break;
				
			case 0x04:
				// Right
				LCD_writeChar( ';' );
				break;
		}
		
		// Reset sleep timer
		RESET_TIMEOUT;
	}
	else if( (PINB & (1<< PB1)) )
	{
		// Key up
		// Restart keypad poll timer
		TCCR0B = (1<< CS02) | (1<< CS00);	// Start clock with prescaler 1024
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
	
	// Keypad MUX select pins -> output
	DDRC |= (1<< PC0) | (1<<PC1) | (1<<PC2);	
	
	// External interrupts -> input (set by default)
	PORTC |= (1<< PC3);		// pull-up PC3/PCINT11: interrupt on USB PWREN#
	PORTD |= (1<< PD6);		// pull-up PD6/PCINT22: interrupt on alarm from RTC
	PORTB |= (1<< PB0);		// pull-up PB0/PCINT0: nRF24L01+ IRQ input pin
	PORTB |= (1<< PB1);		// pull-up PB1/PCINT1: keypad input pin
	
	// Set pull-ups on I2C pins
	PORTC |= _BV( PC4 ) | _BV( PC5 );
}

void setup_interrupts()
{
	// Enable pin-change interrupt on USB PWREN# (PC3/PCINT11)
	PCICR |= (1<< PCIE0) | (1<< PCIE1) | (1<< PCIE2);	// Enable PCINT0, PCINT1 and PCINT2
	PCMSK0 = (1<< PCINT0) | (1<< PCINT1);				// Trigger on change of PCINT0 (PB0/pin14) and PCINT1 (PB1/pin15)
	PCMSK1 = (1<< PCINT11);								// Trigger on change of PCINT11 (PC3/pin26)
	PCMSK2 = (1<< PCINT22);								// Trigger on change of PCINT22 (PD6/pin12)
	
	// Enable USART RX interrupt
	UCSR0B |= (1<< RXCIE0);	// Enable USART RX interrupt
}

// Enables USART comm
// Remember to call this method after waking up
void enable_serial()
{
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
	SPCR = 0x52;			// SPI enable, master, sample on leading edge, rising, rate=Fosc/64
	SPSR = 0;				// not 2x data rate
}

void enable_i2c()
{
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
	
	
	// Select MUX to Y0 for center key
	keypad_mask = 0;
	PORTC &= 0b11111000;
	
	SMCR |= (1<< SM1) | (1<< SE);	// sleep-mode=power-down, enable sleep
	
	asm("sleep");					// nighty, night.
}

void setupTimeoutCounter()
{
	TCCR1A = 0;							// Normal mode, output compare pins disabled	
	TCCR1B = (1<< WGM12) | (1<< CS12) | (1<< CS10);	// CTC, top at OCR1A, prescaler 1024
	OCR1A = TIMER_COMPARE_VALUE;		// Set timer compare value
	TIMSK1 |= (1<< OCIE1A);				// Enable timer1 output compare match interrupt
	
	// Timer0 (8bit) for keypad polling
	TCCR0A = (1<< WGM01);				// Mode 2: CTC
	TCCR0B = (1<< CS02) | (1<< CS00);	// Start clock with prescaler 1024
	OCR0A = KEYPAD_POLL_TIMER_COMPARE_VALUE;
	TIMSK0 |= (1<< OCIE0A);				// Enable timer0 output compare match interrupt
	
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

	// Setup radio
	nRF24L01p_init();
	nRF24L01p_startRX();
	
	// OK to go to sleep
	sleepAllowed = 1;
	
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

		_delay_ms( 100 );
	}

    return 0;
}
