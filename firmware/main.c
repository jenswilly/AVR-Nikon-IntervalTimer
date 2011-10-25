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
#define TIMER_COMPARE_VALUE ((F_CPU/1024)-1)
// #define TIMER_COMPARE_VALUE 11718	// pre-calculated for 12 MHz: 12000000/1024/1 - 1 = 11717.75
// #define TIMER_COMPARE_VALUE 7812	// pre-calculated for 8 MHz

// Macros
#define RESET_TIMEOUT TCNT1 = 0; sleepTimer=0	// Reset timeout counter macro

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "main.h"
#include "5110LCD.h"
#include"PCF8563RTC.h"
#include "images.h"
#include "nRF24L01p.h"
#include "states.h"

// Commands
#define	CMD_NOP			0
#define	CMD_SLEEP		1
#define	CMD_STATUS		2	// s
#define	CMD_CLOCKSET	3	// zYYMMDDW-HHMMSS
#define	CMD_CLOCKREAD	4	// q
#define CMD_ALARMSET	5	// aHHMM
#define CMD_RDO_OFF		6	// r0
#define CMD_RDO_ON		7	// r1
#define CMD_UPDATE_TIME	8
#define CMD_PROGRAM		9	// p[HHMM|T???],IIII,SSSSS	HHMM = start time; IIII = interval delay in half seconds; SSSSS = number of shots

// Global vars
#define RX_BUF_SIZE	20
unsigned char EEMEM eepromLCD_VOP;
unsigned char sleepAllowed;
unsigned char usartBuffer[RX_BUF_SIZE];		// USART receive buffer
volatile unsigned char usartPtr;			// USART buffer pointer
volatile unsigned char nextCommand;			// Next command mode
volatile int sleepTimer;					// Seconds timer for sleep timeout
volatile unsigned char keypad_mask;			// Bitmask for selecting MUX channel
volatile unsigned char rdoBuffer[4];		// Buffer for nRF radio
static char hex_chars[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
volatile unsigned char lcdOn;
volatile int rdo_sequence = 0;

unsigned char global_mode = 0;

// sequence
volatile int seq_total_shots;
volatile int seq_remain_shots;
volatile int seq_delay;

// Serial stream
static int uart_putchar( char c, FILE *stream );
FILE mystdout = FDEV_SETUP_STREAM( uart_putchar, NULL, _FDEV_SETUP_WRITE );

// LCD stream
static int LCD_putchar( char c, FILE *stream );
FILE lcd = FDEV_SETUP_STREAM( LCD_putchar, NULL, _FDEV_SETUP_WRITE );

// Timer1 compare match interrupt used for timeout to sleep
ISR( TIMER1_COMPA_vect )
{
	// 1 second passed: increase counter
	sleepTimer++;
	
	// Update time in display
	nextCommand = CMD_UPDATE_TIME;	
	
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
		}
		else if( usartBuffer[0] == 'q' )
		{
			nextCommand = CMD_CLOCKREAD;
		}
		else if( usartBuffer[0] == 'z' )
		{
			nextCommand = CMD_CLOCKSET;
		}
		else if( usartBuffer[0] == 'a' )
		{
			nextCommand = CMD_ALARMSET;
		}
		else if( usartBuffer[0] == 'r' )
		{
			if( usartBuffer[1] == '0' )
				nextCommand = CMD_RDO_OFF;
			else if( usartBuffer[1] == '1' )
				nextCommand = CMD_RDO_ON;
		}
		else if( usartBuffer[0] == 'p' )
			nextCommand = CMD_PROGRAM;
		

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
		
		// This interrupt might result in a MCU wake-up. So switch on LCD if it was on
		if( !lcdOn )
		{
			LCD_powerUp();
			lcdOn = 1;
		}
		
		// LCD LED on
		PORTB |= (1<<PB2);
//		LCD_drawImage( nuke );
		
		// Snap it
		trigger();
		global_mode &= ~GLOBAL_TIMED;
		global_mode &= ~GLOBAL_TRIGGERED;

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
		
		// This interrupt might result in a MCU wake-up. So switch on LCD if it was on
		if( !lcdOn )
		{
			LCD_powerUp();
			lcdOn = 1;
		}
		
		// Power up 5110 LED
		PORTB |= (1<<PB2);
		
		// Clear LCD and draw image
		LCD_clear();
		fprintf( &lcd, "USB connect" );
		
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
		fprintf( &lcd, "USB disconn." );
		
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
		// Yes: disable interrupts until we're done with this
		cli();
		
		// This interrupt might result in a MCU wake-up. So switch on LCD if it was on
		if( !lcdOn )
		{
			LCD_powerUp();
			lcdOn = 1;
		}
		
		// LEDs on
		PORTB |= (1<<PB2);
		
		// Do we have data ready (we should but...)
		if( nRF24L01p_dataReady() )
		{
			nRF24L01p_readData( (uint8_t*)rdoBuffer );
			
			LCD_gotoXY( 0, 0 );
			switch( rdoBuffer[0] )
			{
				case 0x17: 
					LCD_writeString_F( "< left     $" );
					break;
					
				case 0x1E:
					LCD_writeString_F( "> down     $" ); 
					break;
					
				case 0x1B: 
					LCD_writeString_F( "; right    $" ); 
					break;
					
				case 0x1D:
					LCD_writeString_F( "= up       $" ); 
					break;
					
				case 0x0F: 
					LCD_writeString_F( "@ select   $" ); 
					trigger();
					break;
					
				default: 
					LCD_writeString_F( "? NOP      $" ); 
					break;
			}
			
			rdo_sequence = rdoBuffer[1] << 8 | rdoBuffer[2];
			LCD_writeChar( hex_chars[ rdo_sequence >> 12 ] );
			LCD_writeChar( hex_chars[ (rdo_sequence >> 8) & 0x0F ] );
			LCD_writeChar( '-' );
			LCD_writeChar( hex_chars[ (rdo_sequence & 0x00F0) >> 4 ] );
			LCD_writeChar( hex_chars[ (rdo_sequence & 0x000F) ] );
		}
		else
		{
			LCD_clear();
			LCD_writeString_F( "No RF data?" );
		}
		
		// Re-enable interrupts
		sei();
		
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
			
		if( !lcdOn )
		{
			LCD_powerUp();
			lcdOn = 1;
		}
		
		// Make sure LED is on
		PORTB |= (1<<PB2);
		
		// Cursor at 0,0
		LCD_gotoXY( 0, 0 );
		
		// Which button was pressed?
		switch( keypad_mask )
		{
			case 0x00:
				// Center button
				// If there's an action, perform that
				if( currentState->actionMethod )
					currentState->actionMethod();
				// Otherwise, switch state if a center state is specified
				else if( currentState->keyCenterState )
					setState( currentState->keyCenterState );
//				LCD_writeString_F( "@ select    " );
				break;
				
			case 0x01:
				// Up
				if( currentState->keyUpState )
					setState( currentState->keyUpState );
//				LCD_writeString_F( "= up        " );
				break;
				
			case 0x02:
				// Down
				if( currentState->keyDownState )
					setState( currentState->keyDownState );
//				LCD_writeString_F( "> down      " );
				break;
				
			case 0x03:
				// Left
				if( currentState->keyLeftState )
					setState( currentState->keyLeftState );
//				LCD_writeString_F( "< left      " );
				break;
				
			case 0x04:
				// Right
				   if( currentState->keyRightState )
				   setState( currentState->keyRightState );
//				LCD_writeString_F( "; right     " );
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
	
	// nRF24L01p ports
	DDRD |= (1<< PD4) | (1<< PD5);					// CSN and CE -> output. IRQ is already input
	
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

static int LCD_putchar( char c, FILE *stream )
{
	LCD_writeChar( c );
	return 0;
}

static int uart_putchar(char c, FILE *stream)
{
	// Wait until we're ready to send
	while( !(UCSR0A & (1<< UDRE0)))
		;
	
	// Send byte
	UDR0 = c;
	return 0;
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
	LCD_powerDown();
	lcdOn = 0;
	
	// Select MUX to Y0 for center key
	keypad_mask = 0;
	PORTC &= 0b11111000;
	
	SMCR |= (1<< SM1) | (1<< SE);	// sleep-mode=power-down, enable sleep
	
	asm("sleep");					// nighty, night.
}

void setup_timers()
{
	// Timer1 (16bit): sleep timeout countdown
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

void trigger()
{
	// Disable interrupts
	cli();
	
	// Set PB1 -> output
	DDRB |= (1<< PB1);
	
	// Set high
//	PORTB |= (1<< PB1);		// Is already high, since we use the pull-up when PB1 is input
	
	// Select MUX channel 5
	PORTC |= (1<< PC2) | (1<< PC0);
	
	// Pull low to activate trigger
	PORTB &= ~(1<< PB1);
	
	// wait a while
	_delay_ms( 100 );
	
	// Set high
	PORTB |= (1<< PB1);
	
	// Set PB1 -> input
	DDRB &= ~(1<< PB1);
	
	// Reset MUX channel to last used keypad input mask
	PORTC &= 0b11111000;	// clear PC0 - PC2
	PORTC |= keypad_mask;	// set current keypad_mask
	
	// Re-enable keypad polling and interrupts
	sei();
}

// Main loop for adjust LCD contrast mode
void adjust_LCD_contrast()
{
	// First, configure timer for keypad polling
	// Timer0 (8bit) for keypad polling
	TCCR0A = (1<< WGM01);				// Mode 2: CTC
	TCCR0B = (1<< CS02) | (1<< CS00);	// Start clock with prescaler 1024
	OCR0A = KEYPAD_POLL_TIMER_COMPARE_VALUE;
	TIMSK0 |= (1<< OCIE0A);				// Enable timer0 output compare match interrupt
	
	// Read existing LCD contrast value
	uint8_t contrast = eeprom_read_byte( &eepromLCD_VOP );
	
	// Show
	LCD_gotoXY(0,0);
	fprintf( &lcd, "LCD contrast" );
	fprintf( &lcd, "            " );
	fprintf( &lcd, "---- %02x ----", contrast );
	fprintf( &lcd, "            " );
	fprintf( &lcd, "------------" );
	fprintf( &lcd, "XXXXXXXXXXXX" );
	
	// Main loop
	for( ;; )
	{
		// MUX channel 01: up
		PORTC &= 0b11111000;
		PORTC |= 0b00000001;
		_delay_ms( 30 );
		if( !(PINB & (1<< PB1)) )
		{
			// UP is pressed: increase contrast
			contrast++;
			
			if( contrast == 0 )
				contrast = 0x80;
		}
		
		// MUX channel 02: down
		PORTC &= 0b11111000;
		PORTC |= 0b00000010;
		_delay_ms( 30 );
		if( !(PINB & (1<< PB1)) )
		{
			// DOWN is pressed: decrease contrast
			contrast--;
			
			if( contrast == 0x7F )
				contrast = 0xFF;
		}

		// Make double-sure that contrast is between 0x80 and 0xFF
		if( contrast >= 0x80 && contrast <= 0xFF )
		{
			// Set VOP
			LCD_writeCommand( 0x21 );  // LCD Extended Commands.
			LCD_writeCommand( contrast );  // Set LCD Vop (Contrast): straight=0xA8, slight up=0xAC
			LCD_writeCommand( 0x20 );  // LCD Standard Commands, Horizontal addressing mode.
			
			// Write to EEPROM
			eeprom_write_byte( &eepromLCD_VOP, contrast );
		}

		// Update display
		LCD_gotoXY( 0, 2 );
		fprintf( &lcd, "---- %02x ----", contrast );
	}
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
	
	// Disable analog comparator and ADC to reduce power
	// NOTE: these will *not* be switched on again since we're not using them
	ACSR = (1<<ACD);		// Turn off Analog Comparator - this removes about 1uA
	PRR |= (1<< PRADC);		// Shut down ADC
	
	// Initialize LCD
	LCD_init();
	
	// Set VOP from EEPROM
	LCD_writeCommand( 0x21 );  // LCD Extended Commands.
	LCD_writeCommand( eeprom_read_byte( &eepromLCD_VOP ));  // Set LCD Vop (Contrast): straight=0xA8, slight up=0xAC
	LCD_writeCommand( 0x20 );  // LCD Standard Commands, Horizontal addressing mode.

	lcdOn = 1;
	_delay_ms( 200 );

	// LCD backlight on
	PORTB |= (1<<PB2);
	
	// Mode check: run or setup LCD contrast?
	// MUX channel is preset to 0: center button
	if( !(PINB & (1<< PB1)) )
	{
		// Button pressed: goto adjust LCD mode
		// NOTE: RTC, timers and interrupts are *not* yet set up
		// NOTE: this method will never return
		adjust_LCD_contrast();
	}
	
	// Normal mode: continue execution
	
	
	// Enter initial state
	currentState = &state_idle;
	currentState->entryMethod();
	
	// Initialize RTC
	initRTC();
	
	// Setup timeout counter
	setup_timers();
	
	// Enable interrupts
	setup_interrupts();
	sei();

	// Setup radio
	nRF24L01p_init();
	nRF24L01p_powerDown();
//	nRF24L01p_startRX();
	
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
		else if( nextCommand == CMD_SLEEP )
		{
			nextCommand = CMD_NOP;
			sleep();
		}
		else if( nextCommand == CMD_CLOCKREAD )
		{
			nextCommand = CMD_NOP;
			// read clock and output on serial
			readRTCClock( &year, &month, &day, &weekDay, &hour, &minute, &second );
			fprintf( &mystdout, "Clock: %02d:%02d:%02d\r\n", hour, minute, second );
/*			serialWriteByte( '0' + hour/10 );
			serialWriteByte( '0' + hour%10 );
			serialWriteByte( ':' );
			serialWriteByte( '0' + minute/10 );
			serialWriteByte( '0' + minute%10 );
			serialWriteByte( ':' );
			serialWriteByte( '0' + second/10 );
			serialWriteByte( '0' + second%10 );
			serialWriteString( "\r\n" );
 */
		}
		else if( nextCommand == CMD_CLOCKSET )
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
		else if( nextCommand == CMD_ALARMSET )
		{
			nextCommand = CMD_NOP;
			
			// Parse format: aHHMM
			hour = ((usartBuffer[1] - '0') * 10) + (usartBuffer[2] - '0');
			minute = ((usartBuffer[3] - '0') * 10) + (usartBuffer[4] - '0');
			
			setRTCAlarm( hour, minute );
			serialWriteString( "\r\nAlarm set\r\n" );
		}
		else if( nextCommand == CMD_RDO_ON  )
		{
			nextCommand = CMD_NOP;
			
			// Radio on
			nRF24L01p_startRX();
			global_mode |= GLOBAL_RDO;
			serialWriteString( "\r\nRadio ON\r\n" );
		}
		else if( nextCommand == CMD_RDO_OFF )
		{
			nextCommand = CMD_NOP;
			
			// Radio off
			nRF24L01p_powerDown();
			global_mode &= ~GLOBAL_RDO;
			serialWriteString( "\r\nRadio OFF\r\n" );
		}
		else if( nextCommand == CMD_UPDATE_TIME )
		{
			// Update time in display
			nextCommand = CMD_NOP;
			
			if( currentState->recurringMethod )
				currentState->recurringMethod();
		}
		else if( nextCommand == CMD_PROGRAM )
		{
			nextCommand = CMD_NOP;
			// Format: pHHMM|T???,IIII,SSSSS
			//		   01234 123456789012345
			//		   0			  1
			
			// triggered or timed?
			if( usartBuffer[1] == 'T' )
			{
				// Triggered (nevermind chars 2-4)
				global_mode |= GLOBAL_TRIGGERED;
				global_mode &= ~GLOBAL_TIMED;
				fprintf( &mystdout, "\r\nManual triggering\r\n" );
			}
			else
			{
				// Timed
				global_mode &= ~GLOBAL_TRIGGERED;
				global_mode |= GLOBAL_TIMED;
				
				// Parse format: pHHMM
				hour = ((usartBuffer[1] - '0') * 10) + (usartBuffer[2] - '0');
				minute = ((usartBuffer[3] - '0') * 10) + (usartBuffer[4] - '0');
				
				setRTCAlarm( hour, minute );
				fprintf( &mystdout, "\r\nAlarm set: %02d:%02d\r\n", hour, minute );
			}
			
			// Set delay (in half seconds)
			seq_delay = ((usartBuffer[6] - '0') * 1000) + ((usartBuffer[7] - '0') * 100) + ((usartBuffer[8] - '0') * 10) + (usartBuffer[9] - '0');
			
			// Set total shots
			seq_total_shots = ((usartBuffer[11] - '0') * 10000) + ((usartBuffer[12] - '0') * 1000) + ((usartBuffer[13] - '0') * 100) + ((usartBuffer[14] - '0') * 10) + (usartBuffer[15] - '0');
			seq_remain_shots = seq_total_shots;
			
			fprintf( &mystdout, "Sequence set: %d shots, %d seconds delay\r\n", seq_total_shots, seq_delay/2 );
		}

		_delay_ms( 10 );
	}

    return 0;
}
