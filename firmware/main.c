/*	NEXT STEPS:
 *
 *	1)	UART comm -> FTDI
 *		keep writing test string, then delay
 *	2)	Sleep
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


// Global vars
volatile unsigned char SLEEPING = 0;


// Interrupt handler for PCINT1
// This is used for PCINT12: PWREN# from USB connect/disconnect
ISR(PCINT1_vect)
{
	// Re-enable SPI if we were sleeping
	if( SLEEPING )
	{
		enable_spi();
		SLEEPING = 0;
	}
	
	// Debounce
	_delay_ms( 20 );
	
	// Is PWREN# low?
	if( !(PINC & (1<<PC3)) )
	{
		// Yes: USB connected
		
		// Power up 5110 LED
		PORTB |= (1<<PB2);
		
		// Clear LCD and draw image
		LCD_clear();
		LCD_drawImage( nuke );
	}
	else
	{
		// No: USB disconnected
		
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
//	DDRD |= _BV( PD2 ) | _BV( PD3 ) | _BV( PD7 );	// SCE, RESET and D/C -> output
	DDRB |= _BV( PB2 ) | _BV( PB3 ) | _BV( PB5 );	// SS, MOSI and SCLK -> output
	DDRB &= ~(1<<PB4);	// MISO input

//	PORTD |= _BV( PD2 );	// SS high
//	PORTB &= ~(1<<PB5);	// CLK low
//	PORTB &= ~(1<<PB3);	// MOSI low
//	DDRB &= ~(1<<PB4);	// MISO input
//	PORTB |= (1<<PB4);	// MISO pull-up
	
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
}

// Call this method to initialize SPI modules.
// Remember to call this method after waking up
void enable_spi()
{
	// Init SPI
	SPCR = 0x52;	// SPI enable, master, sample on leading edge, rising, rate=Fosc/64
	SPSR = 0;		// not 2x data rate
}

// Call this method to go to sleep
void sleep()
{
	SMCR |= (1<< SM1) | (1<< SE);	// sleep-mode=power-down, enable sleep
	PRR = 0xEF;						// turn everything off (actually, we don't need to switch everything off since power-down will turn off lots of stuff by itself)
	
	SLEEPING = 1;
	asm("sleep");					// nighty, night.
}

int main(void)
{
	setup_ports();
	setup_interrupts();
	
	// Init SPI
	enable_spi();
	
	LCD_init();
	LCD_writeString_F( "Ready..." );

	// Enable interrupts
	sei();

	// Main loop
	for( ;; )
	{
		// wait 3 secs, then sleep
		_delay_ms( 3000 );
		sleep();
	}

    return 0;
}
