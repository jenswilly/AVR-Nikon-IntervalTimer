//
//  main.h
//  IntervalTimer
//
//  Created by Jens Willy Johannsen on 18-09-11.
//  Copyright 2011 Greener Pastures. All rights reserved.
//

#ifndef MAIN
#define MAIN

#include <stdio.h>

// Prototypes
void enable_spi();
void sleep();
void setup_ports();
void setup_interrupts();
void setup_timers();
void enable_serial();
void serialWriteByte( char data );
void serialWriteString( const char* string );
void trigger();

extern unsigned char global_mode;
#define GLOBAL_RDO			1
#define GLOBAL_TRIGGERED	2
#define GLOBAL_TIMED		4

extern FILE mystdout;
extern FILE lcd;

#endif