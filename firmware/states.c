//
//  states.c
//  IntervalTimer
//
//  Created by Jens Willy Johannsen on 09-09-11.
//  Copyright 2011 Greener Pastures. All rights reserved.
//

#include "states.h"
#include "5110LCD.h"
#include "PCF8563RTC.h"
#include "nRF24L01p.h"
#include "main.h"

state *currentState;

void setState( state* newState )
{
	// Exit method on current state
	if( currentState->exitMethod )
		currentState->exitMethod();
	
	// Set new state
	currentState = newState;
	
	// Entry method on new state
	if( currentState->entryMethod )
		currentState->entryMethod();
}

/* MENU 1.1 */
state state_menu1_1 = { &state_menu1_3, &state_menu1_2, nil, nil, nil, nil, setup_menu1_1, exit_menu1_1, nil };

void setup_menu1_1()
{
	LCD_clear();
	LCD_writeString_F( ") Program   " );
	LCD_writeString_F( "( Setup     " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "( Back      " );
}

void exit_menu1_1()
{
	// Deselect current item
	LCD_gotoXY( 0,0 );
	LCD_writeChar( '(' );
}

/* MENU 1.2 */
state state_menu1_2 = { &state_menu1_1, &state_menu1_3, nil, nil, &state_menu1_2_1, nil, setup_menu1_2, exit_menu1_2, nil };

void setup_menu1_2()
{
	// Select current item
	LCD_gotoXY( 0, 1 );
	LCD_writeChar( ')' );
}

void exit_menu1_2()
{
	// Deselect current item
	LCD_gotoXY( 0, 1 );
	LCD_writeChar( '(' );
}

/* MENU 1.3 */
state state_menu1_3 = { &state_menu1_2, &state_menu1_1, nil, nil, &state_idle, nil, setup_menu1_3, exit_menu1_3, nil };

void setup_menu1_3()
{
	// Select current item
	LCD_gotoXY( 0, 5 );
	LCD_writeChar( ')' );
}

void exit_menu1_3()
{
	// Deselect current item
	LCD_gotoXY( 0, 5 );
	LCD_writeChar( '(' );	
}


/* MENU 1.2.1 */
state state_menu1_2_1 = { &state_menu1_2_3, &state_menu1_2_2, nil, nil, &state_menu1_2_1_1, nil, setup_menu1_2_1, exit_menu1_2_1, nil };

void setup_menu1_2_1()
{
	LCD_clear();
	LCD_writeString_F( ") Radio     " );
	LCD_writeString_F( "( Date/time " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "( Back      " );
}

void exit_menu1_2_1()
{
	// Deselect current item
	LCD_gotoXY( 0, 0 );
	LCD_writeChar( '(' );	
}

/* MENU 1.2.2 */
state state_menu1_2_2 = { &state_menu1_2_1, &state_menu1_2_3, nil, nil, nil, nil, setup_menu1_2_2, exit_menu1_2_2, nil };

void setup_menu1_2_2()
{
	// Select current item
	LCD_gotoXY( 0, 1 );
	LCD_writeChar( ')' );
}

void exit_menu1_2_2()
{
	// Deselect current item
	LCD_gotoXY( 0, 1 );
	LCD_writeChar( '(' );	
}

/* MENU 1.2.3 */
state state_menu1_2_3 = { &state_menu1_2_2, &state_menu1_2_1, nil, nil, &state_menu1_1, nil, setup_menu1_2_3, exit_menu1_2_3, nil };

void setup_menu1_2_3()
{
	// Select current item
	LCD_gotoXY( 0, 5 );
	LCD_writeChar( ')' );
}

void exit_menu1_2_3()
{
	// Deselect current item
	LCD_gotoXY( 0, 5 );
	LCD_writeChar( '(' );	
}

/* 1.2.1.1 */
state state_menu1_2_1_1 = { &state_menu1_2_1_3, &state_menu1_2_1_2, nil, nil, nil, action_menu1_2_1_1, setup_menu1_2_1_1, exit_menu1_2_1_1, nil };

void setup_menu1_2_1_1()
{
	LCD_clear();
	LCD_writeString_F( ") On        " );
	LCD_writeString_F( "( Off       " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "( Back      " );
}

void exit_menu1_2_1_1()
{
	// Deselect current item
	LCD_gotoXY( 0, 0 );
	LCD_writeChar( '(' );
}

void action_menu1_2_1_1()
{
	global_mode |= GLOBAL_RDO;
	nRF24L01p_startRX();
}

/* 1.2.1.2 */
state state_menu1_2_1_2 = { &state_menu1_2_1_1, &state_menu1_2_1_3, nil, nil, nil, action_menu1_2_1_2, setup_menu1_2_1_2, exit_menu1_2_1_2, nil };

void setup_menu1_2_1_2()
{
	// Deselect current item
	LCD_gotoXY( 0, 1 );
	LCD_writeChar( ')' );
}

void exit_menu1_2_1_2()
{
	// Deselect current item
	LCD_gotoXY( 0, 1 );
	LCD_writeChar( '(' );
}

void action_menu1_2_1_2()
{
	global_mode &= ~GLOBAL_RDO;
	nRF24L01p_powerDown();
}

/* MENU 1.2.1.3 */
state state_menu1_2_1_3 = { &state_menu1_2_1_2, &state_menu1_2_1_1, nil, nil, &state_menu1_2_1, nil, setup_menu1_2_1_3, exit_menu1_2_1_3, nil };

void setup_menu1_2_1_3()
{
	// Select current item
	LCD_gotoXY( 0, 5 );
	LCD_writeChar( ')' );
}

void exit_menu1_2_1_3()
{
	// Deselect current item
	LCD_gotoXY( 0, 5 );
	LCD_writeChar( '(' );	
}


	
/* IDLE */
state state_idle = { &state_menu1_1, &state_menu1_1, &state_menu1_1, &state_menu1_1, nil, trigger, setup_idle, nil, recurring_idle };

void setup_idle()
{
	LCD_clear();
	LCD_writeString_F( "          " );
	if( global_mode & GLOBAL_TIMED )
		LCD_writeChar( '+' );
	else if( global_mode & GLOBAL_TRIGGERED )
		LCD_writeChar( '@' );
	else
		LCD_writeChar( ' ' );
	
	if( global_mode & GLOBAL_RDO )
		LCD_writeChar( '$' );
	else
		LCD_writeChar( '%' );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "            " );
	LCD_writeString_F( "--:--:--    " );
}

void recurring_idle()
{
	unsigned char year, month, day, weekDay, hour, minute, second;
	
	LCD_gotoXY( 0,0 );
	LCD_writeString_F( "          " );
	if( global_mode & GLOBAL_TIMED )
		LCD_writeChar( '+' );
	else if( global_mode & GLOBAL_TRIGGERED )
		LCD_writeChar( '@' );
	else
		LCD_writeChar( ' ' );
	
	if( global_mode & GLOBAL_RDO )
		LCD_writeChar( '$' );
	else
		LCD_writeChar( '%' );

	// Print time
	readRTCClock( &year, &month, &day, &weekDay, &hour, &minute, &second );
	LCD_gotoXY( 0, 5 );
	fprintf( &lcd, "%02d:%02d:%02d", hour, minute, second );
}