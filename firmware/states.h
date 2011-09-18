//
//  states.h
//  IntervalTimer
//
//  Created by Jens Willy Johannsen on 09-09-11.
//  Copyright 2011 Greener Pastures. All rights reserved.
//

#ifndef STATES
#define STATES

#define nil 0

typedef struct stateStruct
{
	void *keyUpState;
	void *keyDownState;
	void *keyLeftState;
	void *keyRightState;
	void *keyCenterState;
	void (*actionMethod)();
	void (*entryMethod)();
	void (*exitMethod)();
	void (*recurringMethod)();
} state;

extern state *currentState;
void setState( state* newState );

// Idle
extern state state_idle;
void setup_idle();
void recurring_idle();

// Menu1_1
extern state state_menu1_1;
void setup_menu1_1();
void exit_menu1_1();

// Menu1_2
extern state state_menu1_2;
void setup_menu1_2();
void exit_menu1_2();

	// Menu1_2_1
	extern state state_menu1_2_1;
	void setup_menu1_2_1();
	void exit_menu1_2_1();

		// Menu1_2_1_1
		extern state state_menu1_2_1_1;
		void setup_menu1_2_1_1();
		void exit_menu1_2_1_1();
		void action_menu1_2_1_1();

		// Menu1_2_1_2
		extern state state_menu1_2_1_2;
		void setup_menu1_2_1_2();
		void exit_menu1_2_1_2();
		void action_menu1_2_1_2();

		// Menu1_2_1_3
		extern state state_menu1_2_1_3;
		void setup_menu1_2_1_3();
		void exit_menu1_2_1_3();

	// Menu1_2_2
	extern state state_menu1_2_2;
	void setup_menu1_2_2();
	void exit_menu1_2_2();

	// Menu1_2_3
	extern state state_menu1_2_3;
	void setup_menu1_2_3();
	void exit_menu1_2_3();

// Menu1_3
extern state state_menu1_3;
void setup_menu1_3();
void exit_menu1_3();


#endif