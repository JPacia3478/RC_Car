/*
 * Joseph Pacia - jpaci001@ucr.edu
 *
 * RC Bluetooth Car
 *
 * I acknowledge all content contained herein, excluding template or example
 * code, is my own original work.
 */ 

#include <avr/io.h>
#include "usart_ATmega1284.h"
#include <avr/interrupt.h>
#include "timer.h"

// Scheduler code--------------------------------------------------------------

//--------Find GCD function --------------------------------------------------
unsigned long int findGCD(unsigned long int a, unsigned long int b)
{
	unsigned long int c;
	while(1){
		c = a%b;
		if(c==0){return b;}
		a = b;
		b = c;
	}
	return 0;
}
//--------End find GCD function ----------------------------------------------

//--------Task scheduler data structure---------------------------------------
// Struct for Tasks represent a running process in our simple real-time operating system.
typedef struct _task {
	/*Tasks should have members that include: state, period,
		a measurement of elapsed time, and a function pointer.*/
	signed char state; //Task's current state
	unsigned long int period; //Task period
	unsigned long int elapsedTime; //Time elapsed since last task tick
	int (*TickFct)(int); //Task tick function
} task;

//--------End Task scheduler data structure-----------------------------------

unsigned char count = 0x00;
unsigned char leftSens;
unsigned char frontSens;
unsigned char rightSens;
unsigned char left;
unsigned char right;
unsigned char UT_count = 0x00;
unsigned char mode;
unsigned char rCount = 0x00;

//--------- State machine for automatic movement-----------------------

enum moveStates{init, wait, straight, TL, TR, RL, RR, reverse, UT};

int moveTick(int state)
{
	leftSens = ~PINA & 0x08;
	frontSens = ~PINA & 0x10;
	rightSens = ~PINA & 0x20;

	switch(state)
	{
		case init:
			state = wait;
		break;
		case wait:
			if (mode == 0x01)
				state = straight;
			else
				state = wait;
		break;
		case straight:
			if (mode == 0x00)
				state = wait;
			if (rightSens)
			{
				if (frontSens)
					state = RL;
				else
					state = TL;
			}
			else if (leftSens)
			{
				if (frontSens)
					state = RR;
				else
					state = TR;
			}
			else if (frontSens && !leftSens && !rightSens)
				state = reverse;
			else if (leftSens && frontSens && rightSens)
				state = UT;
			else
				state = straight;
		break;
		case TL:
			state = straight;
		break;
		case RL:
			if (left >= 0x07)
			{
				left = 0x00;
				state = RR;
			}
			else
			{
				left = left + 0x01;
				state = straight;
			}
		break;
		case UT:
			if (UT_count >= 0x08)
			{
				UT_count = 0x00;
				state = straight;
			}
			else
			{
				UT_count = UT_count + 0x01;
				state = UT;
			}
		break;
		case RR:
			if (right >= 0x07)
			{
				right = 0x00;
				state = RL;
			}
			else
			{
				right = right + 0x01;
				state = straight;
			}
		break;
		case TR:
			state = straight;
		break;
		case reverse:
			if (rCount > 0x08)
			{
				rCount = 0x00;
				state = RR;
			}
			else
			{
				rCount = rCount + 0x01;
				state = reverse;
			}
		break;
		default:
			state = init;
		break;
	}
	switch(state)
	{
		case init:
		break;
		case wait:
			PORTB = 0x00;
			count = 0x00;
		break;
		case straight:
			PORTB = 0x05;
		break;
		case TL:
			PORTB = 0x01;
		break;
		case TR:
			PORTB = 0x04;
		break;
		case RL:
			PORTB = 0x09;
		break;
		case RR:
			PORTB = 0x06;
		break;
		case reverse:
			PORTB = 0x0A;
		break;
		case UT:
			PORTB = 0x06;
		break;
		default:
		break;
	}
	return state;
}

//----------------------------------------------------------------------------------------------

//--------------------- State machine for switching between manual and auto---------------------

unsigned char mode_select;
enum modeStates{modeInit, manual, automatic, release};

int modeTick(int state)
{
	mode_select = ~PINA & 0x01;

	switch (state) // transitions
	{
		case modeInit:
			state = manual;
		break;
		case manual:
			if (mode_select)
				state = release;
			else
				state = manual;
		break;
		case automatic:
			if (mode_select)
				state = release;
			else
				state = automatic;
		break;
		case release:
			if (!mode_select)
			{
				if (mode == 0x00)
					state = automatic;
				else
					state = manual;
			}
			else
				state = release;
		break;
		default:
			state = modeInit;
		break;
	}
	switch(state) // state actions
	{
		case modeInit:
			mode = 0x00;
		break;
		case manual:
			mode = 0x00;
		break;
		case automatic:
			mode = 0x01;
		break;
		case release:
		break;
		default:
		break;
	}
	return state;
}

//-----------------------------------------------------------------------------------

//------------- State machine for receiving controller commands ---------------------

enum followStates{followInit, standby, follow};

int followTick(int state)
{
	switch (state) // transitions
	{
		case followInit:
			state = standby;
		break;
		case standby:
			if (mode == 0x00)
				state = follow;
			else
				state = standby;
		break;
		case follow:
			if (mode == 0x01)
				state = standby;
			else
				state = follow;
		break;
		default:
			state = followInit;
		break;
	}
	switch (state) // state actions
	{
		case followInit:
			USART_Flush(0);
		break;
		case standby:
		break;
		case follow:
			PORTB = USART_Receive(0);
		break;
		default:
		break;
	}
	return state;
}

//----------------------------------------------------------------------------------------------

//-------------------------- Main Function -----------------------------------------------------

int main(void)
{
	// Set Registers
	DDRA = 0x00; PORTA = 0xFF; // Buttons and sensors
	DDRB = 0xFF; PORTB = 0x00; // Motors

	// Task Periods
	unsigned long int moveTick_calc = 125;
	unsigned long int modeTick_calc = 250;
	unsigned long int followTick_calc = 125;

	// Calculating GCD
	unsigned long int tmpGCD = 1;
	tmpGCD = findGCD(moveTick_calc, modeTick_calc);
	tmpGCD = findGCD(tmpGCD, followTick_calc);

	// GCD for all tasks or smallest time unit for tasks
	unsigned long int tasksPeriodGCD = tmpGCD;

	// Recalculate GCD periods for scheduler
	unsigned long int moveTick_period = moveTick_calc/tasksPeriodGCD;
	unsigned long int modeTick_period = modeTick_calc/tasksPeriodGCD;
	unsigned long int followTick_period = followTick_calc/tasksPeriodGCD;

	// Declare an array of tasks
	static task moveTask, modeTask, followTask;
	task *tasks[] = {&moveTask, &modeTask, &followTask};
	const unsigned short numTasks = sizeof(tasks)/sizeof(task*);

	// Movement Task
	moveTask.state = -1;
	moveTask.period = moveTick_period;
	moveTask.elapsedTime = moveTick_period;
	moveTask.TickFct = &moveTick;

	// Mode Task
	modeTask.state = -1;
	modeTask.period = modeTick_period;
	modeTask.elapsedTime = modeTick_period;
	modeTask.TickFct = &modeTick;

	// Follow Task
	followTask.state = -1;
	followTask.period = followTick_period;
	followTask.elapsedTime = followTick_period;
	followTask.TickFct = &followTick;

	// Set and turn on timer
	TimerSet(tasksPeriodGCD);
	TimerOn();

	// Turn on USART0
	initUSART(0);

	// Scheduler iterator
	unsigned short i;

	while(1) {
		// Scheduler code
		for ( i = 0; i < numTasks; i++ ) {
			// Task is ready to tick
			if ( tasks[i]->elapsedTime == tasks[i]->period ) {
				// Setting next state for task
				tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
				// Reset the elapsed time for next tick.
				tasks[i]->elapsedTime = 0;
			}
			tasks[i]->elapsedTime += 1;
		}
		while(!TimerFlag);
		TimerFlag = 0;
	}

	return 0;
}

