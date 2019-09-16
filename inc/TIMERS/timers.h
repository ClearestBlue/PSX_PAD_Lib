/*
 * timers.h
 *
 *  Created on: 7 wrz 2019
 *      Author: Mariusz
 */

#ifndef TIMERS_TIMERS_H_
#define TIMERS_TIMERS_H_

#define TIMER_IN_USE 1

enum timerMode{
	TIMER_MODE_CONTINUOUS,
	TIMER_MODE_SINGLESHOT
};

typedef struct{
	uint8_t start_flag;
	uint8_t timeout_flag;
	uint8_t mode;
	uint16_t interval;
	uint16_t counter;
	void (*event)( void );
}Timer_t;

void timerStart( Timer_t * timer );
void timerStop( Timer_t * timer );
void timerInterval( Timer_t * timer, uint16_t msec );
void timerSet( Timer_t * timer, uint16_t msec, void(*event)( void ), uint8_t mode );
void TIMERS_EVENTS( void );

#endif /* TIMERS_TIMERS_H_ */
