/*
 * timers.c
 *
 *  Created on: 7 wrz 2019
 *      Author: Mariusz
 */
#include "stm32f7xx_hal.h"
#include "TIMERS/timers.h"


static volatile Timer_t * timers[TIMER_IN_USE];
static volatile uint8_t timerCounter;

void timerStart( Timer_t * timer ){
	timer->start_flag = 1;
}

void timerStop( Timer_t * timer ){
	timer->start_flag = 0;
}

void timerInterval( Timer_t * timer, uint16_t msec ){
	timer->interval = msec;
	timer->counter = timer->interval;
}

void timerSet( Timer_t * timer, uint16_t msec, void(*event)( void ), uint8_t mode ){

	timer->interval = msec;
	timer->counter = timer->interval;
	timer->mode = mode;
	timer->start_flag = 0;
	timer->timeout_flag = 0;
	timer->event = event;

	timers[timerCounter] = timer;
	timerCounter++;
}

void TIMERS_EVENTS( void ){

	for( uint8_t i=0; i<timerCounter; i++ ){
		if( timers[i] ){
			if( timers[i]->timeout_flag ){
				if( timers[i]->event ) timers[i]->event();
				timers[i]->timeout_flag = 0;
			}
		}
	}

}


void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim ){

	for( uint8_t i=0; i<timerCounter; i++ ){
		if( timers[i]->start_flag ){
			if( timers[i]->counter ) timers[i]->counter--;
			else{
				timers[i]->counter = timers[i]->interval;
				timers[i]->timeout_flag = 1;
				if( timers[i]->mode == 1 ) timers[i]->start_flag = 0;
			}
		}
	}
}

