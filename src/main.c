/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f7xx.h"
#include "init.h"
#include "PSX_PAD/psx_pad.h"
#include "SPI_soft/spi_soft.h"
#include "TIMERS/timers.h"

void timer0_event( void );

int main( void ){

	HAL_Init();
	init();

//	spiSoft_init();

	psx_pad_init();
	psx_pad_setADMode( PSX_PAD_MODE_ANALOG, 0 );

	Timer_t Timer0;
	timerSet( &Timer0, 100, timer0_event, TIMER_MODE_CONTINUOUS );
	timerStart( &Timer0 );

	printf( "PSX_PAD stm test\r\n" );

	while(1){

		TIMERS_EVENTS();

	}
}

void timer0_event( void ){


	if( psx_pad_pool() == PSX_PAD_MODE_ANALOG ){

		printf( "%d, %d, %d, %d, %d, %d\r\n", psxPadData[0],
											  psxPadData[1],
											  psxPadData[2],
											  psxPadData[3],
											  psxPadData[4],
											  psxPadData[5] );

	}

}
