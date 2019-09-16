#include "stm32f7xx_hal.h"
#include "SPI_soft/spi_soft.h"

static void (*spiSoft_delay_us)( uint16_t us );

void softSpi_delay_us_callback( void (*delay_us)( uint16_t us ) ){
	spiSoft_delay_us = delay_us;
}

void spiSoft_init( void ){

// 	SOFT_MOSI_DDR |= SOFT_MOSI;
// 	SOFT_SCK_DDR |= SOFT_SCK;
// 	SOFT_MISO_DDR &= ~SOFT_MISO;

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
//	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, SCK_Pin|MOSI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : CS_Pin SCK_Pin MOSI_Pin */
	GPIO_InitStruct.Pin = SCK_Pin|MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : MISO_Pin */
	GPIO_InitStruct.Pin = MISO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MISO_GPIO_Port, &GPIO_InitStruct);


// 	SOFT_MOSI_PORT |= SOFT_MOSI;
	HAL_GPIO_WritePin( MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET );

#if SOFT_SPI_MODE == 0
//	SOFT_SCK_PORT &= ~SOFT_SCK;
	HAL_GPIO_WritePin( SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET );
#elif SOFT_SPI_MODE == 1
//	SOFT_SCK_PORT &= ~SOFT_SCK;
	HAL_GPIO_WritePin( SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET );
#elif SOFT_SPI_MODE == 2
//	SOFT_SCK_PORT |= SOFT_SCK;
	HAL_GPIO_WritePin( SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET );
#elif SOFT_SPI_MODE == 3
//	SOFT_SCK_PORT |= SOFT_SCK;
	HAL_GPIO_WritePin( SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET );
#endif

}

uint8_t spiSoft_transfer( uint8_t byte ){

	uint8_t data = 0x00;
	for( uint8_t b=0; b<8; b++ ){
#if SOFT_SPI_DATA_ORDER == 0
		if( byte & (0x80 >> b) )
#else
		if( byte & (0x01 << b) )
#endif
//			SOFT_MOSI_PORT |= SOFT_MOSI;
			HAL_GPIO_WritePin( MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET );
		else
//			SOFT_MOSI_PORT &= ~SOFT_MOSI;
			HAL_GPIO_WritePin( MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET );

#if SOFT_SPI_MODE == 0 || SOFT_SPI_MODE == 2
//		if( SOFT_MISO_PIN & SOFT_MISO )
		if( HAL_GPIO_ReadPin( MISO_GPIO_Port, MISO_Pin ) )
			data |= (0x01 << b);
#endif

#if SOFT_SPI_MODE == 0 || SOFT_SPI_MODE == 1
//		SOFT_SCK_PORT |= SOFT_SCK;
		HAL_GPIO_WritePin( SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET );
#else
//		SOFT_SCK_PORT &= ~SOFT_SCK;
		HAL_GPIO_WritePin( SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET );
#endif
//		_delay_us( SOFT_SPI_CLK_TIME_US );
		spiSoft_delay_us( SOFT_SPI_CLK_TIME_US );

#if SOFT_SPI_MODE == 1 || SOFT_SPI_MODE == 3
//		if( SOFT_MISO_PIN & SOFT_MISO )
		if( HAL_GPIO_ReadPin( MISO_GPIO_Port, MISO_Pin ) )
			data |= (0x01 << b);
#endif

#if SOFT_SPI_MODE == 0 || SOFT_SPI_MODE == 1
//		SOFT_SCK_PORT &= ~SOFT_SCK;
		HAL_GPIO_WritePin( SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET );
#else
//		SOFT_SCK_PORT |= SOFT_SCK;
		HAL_GPIO_WritePin( SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET );
#endif
//		_delay_us( SOFT_SPI_CLK_TIME_US );
		spiSoft_delay_us( SOFT_SPI_CLK_TIME_US );
	}

//	SOFT_MOSI_PORT |= SOFT_MOSI;
	HAL_GPIO_WritePin( MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET );

//	printf( "%d\r\n", data );
	return data;

}

uint8_t spiSoft_inverse( uint8_t byte ){

	uint8_t mask = 1, result = 0;

	while( mask ){
		if( byte & 0x80 )
			result |= mask;
		mask <<= 1;
		byte <<= 1;
	}

	return result;
}
