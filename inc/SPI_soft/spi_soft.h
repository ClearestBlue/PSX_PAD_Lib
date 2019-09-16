/*
 * spi_soft.h
 *
 *  Created on: 07.09.2019
 *      Author: Mariusz
 */

#ifndef SPI_SOFT_SPI_SOFT_H_
#define SPI_SOFT_SPI_SOFT_H_

#define SCK_Pin 	GPIO_PIN_5
#define MISO_Pin 	GPIO_PIN_6
#define MOSI_Pin 	GPIO_PIN_7

#define SCK_GPIO_Port 	GPIOA
#define MOSI_GPIO_Port 	GPIOA
#define MISO_GPIO_Port 	GPIOA

#define SOFT_SPI_MODE	3
#define SOFT_SPI_DATA_ORDER 1
#define SOFT_SPI_CLK_TIME_US	(((1.0 / 250000UL) * 1000000.0) / 2.0)

/*
 * SPI_MODE 0..3
 * 0: CPOL = 0, CPHA = 0
 * 1: CPOL = 0, CPHA = 1
 * 2: CPOL = 1, CPHA = 0
 * 3: CPOL = 1, CPHA = 1
 */
/*
 * SPI_DATA_ORDER 0..1
 * 0: MSB transmitted first
 * 1: LSB transmitted first
 */

void softSpi_delay_us_callback( void (*delay_us)( uint16_t us ) );
void spiSoft_init( void );
uint8_t spiSoft_transfer( uint8_t byte );
uint8_t spiSoft_inverse( uint8_t byte );

#endif /* SPI_SOFT_SPI_SOFT_H_ */
