/*
 * psx_pad.h
 *
 *  Created on: 7 wrz 2019
 *      Author: Mariusz
 */

#ifndef PSX_PAD_PSX_PAD_H_
#define PSX_PAD_PSX_PAD_H_

/*
 * PSX_PAD v1.0
 */

/*
 * 1: Data (MISO) ( 2,2k pull up )
 * 2: Command (MISO)
 * 3: 7V Rumble motor
 * 4: GND
 * 5: 3,3V
 * 6: ATT
 * 7: CLK (SCK)
 * 8: NC
 * 9: ACK
 *
 * SPI
 * SPI_MODE: 3
 * AVR: (1<<CPHA) | (1<<CPOL)
 * STM32: SPI_POLARITY_HIGH, SPI_PHASE_2EDGE
 * SPI_BIT_ORDER: LSB first
 * SPI_CLK_SPEED: <250kHz
 */

typedef enum {
	PSX_PAD_MODE_DIGITAL = 0x41,
	PSX_PAD_MODE_ANALOG = 0x73
}PSXPad_mode_t;


enum psxPadDataIndex{
	keys1,
	keys2,
	joyLeftRightR,
	joyUpDownR,
	joyLeftRightL,
	joyUpDownL
};

enum keys1Decode{
	SELECT 	= 0b11111110,
	L3 		= 0b11111101,
	R3 		= 0b11111011,
	START 	= 0b11110111,
	UP 		= 0b11101111,
	RIGHT 	= 0b11011111,
	DOWN 	= 0b10111111,
	LEFT 	= 0b01111111
};

enum keys2Decode{
	L2 			= 0b11111110,
	R2 			= 0b11111101,
	L1 			= 0b11111011,
	R1 			= 0b11110111,
	TRIANGLE	= 0b11101111,
	CIRCLE 		= 0b11011111,
	CROSS 		= 0b10111111,
	SQUARE 		= 0b01111111
};

extern uint8_t psxPadData[6];

void psx_pad_delay_us_callback( void (*delay_us)( uint16_t us ) );
void psx_pad_spi_transfer_callback( uint8_t (*spi_transfer)( uint8_t byte ) );
void psx_pad_att_set_callback( void (*att_set)( void ) );
void psx_pad_att_clr_callback( void (*att_clr)( void ) );

void psx_pad_setEnableMotor( const uint8_t i_bMotor1Enable, const uint8_t i_bMotor2Enable );
void psx_pad_setMotorLevel( const uint8_t i_bMotor1Level, const uint8_t i_bMotor2Level, const uint8_t i_bPool );
void psx_pad_setADMode( const PSXPad_mode_t mode, const uint8_t lock );
void psx_pad_getKeyState( void );
PSXPad_mode_t psx_pad_pool( void );
void psx_pad_init( void );

#endif /* PSX_PAD_PSX_PAD_H_ */
