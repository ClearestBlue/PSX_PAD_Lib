/*
 * psx_pad.c
 *
 *  Created on: 7 wrz 2019
 *      Author: Mariusz
 */

#include <inttypes.h>
#include <string.h>
#include "PSX_PAD/psx_pad.h"

const uint8_t PSX_PAD_CMD_POLL[]         = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t PSX_PAD_CMD_ENTER_CFG[]    = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t PSX_PAD_CMD_EXIT_CFG[]     = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
const uint8_t PSX_PAD_CMD_ENABLE_MOTOR[] = {0x01, 0x4D, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t PSX_PAD_CMD_ALL_PRESSURE[] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
const uint8_t PSX_PAD_CMD_AD_MODE[]      = {0x01, 0x44, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t bMotor1Enable;
static uint8_t bMotor2Enable;
static uint8_t bMotor1Level;
static uint8_t bMotor2Level;
static uint8_t lbEnableMotor[9];
static uint8_t lbResponse[21];
static uint8_t lbPoolCmd[21];
static uint8_t lbADMode[9];

uint8_t psxPadData[6];

static void (*psx_pad_delay_us)( uint16_t us );
static uint8_t (*psx_pad_spi_transfer)( uint8_t byte );
static void (*psx_pad_att_set)( void );
static void (*psx_pad_att_clr)( void );

void psx_pad_delay_us_callback( void (*delay_us)( uint16_t us ) ){
	psx_pad_delay_us = delay_us;
}

void psx_pad_spi_transfer_callback( uint8_t (*spi_transfer)( uint8_t byte ) ){
	psx_pad_spi_transfer = spi_transfer;
}

void psx_pad_att_set_callback( void (*att_set)( void ) ){
	psx_pad_att_set = att_set;
}

void psx_pad_att_clr_callback( void (*att_clr)( void ) ){
	psx_pad_att_clr = att_clr;
}

void psx_pad_command( const uint8_t i_lbSendCmd[], const uint8_t i_bSendCmdLen ){
	uint8_t bCmd;

	psx_pad_att_clr();
	psx_pad_delay_us( 10 );

	for( uint8_t i=0; i<21; i++ ){
		bCmd = (i < i_bSendCmdLen) ? i_lbSendCmd[i] : 0x00;
		lbResponse[i] = psx_pad_spi_transfer( bCmd );
		psx_pad_delay_us( 10 );
	}

	psx_pad_att_set();
}

PSXPad_mode_t psx_pad_pool( void ){
	psx_pad_command( lbPoolCmd, sizeof( PSX_PAD_CMD_POLL ) );

	uint8_t * data = &lbResponse[3];
	memcpy( psxPadData, data, sizeof( psxPadData ) );

	return lbResponse[1];
}

void psx_pad_setEnableMotor( const uint8_t i_bMotor1Enable, const uint8_t i_bMotor2Enable ){

	bMotor1Enable = i_bMotor1Enable;
	bMotor2Enable = i_bMotor1Enable;

	lbEnableMotor[3] = (bMotor1Enable == 0) ? 0x00 : 0xFF;
	lbEnableMotor[4] = (bMotor2Enable == 0) ? 0x01 : 0xFF;

	psx_pad_command( PSX_PAD_CMD_ENTER_CFG, sizeof( PSX_PAD_CMD_ENTER_CFG) );
	psx_pad_command( lbEnableMotor, sizeof( PSX_PAD_CMD_ENABLE_MOTOR) );
	psx_pad_command( PSX_PAD_CMD_EXIT_CFG, sizeof( PSX_PAD_CMD_EXIT_CFG ) );
}

void psx_pad_setMotorLevel( const uint8_t i_bMotor1Level, const uint8_t i_bMotor2Level, const uint8_t i_bPool ){
	bMotor1Level = i_bMotor1Level;
	bMotor2Level = i_bMotor2Level;

	lbPoolCmd[3] = bMotor1Level ? 0xFF : 0x00;
	lbPoolCmd[4] = bMotor2Level;

	if( i_bPool )
		psx_pad_pool();
}

void psx_pad_setADMode( const PSXPad_mode_t mode, const uint8_t lock ){

	switch( mode ){
	case PSX_PAD_MODE_DIGITAL:
		lbADMode[3] = 0x00;
		break;
	case PSX_PAD_MODE_ANALOG:
		lbADMode[3] = 0x01;
		break;
	}
	lbADMode[4] = lock ? 0x03 : 0x00;

	psx_pad_command( PSX_PAD_CMD_ENTER_CFG, sizeof( PSX_PAD_CMD_ENTER_CFG ) );
	psx_pad_command( lbADMode, sizeof( PSX_PAD_CMD_AD_MODE ) );
	psx_pad_command( PSX_PAD_CMD_ALL_PRESSURE, sizeof( PSX_PAD_CMD_ALL_PRESSURE ) );
	psx_pad_command( PSX_PAD_CMD_EXIT_CFG, sizeof( PSX_PAD_CMD_EXIT_CFG ) );
}

void psx_pad_init( void ){
	psx_pad_att_set();

	uint8_t i;
	for( i=0; i<sizeof( PSX_PAD_CMD_AD_MODE ); i++ )
		lbADMode[i] = PSX_PAD_CMD_AD_MODE[i];
	for( i=0; i<sizeof( PSX_PAD_CMD_POLL ); i++ )
		lbPoolCmd[i] = PSX_PAD_CMD_POLL[i];
	for( i=0; i<sizeof( PSX_PAD_CMD_ENABLE_MOTOR ); i++ )
		lbEnableMotor[i] = PSX_PAD_CMD_ENABLE_MOTOR[i];

}
