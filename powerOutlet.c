/*
 * powerOutlet.c
 *
 *  Created on: 05.11.2016
 *      Author: flo
 */


#include "powerOutlet.h"
#include <string.h>

#include "rfm69ModemSettings.h"
#include "rfm69.h"

static const uint8_t logicZero = 0x88;
static const uint8_t logicOne = 0xEE;
static const uint8_t logicFloat = 0x8E;

static const RFM69MODEMPARMS_t powerOutletConfig =
{ CONFIG_OOK, 0x3200, 0x0029, 0xf4, 0xf4, CONFIG_NOWHITE}; // GFSK_Rb2_5Fd5_0

int8_t powerOutlet_hx2262(const char* cmd)
{
	RFM69PKT_t pkt;
	uint8_t temp;

	/*
	 * Generate Packet
	 */
	if(strlen(cmd) < 12) return -1;

	/*
	 * Iterate over 12 characters - Cmd length of hx2262
	 */
	for(int8_t i = 0; i < 12; i++)
	{
		switch(cmd[i])
		{
		case '0':
			temp = logicZero;
			break;
		case '1':
			temp = logicOne;
			break;
		case 'F':
			temp = logicFloat;
			break;
		case 'f':
			temp = logicFloat;
			break;
		default:
			return -2;
			break;
		}

		pkt.data[i] = temp;

	}

	/*
	 * Add Sync
	 */
	pkt.data[12] = 0x80;
	memset(pkt.data+13, 0, 3);

	//No specific key
	pkt.encKey = NULL;

	//No encryption
	pkt.enc = RF69PKTENC_OFF;

	/*
	 * Expand packet to get four times one hx2262 packet
	 */
	for(int8_t i = 0; i < 3; i++) memcpy(pkt.data+(16*(i+1)),pkt.data,16);
	pkt.dst = 0xAA;
	pkt.size = 64;


	rfm69_sendPacketModSet(&pkt, powerOutletConfig);

	return 0;
}


int8_t powerOutlet_elroab440(const char* devCode, bool state)
{
	char code[12];

	if(strlen(devCode) < 10) return -1;

	for(int8_t i = 0; i < 10; i++)
	{
		switch(devCode[i])
		{
		case '0':
			code[i] = 'F';
			break;

		case '1':
			code[i] = '0';
			break;

		default:
			return -2;
			break;
		}
	}

	if(state)
	{
		code[10] = '0';
		code[11] = 'F';
	}

	else
	{
		code[10] = 'F';
		code[11] = '0';
	}

	return powerOutlet_hx2262(code);
}
