/*
 * rfm69.c
 *
 *  Created on: 28.11.2015
 *      Author: flo
 */


#include "rfm69.h"
#include "rfm69net.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "../usart.h"


#define FREQ 7109345UL

#define NEW_METHOD

//USE POWER OF 2!!!
#define MAXPKTS 4


typedef enum
{
	IDLE,
	RX,
	TX,

}rfm69State_t;

static volatile rfm69State_t state;

//static uint8_t buffer[100];

static uint8_t currentPkt = 0;
static uint8_t nextPkt = 0;
static RFM69PKTMGMT_t packets[MAXPKTS];

uint8_t exchangebyte(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

void select()
{
	PORTB &= ~(1<<PB4);
}

void deselect()
{
	PORTB |= (1<<PB4);
}

void rfm69_init()
{
	uint8_t i;
	/*
	 * Physical stuff here
	 */
	uint8_t freq[3];

	freq[0] = (FREQ>>16) & 0xFF;
	freq[1] = (FREQ>>8) & 0xFF;
	freq[2] = (FREQ) &0xFF;

	//Set Mode
	rfm69_writeReg(REG_OP_MODE, REG_OP_MODE_STDBY);

	//FSK PACKETMODE NOSHAPING
	//rfm69_writeReg(REG_DATA_MODUL, 0);
	rfm69_writeReg(REG_DATA_MODUL, REG_DATA_MODUL_MODULATION_SHAPING_GAUSS_BT1_0);

	rfm69_writeReg(REG_PREAMBLE_LSB, 10);

	/*
	 * Speed specific configuration
	 */

	//Bitrate
	rfm69_setBitrate(0x006b);

	//FDEV
	rfm69_setFdev(0x52*64);

	//RxBw
	rfm69_setRxBw(0b010, 0, 0);

	//Improved AFC
	rfm69_writeReg(REG_AFCCTRL, REG_AFCCTRL_AFC_LOW_BETA_ON);

	//DAGC
	rfm69_writeReg(REG_TEST_DAGC, 0x20);

	rfm69_writeReg(REG_TEST_AFC, 45);



	//Set Power
	rfm69_writeReg(REG_PA_LEVEL, REG_PA_LEVEL_PA0ON + 25);

	//Set Frequency
	rfm69_writeRegBurst(REG_FRF_MSB, freq, 3);

	//CLK OFF
	rfm69_writeReg(REG_DIO_MAPPING2, 0b111);

	//Set RSSI Threshold
	rfm69_writeReg(REG_RSSI_THRESH, 220);


	/*
	 * Init Packet Management
	 */

	rfm69_writeReg(REG_PREAMBLE_LSB, 3);

	//SYNC ON, 2 SYNC BYTES
	rfm69_writeReg(REG_SYNC_CONFIG, REG_SYNC_CONFIG_SYNC_ON + (1<<3));


	//SYNC VALUE
	rfm69_writeReg(REG_SYNC_VALUE1, 0x2D);

	//this could be changed through setPANID
	rfm69_writeReg(REG_SYNC_VALUE2, 0xD4);


	rfm69_writeReg(REG_PACKET_CONFIG1, REG_PACKET_CONFIG1_PACKET_FORMAT_VARLEN | REG_PACKET_CONFIG1_CRC_ON | REG_PACKET_CONFIG1_ADDR_FILTER_NODE_OR_BRDCAST_ADDR);

	rfm69_writeReg(REG_PAYLOAD_LENGTH, 66);

	//just dummy - set by tx routine
	rfm69_writeReg(REG_FIFO_THRESH, 1);

	rfm69_writeReg(REG_PACKET_CONFIG2, REG_PACKET_CONFIG2_AUTO_RX_RESTART_ON | REG_PACKET_CONFIG2_RESTART_RX);

	/*
	 * Init datastructures
	 */
	state = IDLE;

	for(i = 0; i < MAXPKTS; i++)
	{
		packets[i].clear = true;
		usart_puts("PKT Clear\r\n");
	}


}


void rfm69_writeReg(uint8_t addr, uint8_t data)
{
	select();
	exchangebyte(addr + 0x80);
	exchangebyte(data);
	deselect();
}

void rfm69_writeRegBurst(uint8_t addr, uint8_t* data, uint8_t size)
{
	uint8_t i;

	select();
	exchangebyte(addr | 0x80);

	for(i=0; i < size; i++)
	{
		exchangebyte(data[i]);
	}
	deselect();
}

uint8_t rfm69_readReg(uint8_t addr)
{
	uint8_t temp;
	select();

	exchangebyte(addr&0x7F);

	temp = exchangebyte(0);

	deselect();

	return temp;
}

void rfm69_readRegBurst(uint8_t addr, uint8_t* data, uint8_t size)
{
	uint8_t i;

	select();
	exchangebyte(addr&0x7F);
	if(data != NULL)
	{
		for (i = 0; i < size; i++)
		{
			*data++ = exchangebyte(0);
		}
	}

	else
	{
		for (i = 0; i < size; i++)
		{
			exchangebyte(0);
		}
	}

	deselect();
}


void rfm69_setFdev(uint16_t deviation)
{
	uint8_t temp[2];
	temp[0] = deviation>>8;
	temp[1] = deviation&0xFF;

	rfm69_writeRegBurst(REG_FDEV_MSB, temp, 2);
}

void rfm69_setBitrate(uint16_t bitrate)
{
	uint8_t temp[2];
	temp[0] = bitrate>>8;
	temp[1] = bitrate&0xFF;

	rfm69_writeRegBurst(REG_BITRATE_MSB, temp, 2);
}


void rfm69_setRxBw(uint8_t DccFreq, uint8_t RxBwMant, uint8_t RxBwExp)
{
	uint8_t temp = ((DccFreq<<5)&0xE0) + ((RxBwMant<<3)&0x18) + (RxBwExp&0x07);
	rfm69_writeReg(REG_RX_BW, temp);
}

void rfm69_setPANID(uint8_t panid)
{
	rfm69_writeReg(REG_SYNC_VALUE2, panid);
}


void rfm69_setAddr(uint8_t addr)
{
	rfm69_writeReg(REG_NODE_ADRS, addr);
}

void rfm69_setBroadcastAddr(uint8_t baddr)
{
	rfm69_writeReg(REG_BROADCAST_ADRS, baddr);
}


void rfm69_setKey(uint8_t* key)
{
	rfm69_writeRegBurst(REG_AES_KEY1, key, 16);
}

void rfm69_setEnEnc(bool enable)
{
	uint8_t temp = rfm69_readReg(REG_PACKET_CONFIG2);
	temp &= ~(1<<0);
	temp |= 1 & enable;
	rfm69_writeReg(REG_PACKET_CONFIG2, temp);
}

void rfm69_writeFIFO(uint8_t* data, uint8_t size)
{
	rfm69_writeRegBurst(REG_FIFO, data, size);
}

void rfm69_sendPacket(RFM69PKT_t *pkt)
{
	PORTD &= ~(1<<PD7);

	//switch to TX mode
	rfm69_writeReg(REG_OP_MODE, 0x0C);

	state = TX;

	//set fifo trigger to size - 1; set msb to 1
	rfm69_writeReg(REG_FIFO_THRESH, pkt->size+1);

	//DIO0 if packet sent - connect DIO0 to interrupt
	rfm69_writeReg(REG_DIO_MAPPING1, 0x00);

	//write length to fifo - remember to add one for addr
	rfm69_writeReg(REG_FIFO, pkt->size+1);

	//write addr to fifo
	rfm69_writeReg(REG_FIFO, pkt->dst);

	//write payload to fifo
	rfm69_writeRegBurst(REG_FIFO, pkt->data, pkt->size);
}

RFM69PKT_t* rfm69_getPacket(void)
{
	if(!packets[nextPkt].clear)
	{
		return &packets[nextPkt].pkt;
	}

	return NULL;
}

void rfm69_clearPacket()
{
	if(!packets[nextPkt].clear)
	{
		packets[nextPkt].clear = true;
		++nextPkt;
		nextPkt &= 0b11;
	}
}

void rfm69_RX()
{
	state = RX;

	//SET DIO0 to PAYLOADREADY
	rfm69_writeReg(REG_DIO_MAPPING1, (0b01<<6));

	//Set opmode to RX
	rfm69_writeReg(REG_OP_MODE, REG_OP_MODE_RX);

	rfm69_writeReg(REG_PACKET_CONFIG2, REG_PACKET_CONFIG2_AUTO_RX_RESTART_ON | REG_PACKET_CONFIG2_RESTART_RX);

}

void rfm69_DIO0_INT()
{
	uint8_t size;
	char string[100], i;
	RFM69NETFRAME_t* frame;
	switch (state)
	{
	case IDLE:

		break;

	case RX:
		//Read FIFO
		PORTC &= ~(1<<PC0);
		size = rfm69_readReg(REG_FIFO);
		sprintf(string, "Size: %u\r\n", size);
		//usart_puts(string);
#ifndef NEW_METHOD
		if(NULL != (frame = rfm69net_reqFrame(size+1)))
		{
			frame->packet->size = size-3;

			rfm69_readRegBurst(REG_FIFO, (uint8_t*) &(frame->packet->dst), size);

			/*
			sprintf(string, "Data size: %u\r\n", frame->packet->size);
			usart_puts(string);

			sprintf(string, "Dst=%u\r\n", frame->packet->dst);
			usart_puts(string);

			sprintf(string, "Src=%u\r\n", frame->packet->src);
			usart_puts(string);

			sprintf(string, "Service=%u\r\n", frame->packet->service);
			usart_puts(string);

			memcpy(string, frame->packet->data, frame->packet->size);
			string[frame->packet->size] = 0;
			usart_puts(string);
			usart_puts("\r\n");
			*/

			frame->ready=true;

		}

		else
		{
			rfm69_readRegBurst(REG_FIFO, buffer, size);
		}

#else
		if(packets[currentPkt].clear)
		{
			packets[currentPkt].clear = false;
			packets[currentPkt].pkt.size = size;
			rfm69_readRegBurst(REG_FIFO, &packets[currentPkt].pkt.dst, size);

			++currentPkt;
			currentPkt &= 0b11;
		}

		else
		{
			rfm69_readRegBurst(REG_FIFO, NULL, size);
		}
#endif
		PORTC |= (1<<PC0);
		//PORTD |= (1<<PD7);
		break;

	case TX:
		//sending should be finished here - return to RX
		rfm69_RX();
		PORTD |= (1<<PD7);
		break;
	}
}
