/*
 * rfm69.c
 *
 *  Created on: 28.11.2015
 *      Author: flo
 */


#include "rfm69.h"
#include "rfm69net.h"

#include <stdlib.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

//USE POWER OF 2!!!
#define MAXPKTS 4


typedef enum
{
	IDLE,
	RX,
	TX,
	TXMODSET,
}rfm69State_t;

static volatile rfm69State_t state;

static RFM69INTERFACE_t interface;

static uint8_t currentPkt = 0;
static uint8_t nextPkt = 0;
static RFM69PKTMGMT_t packets[MAXPKTS];

static RFM69MODEMPARMS_t lastModemParams;

static uint8_t lastKey[16];
static bool lastEncState = false;

static bool restoreKey = false;
static bool restoreEncState = false;



void setEnableEncryption(bool enable);
void restoreEncryption(void);

bool rfm69_init(RFM69INTERFACE_t paramInterface)
{
	uint8_t i;

	interface = paramInterface;

	/*
	 * Init datastructures
	 */

	for(i = 0; i < MAXPKTS; i++)
	{
		packets[i].clear = true;
	}

	/*
	 * Geral stuff
	 */

	//Small check if module is accessible
	if( 0x24 != rfm69_readReg(REG_VERSION)) return false;

	rfm69_IDLE();

	//CLK OFF - To save energy
	rfm69_writeReg(REG_DIO_MAPPING2, 0b111);


	/*
	 * Physical stuff here
	 */

	/*
	 * Modem parameters replaced by rfm69_setModemParams
	 */

	/*
	rfm69_writeReg(REG_DATA_MODUL, REG_DATA_MODUL_MODULATION_SHAPING_GAUSS_BT1_0);

	//Bitrate
	rfm69_setBitrate(0x006b);

	//FDEV
	rfm69_setFdev(0x52*64);

	//RxBw
	rfm69_setRxBw(0b010, 0, 0);

	//AfcBw
	rfm69_setAfcBw(0b010, 0, 0);


	rfm69_writeReg(REG_PACKET_CONFIG1, REG_PACKET_CONFIG1_PACKET_FORMAT_VARLEN | REG_PACKET_CONFIG1_CRC_ON | REG_PACKET_CONFIG1_ADDR_FILTER_NODE_OR_BRDCAST_ADDR);

	//Improved AFC
	rfm69_writeReg(REG_AFCCTRL, REG_AFCCTRL_AFC_LOW_BETA_ON);

	//DAGC
	rfm69_writeReg(REG_TEST_DAGC, 0x20);

	rfm69_writeReg(REG_TEST_AFC, 45);
	 */

	rfm69_setModemParameter(MODEM_CONFIG_TABLE[18]);

	//Set Power
	rfm69_setTxPower(-18);

	rfm69_setFrequency(433.92);

	//Set RSSI Threshold
	rfm69_writeReg(REG_RSSI_THRESH, 220);


	/*
	 * Init Packet Management
	 */

	rfm69_writeReg(REG_PREAMBLE_LSB, 3);

	//SYNC ON, 2 SYNC BYTES
	rfm69_writeReg(REG_SYNC_CONFIG, REG_SYNC_CONFIG_SYNC_ON + (1<<3));

	//Sync values - for compatibility with rfm12 Sync is set to 0x2DD4
	rfm69_writeReg(REG_SYNC_VALUE1, 0x2D);

	//Second sync byte is used as PANID and can be changed through rfm69_setPANID
	rfm69_writeReg(REG_SYNC_VALUE2, 0xD4);


	rfm69_writeReg(REG_PAYLOAD_LENGTH, 66);

	//Just a dummy - set by tx routine
	rfm69_writeReg(REG_FIFO_THRESH, 1);


	//Restart RX after packet received
	rfm69_writeReg(REG_PACKET_CONFIG2, REG_PACKET_CONFIG2_AUTO_RX_RESTART_ON | REG_PACKET_CONFIG2_RESTART_RX);

	return true;
}

void rfm69_writeReg(uint8_t addr, uint8_t data)
{
#if SPI_METHOD == 0
	interface.select();
	interface.exchangebyte(addr + 0x80);
	interface.exchangebyte(data);
	interface.deselect();
#else
	uint8_t temp[2];
	temp[0] = addr | 0x80;
	temp[1] = data;

	interface.spiRW(temp, sizeof(temp));
#endif
}

#if SPI_METHOD == 0
void rfm69_writeRegBurst(uint8_t addr, uint8_t* data, uint8_t size)
{
	uint8_t i;

	interface.select();
	interface.exchangebyte(addr | 0x80);

	for(i=0; i < size; i++)
	{
		interface.exchangebyte(data[i]);
	}
	interface.deselect();
}
#endif


uint8_t rfm69_readReg(uint8_t addr)
{

#if SPI_METHOD == 0
	uint8_t temp;
	interface.select();

	interface.exchangebyte(addr&0x7F);

	temp = interface.exchangebyte(0);

	interface.deselect();


#else
	uint8_t temp[2];
	temp[0] = addr & 0x7F;
	interface.spiRW(temp, sizeof(temp));
#endif
	return temp[1];
}

#if SPI_METHOD == 0
void rfm69_readRegBurst(uint8_t addr, uint8_t* data, uint8_t size)
{
	uint8_t i;

	interface.select();
	interface.exchangebyte(addr&0x7F);
	if(data != NULL)
	{
		for (i = 0; i < size; i++)
		{
			*data++ = interface.exchangebyte(0);
		}
	}

	else
	{
		for (i = 0; i < size; i++)
		{
			interface.exchangebyte(0);
		}
	}

	interface.deselect();
}
#endif


void rfm69_setFrequency(float frequency)
{
#if SPI_METHOD == 0
	uint8_t freq[3];
	uint32_t combinedFreq;

	combinedFreq = (uint32_t) ( (float) frequency/61.0f * 1000000.0);

	freq[0] = (combinedFreq>>16) & 0xFF;
	freq[1] = (combinedFreq>>8) & 0xFF;
	freq[2] = (combinedFreq) & 0xFF;

	rfm69_writeRegBurst(REG_FRF_MSB, freq, 3);
#else

	uint8_t freq[4];
	uint32_t combinedFreq;

	combinedFreq = (uint32_t) ( (float) frequency/61.0f * 1000000.0);

	freq[0] = REG_FRF_MSB | 0x80;
	freq[1] = (combinedFreq>>16) & 0xFF;
	freq[2] = (combinedFreq>>8) & 0xFF;
	freq[3] = (combinedFreq) & 0xFF;

	interface.spiRW(freq, sizeof(freq));
#endif
}

void rfm69_setTxPower(int8_t outputPower)
{
	uint8_t regValue;
	if( outputPower < -18 || outputPower > 14 ) return;

	regValue = outputPower + 18;
	regValue |= REG_PA_LEVEL_PA0ON;

	rfm69_writeReg(REG_PA_LEVEL, regValue);
}

void rfm69_setFdev(uint16_t deviation)
{
#if SPI_METHOD == 0
	uint8_t temp[2];
	temp[0] = deviation>>8;
	temp[1] = deviation&0xFF;

	rfm69_writeRegBurst(REG_FDEV_MSB, temp, 2);
#else
	uint8_t temp[3];
	temp[0] = REG_FDEV_MSB | 0x80;
	temp[1] = deviation>>8;
	temp[2] = deviation&0xFF;

	interface.spiRW(temp, sizeof(temp));
#endif
}

void rfm69_setBitrate(uint16_t bitrate)
{
#if SPI_METHOD == 0
	uint8_t temp[2];
	temp[0] = bitrate>>8;
	temp[1] = bitrate&0xFF;

	rfm69_writeRegBurst(REG_BITRATE_MSB, temp, 2);
#else
	uint8_t temp[3];

	temp[0] = REG_BITRATE_MSB | 0x80;
	temp[1] = bitrate>>8;
	temp[2] = bitrate&0xFF;

	interface.spiRW(temp, sizeof(temp));
#endif
}


void rfm69_setRxBw(uint8_t DccFreq, uint8_t RxBwMant, uint8_t RxBwExp)
{
	uint8_t temp = ((DccFreq<<5)&0xE0) + ((RxBwMant<<3)&0x18) + (RxBwExp&0x07);
	rfm69_writeReg(REG_RX_BW, temp);
}

void rfm69_setAfcBw(uint8_t DccFreq, uint8_t RxBwMant, uint8_t RxBwExp)
{
	uint8_t temp = ((DccFreq<<5)&0xE0) + ((RxBwMant<<3)&0x18) + (RxBwExp&0x07);
	rfm69_writeReg(REG_AFC_BW, temp);
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
#if SPI_METHOD == 0
	memcpy(lastKey, key, sizeof(lastKey));
	rfm69_writeRegBurst(REG_AES_KEY1, key, 16);
#else
	uint8_t temp[17];
	temp[0] = REG_AES_KEY1 | 0x80;
	memcpy(temp+1, key, 16);

	interface.spiRW(temp, sizeof(temp));
#endif
}

void rfm69_enableEncryption(bool enable)
{
	lastEncState = enable;
	setEnableEncryption(enable);
}

void setEnableEncryption(bool enable)
{
	uint8_t temp = rfm69_readReg(REG_PACKET_CONFIG2);
	temp &= ~(1<<0);
	temp |= 1 & enable;
	rfm69_writeReg(REG_PACKET_CONFIG2, temp);
}

void restoreEncryption()
{
#if SPI_METHOD == 0
	if(restoreKey) rfm69_writeRegBurst(REG_AES_KEY1, lastKey, 16);
#else
	uint8_t temp[17];
	if(restoreKey)
	{
		temp[0] = REG_AES_KEY1 | 0x80;
		memcpy(temp+1, lastKey, 16);
		interface.spiRW(temp, sizeof(temp));
	}
#endif
	if(restoreEncState) setEnableEncryption(lastEncState);
}

void rfm69_sendPacket(RFM69PKT_t *pkt)
{
	rfm69_writeReg(REG_OP_MODE, REG_OP_MODE_STDBY);

	switch(pkt->enc)
	{
	case RF69PKTENC_OFF:
		if(lastEncState) restoreEncState = true;
		setEnableEncryption(false);
		break;

	case RF69PKTENC_ON:
		if(!lastEncState) restoreEncState = true;
		setEnableEncryption(true);
		break;

	default:
		break;
	}

	if(NULL != pkt->encKey)
	{
#if SPI_METHOD == 0
		rfm69_writeRegBurst(REG_AES_KEY1, pkt->encKey, 16);
#else
		uint8_t temp[17];
		temp[0] = REG_AES_KEY1 | 0x80;
		memcpy(temp+1, lastKey, 16);

		interface.spiRW(temp, sizeof(temp));
#endif
		restoreKey = true;
	}

	//Set fifo trigger to size - 1;
	rfm69_writeReg(REG_FIFO_THRESH, pkt->size+1);

	//Switch to Tx mode
	rfm69_TX();

	//Add one for address
	++pkt->size;

	//Start from size and copy - again add one because of the size byte itself
#if SPI_METHOD == 0
	rfm69_writeRegBurst(REG_FIFO, (uint8_t*) &pkt->size, pkt->size + 1);
#else
	pkt->spiCmdPlaceholder = REG_FIFO | 0x80;

	//Begin from &pkt->spiCmdPlaceholder
	interface.spiRW(&pkt->spiCmdPlaceholder, pkt->size + 2);
#endif

	//Restore original packet size.
	--pkt->size;


}

void rfm69_sendPacketModSet(RFM69PKT_t* pkt, RFM69MODEMPARMS_t modemParams)
{
	rfm69_IDLE();

	state = TXMODSET;
	rfm69_setModemParameter(modemParams);

	rfm69_sendPacket(pkt);
}

RFM69PKT_t* rfm69_getPacket()
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

void rfm69_TX()
{
	if(state != TXMODSET) state = TX;

#if USE_AUTOMODES
	//DIO2 to automode
	rfm69_writeReg(REG_DIO_MAPPING1, (0b11<<2));

	//Setting to tx automode
	rfm69_writeReg(REG_AUTO_MODES, 0x5B);
#else
	//DIO0 if packet sent - connect DIO0 to interrupt
	rfm69_writeReg(REG_DIO_MAPPING1, 0x00);

	//switch to TX mode
	rfm69_writeReg(REG_OP_MODE, 0x0C);
#endif


}

void rfm69_RX()
{
	state = RX;

#if USE_AUTOMODES
	//DIO2 to automode
	rfm69_writeReg(REG_DIO_MAPPING1, (0b11<<2));

	//Setting to auto Mode
	rfm69_writeReg(REG_AUTO_MODES, 0xF2);
#else
	//SET DIO0 to PAYLOADREADY
	rfm69_writeReg(REG_DIO_MAPPING1, (0b01<<6));
#endif
	//Set to RX Mode
	rfm69_writeReg(REG_OP_MODE, REG_OP_MODE_RX);

	rfm69_writeReg(REG_PACKET_CONFIG2, REG_PACKET_CONFIG2_AUTO_RX_RESTART_ON | REG_PACKET_CONFIG2_RESTART_RX);

}

void rfm69_IDLE()
{
	state = IDLE;
	//Disable Auto Mode
	rfm69_writeReg(REG_AUTO_MODES, 0);

	//Switch to standby
	rfm69_writeReg(REG_OP_MODE, REG_OP_MODE_STDBY);

}

void rfm69_setModemParameter(RFM69MODEMPARMS_t mparams)
{
	//Store params.
	if(state != TXMODSET) lastModemParams = mparams;

	rfm69_writeReg(REG_DATA_MODUL, mparams.dataModule);
	rfm69_setBitrate(mparams.bitrate);
	rfm69_setFdev(mparams.fdev);
	rfm69_setRxBw((mparams.rxBw>>5)&0b111, (mparams.rxBw>>3)&0b11, mparams.rxBw&0b111);
	rfm69_setAfcBw((mparams.afcBw>>5)&0b111, (mparams.afcBw>>3)&0b11, mparams.afcBw&0b111);
	rfm69_writeReg(REG_PACKET_CONFIG1, mparams.packetConfig1);
}

void rfm69_interruptHandler()
{
	uint8_t size;
	uint8_t IRQFlags;

#if USE_AUTOMODES
	IRQFlags = rfm69_readReg(REG_IRQ_FLAGS1);
#else
	IRQFlags = rfm69_readReg(REG_IRQ_FLAGS2);
#endif

	switch (state)
	{
	case IDLE:
		break;

	case RX:
		//Read FIFO
#if USE_AUTOMODES
		/*
		 * Checks if automode is unset.
		 * If unset it left the intermediate mode - In RX this means the module received a packet
		 */
		if( (IRQFlags & (1<<1)) ) break;
#else
		//Check if Payload is ready otherwise just continue
		if( ! (IRQFlags & (1<<2)) ) break;
#endif

#if SPI_METHOD == 0
		size = rfm69_readReg(REG_FIFO);
#endif
		if(packets[currentPkt].clear)
		{
			packets[currentPkt].clear = false;
#if SPI_METHOD == 0
			packets[currentPkt].pkt.size = size;
			rfm69_readRegBurst(REG_FIFO, &packets[currentPkt].pkt.dst, size);
#else
			packets[currentPkt].pkt.spiCmdPlaceholder = REG_FIFO;

			interface.spiRW(&packets[currentPkt].pkt.spiCmdPlaceholder, 2);
			size = packets[currentPkt].pkt.size;

			packets[currentPkt].pkt.rxSpiCmdPlaceholder = REG_FIFO;
			interface.spiRW(&packets[currentPkt].pkt.rxSpiCmdPlaceholder, size+1);
			packets[currentPkt].pkt.rxSpiCmdPlaceholder = size;


#endif

			++currentPkt;
			currentPkt &= MAXPKTS-1;
		}

		else
		{
#if SPI_METHOD == 0
			rfm69_readRegBurst(REG_FIFO, NULL, size);
#else
			RFM69PKT_t dummyPkt;
			dummyPkt.spiCmdPlaceholder = REG_FIFO;

			interface.spiRW(&dummyPkt.spiCmdPlaceholder, 2);
			size = dummyPkt.size;

			dummyPkt.rxSpiCmdPlaceholder = REG_FIFO;
			interface.spiRW(&dummyPkt.rxSpiCmdPlaceholder, size+1);
			dummyPkt.rxSpiCmdPlaceholder = size;
#endif
		}
		break;

	case TX:
#if USE_AUTOMODES
		/*
		 * Checks if automode is unset.
		 * If unset it left the intermediate mode - In TX this means the module has sent the packet
		 */
		if( (IRQFlags & (1<<1)) ) break;
#else
		//Check if packet is sent otherwise just continue
		if( ! (IRQFlags & (1<<3) )) break;
#endif

		//sending should be finished here - return to RX
		restoreEncryption();
		rfm69_RX();
		break;

	case TXMODSET:
#if USE_AUTOMODES
		/*
		 * Checks if automode is unset.
		 * If unset it left the intermediate mode - In TX this means the module has sent the packet
		 */
		if( (IRQFlags & (1<<1)) ) break;
#else
		//Check if packet is sent otherwise just continue
		if( ! (IRQFlags & (1<<3) )) break;
#endif
		rfm69_setModemParameter(lastModemParams);
		restoreEncryption();
		rfm69_RX();
		break;


	default:
		break;
	}
}
