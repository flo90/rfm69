/*
 * rfm69.h
 *
 *  Created on: 28.11.2015
 *      Author: flo
 */

#ifndef SRC_RFM69_RFM69_H_
#define SRC_RFM69_RFM69_H_

#include <inttypes.h>
#include <stdbool.h>

/*
 * Register definition
 */

/*
 * Read Command
 */
#define REG_FIFO 0x00


/*
 * Op Mode
 */
#define REG_OP_MODE 0x01

#define REG_OP_MODE_SequencerOff (1<<7)
#define REG_OP_MODE_ListenOn (1<<6)
#define REG_OP_MODE_ListenAbort (1<<5)

#define REG_OP_MODE_SLEEP	(0b000<<2)
#define REG_OP_MODE_STDBY	(0b001<<2)
#define REG_OP_MODE_FS		(0b010<<2)
#define REG_OP_MODE_TX		(0b011<<2)
#define REG_OP_MODE_RX		(0b100<<2)

//#define REG_OPMODE_Mode ()



#define REG_DATA_MODUL 0x02

#define REG_DATA_MODUL_DATAMODE_PKT				(0<<5)
#define REG_DATA_MODUL_DATAMODE_CONTINUOUS_BITSYNC (0b10<<5)
#define REG_DATA_MODUL_DATAMODE_CONTINUOUS			(0b11<<5)

#define REG_DATA_MODUL_MODULATION_TYPE_FSK	(0b00<<3)
#define REG_DATA_MODUL_MODULATION_TYPE_OOK	(0b01<<3)

#define REG_DATA_MODUL_MODULATION_SHAPING_NO_SHAPING	(0b00)
#define REG_DATA_MODUL_MODULATION_SHAPING_GAUSS_BT1_0	(0b01)
#define REG_DATA_MODUL_MODULATION_SHAPING_GAUSS_BT0_5	(0b10)
#define REG_DATA_MODUL_MODULATION_SHAPING_GAUSS_BT0_3	(0b11)


#define REG_BITRATE_MSB 0x03
#define REG_BITRATE_LSB 0x04

#define REG_FDEV_MSB 0x05
#define REG_FDEV_LSB 0x06

#define REG_FRF_MSB 0x07
#define REG_FRF_MID 0x08
#define REG_FRF_LSB 0x09

#define REG_OSC1 0x0A

#define REG_OSC1_RC_CAL_START 	(1<<7)
#define REG_OSC1_RC_CAL_DONE 	(1<<6)

#define REG_AFCCTRL 0x0B

#define REG_AFCCTRL_AFC_LOW_BETA_ON (1<<5)

#define REG_LISTEN1 0x0D

#define REG_LISTEN1_LISTEN_RESOL_IDLE_64US (0b01<<6)
#define REG_LISTEN1_LISTEN_RESOL_IDLE_4MS1 (0b10<<6)
#define REG_LISTEN1_LISTEN_RESOL_IDLE_262MS (0b11<<6)

#define REG_LISTEN1_LISTEN_RESOL_RX_64US (0b01<<4)
#define REG_LISTEN1_LISTEN_RESOL_RX_4MS1 (0b10<<4)
#define REG_LISTEN1_LISTEN_RESOL_RX_262MS (0b11<<4)

#define REG_LISTEN1_LISTEN_CRITERIA_RSSI (0<<3)
#define REG_LISTEN1_LISTEN_CRITERIA_RSSI_AND_SYNC (1<<3)

#define REG_LISTEN1_LISTEN_END_STAY_RX (0b00<<1)
#define REG_LISTEN1_LISTEN_END_GOTO_MODE (0b01<<1)
#define REG_LISTEN1_LISTEN_END_RESUME_IDLE (0b10<<1)

#define REG_LISTEN2 0x0E
#define REG_LISTEN3 0x0F

#define REG_VERSION 0x10

#define REG_PA_LEVEL 0x11

#define REG_PA_LEVEL_PA0ON (1<<7)
#define REG_PA_LEVEL_PA1ON (1<<6)
#define REG_PA_LEVEL_PA2ON (1<<5)

#define REG_PA_RAMP 0x12

#define REG_PA_RAMP_3MS4 (0b0000)
#define REG_PA_RAMP_2MS (0b0001)
#define REG_PA_RAMP_1MS (0b0010)
#define REG_PA_RAMP_500US (0b0011)
#define REG_PA_RAMP_250US (0b0100)
#define REG_PA_RAMP_100US (0b0101)
#define REG_PA_RAMP_62US (0b0111)
#define REG_PA_RAMP_50US (0b1000)
#define REG_PA_RAMP_40US (0b1001)
#define REG_PA_RAMP_31US (0b1010)
#define REG_PA_RAMP_25US (0b1011)
#define REG_PA_RAMP_20US (0b1100)
#define REG_PA_RAMP_15US (0b1101)
#define REG_PA_RAMP_12US (0b1110)
#define REG_PA_RAMP_10US (0b1111)




#define REG_OCP 0x13

#define REG_OCP_OCP_ON (1<<4)


#define REG_LNA 0x18

#define REG_LNA_LNA_ZIN_50OHMS (0<<7)
#define REG_LNA_LNA_ZIN_200OHMS (1<<7)



#define REG_RX_BW 0x19

#define REG_AFC_BW 0x1A
#define REG_OOK_PEAK 0x1B
#define REG_OOK_AVG 0x1C
#define REG_OOK_FIX 0x1D
#define REG_AFC_FEI 0x1E
#define REG_AFC_MSB 0x1F
#define REG_AFC_LSB 0x20
#define REG_FEI_MSB 0x21
#define REG_FEI_LSB 0x22


#define REG_RSSI_CONFIG 0x23

#define REG_RSSI_CONFIG_RSSI_START (1<<0)

#define REG_RSSI_VALUE 0x24
#define REG_DIO_MAPPING1 0x25

#define REG_DIO_MAPPING2 0x26

#define REG_DIO_MAPPING2_CLK_FXOSC (0b000)
#define REG_DIO_MAPPING2_CLK_FXOSC2 (0b001)
#define REG_DIO_MAPPING2_CLK_FXOSC4 (0b010)
#define REG_DIO_MAPPING2_CLK_FXOSC8 (0b011)
#define REG_DIO_MAPPING2_CLK_FXOSC16 (0b100)
#define REG_DIO_MAPPING2_CLK_FXOSC32 (0b101)
#define REG_DIO_MAPPING2_CLK_RC (0b110)
#define REG_DIO_MAPPING2_CLK_OFF (0b111)

#define REG_IRQ_FLAGS1 0x27
#define REG_IRQ_FLAGS2 0x28
#define REG_RSSI_THRESH 0x29
#define REG_RX_TIMEOUT1 0x2A
#define REG_RX_TIMEOUT2 0x2B
#define REG_PREAMBLE_MSB 0x2C
#define REG_PREAMBLE_LSB 0x2D

#define REG_SYNC_CONFIG 0x2E

#define REG_SYNC_CONFIG_SYNC_ON (1<<7)
#define REG_SYNC_CONFIG_FIFO_FILL_COND_SYNC_ADDR (0<<6)
#define REG_SYNC_CONFIG_FIFO_FILL_COND_SET (1<<6)

#define REG_SYNC_VALUE1 0x2F
#define REG_SYNC_VALUE2 0x30
#define REG_SYNC_VALUE3 0x31
#define REG_SYNC_VALUE4 0x32
#define REG_SYNC_VALUE5 0x33
#define REG_SYNC_VALUE6 0x34
#define REG_SYNC_VALUE7 0x35
#define REG_SYNC_VALUE8 0x36

#define REG_PACKET_CONFIG1 0x37

#define REG_PACKET_CONFIG1_PACKET_FORMAT_FIXEDLEN (0<<7)
#define REG_PACKET_CONFIG1_PACKET_FORMAT_VARLEN (1<<7)

#define REG_PACKET_CONFIG1_DC_FREE_NONE (0<<5)
#define REG_PACKET_CONFIG1_DC_FREE_MANCHESTER (0b01<<5)
#define REG_PACKET_CONFIG1_DC_FREE_WHITENING (0b10<<5)

#define REG_PACKET_CONFIG1_CRC_ON (1<<4)

#define REG_PACKET_CONFIG1_CRC_AUTO_CLEAR_OFF (1<<3)

#define REG_PACKET_CONFIG1_ADDR_FILTER_OFF (0b00<<1)
#define REG_PACKET_CONFIG1_ADDR_FILTER_NODE_ADDR (0b01<<1)
#define REG_PACKET_CONFIG1_ADDR_FILTER_NODE_OR_BRDCAST_ADDR (0b10<<1)


#define REG_PAYLOAD_LENGTH 0x38
#define REG_NODE_ADRS 0x39
#define REG_BROADCAST_ADRS 0x3A
#define REG_AUTO_MODES 0x3B

#define REG_FIFO_THRESH 0x3C

#define REG_FIFO_THRESH_TX_START_COND_FIFO_LEVEL (0<<7)
#define REG_FIFO_THRESH_TX_START_COND_FIFO_NOT_EMPTY (1<<7)

#define REG_PACKET_CONFIG2 0x3D

#define REG_PACKET_CONFIG2_RESTART_RX (1<<2)
#define REG_PACKET_CONFIG2_AUTO_RX_RESTART_ON (1<<1)
#define REG_PACKET_CONFIG2_AES_ON (1<<0)

#define REG_AES_KEY1 0x3E
#define REG_AES_KEY2 0x3F
#define REG_AES_KEY3 0x40
#define REG_AES_KEY4 0x41
#define REG_AES_KEY5 0x42
#define REG_AES_KEY6 0x43
#define REG_AES_KEY7 0x44
#define REG_AES_KEY8 0x45
#define REG_AES_KEY9 0x46
#define REG_AES_KEY10 0x47
#define REG_AES_KEY11 0x48
#define REG_AES_KEY12 0x49
#define REG_AES_KEY13 0x4A
#define REG_AES_KEY14 0x4B
#define REG_AES_KEY15 0x4C
#define REG_AES_KEY16 0x4D

#define REG_TEMP1 0x4E
#define REG_TEMP2 0x4F

#define REG_TEST_LNA 0x58
#define REG_TEST_PA1 0x5A
#define REG_TEST_PA2 0x5C
#define REG_TEST_DAGC 0x6F
#define REG_TEST_AFC 0x71

/*
 * Begin Structs
 */
typedef struct
{
	uint8_t size;
	uint8_t dst;
	uint8_t data[64];
}RFM69PKT_t;

typedef struct
{
	bool clear;
	RFM69PKT_t pkt;
}RFM69PKTMGMT_t;

/*
 * Begin function prototypes
 */


void rfm69_init(void);

uint8_t rfm69_readReg(uint8_t addr);
void rfm69_writeReg(uint8_t addr, uint8_t data);


void rfm69_writeRegBurst(uint8_t addr, uint8_t* data, uint8_t size);
void rfm69_readRegBurst(uint8_t addr, uint8_t* data, uint8_t size);


void rfm69_setFdev(uint16_t deviation);
void rfm69_setBitrate(uint16_t bitrate);
void rfm69_setRxBw(uint8_t DccFreq, uint8_t RxBwMant, uint8_t RxBwExp);

void rfm69_setPANID(uint8_t panid);
void rfm69_setAddr(uint8_t addr);
void rfm69_setBroadcastAddr(uint8_t baddr);


void rfm69_setKey(uint8_t* key);
void rfm69_setEnEnc(bool enable);

void rfm69_DIO0_INT(void);

void rfm69_writeFIFO(uint8_t* data, uint8_t size);
void rfm69_readFIFO(void);

void rfm69_sendPacket(RFM69PKT_t *pkt);
RFM69PKT_t* rfm69_getPacket(void);
void rfm69_clearPacket(void);

void rfm69_RX(void);

void rfm69_TX(void);

#endif /* SRC_RFM69_RFM69_H_ */
