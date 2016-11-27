/*  Copyright (C) 2016  Florian Menne

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SRC_RFM69_RFM69MODEMSETTINGS_H_
#define SRC_RFM69_RFM69MODEMSETTINGS_H_

#include <stdint.h>


#define CONFIG_FSK (REG_DATA_MODUL_DATAMODE_PKT | REG_DATA_MODUL_MODULATION_TYPE_FSK | REG_DATA_MODUL_MODULATION_SHAPING_NO_SHAPING)
#define CONFIG_GFSK (REG_DATA_MODUL_DATAMODE_PKT | REG_DATA_MODUL_MODULATION_TYPE_FSK | REG_DATA_MODUL_MODULATION_SHAPING_GAUSS_BT1_0)
#define CONFIG_OOK (REG_DATA_MODUL_DATAMODE_PKT | REG_DATA_MODUL_MODULATION_TYPE_OOK | REG_DATA_MODUL_MODULATION_SHAPING_NO_SHAPING)

#define CONFIG_NOWHITE (REG_PACKET_CONFIG1_PACKET_FORMAT_VARLEN | REG_PACKET_CONFIG1_DC_FREE_NONE | REG_PACKET_CONFIG1_CRC_ON | REG_PACKET_CONFIG1_ADDR_FILTER_NODE_OR_BRDCAST_ADDR)
#define CONFIG_WHITE (REG_PACKET_CONFIG1_PACKET_FORMAT_VARLEN | REG_PACKET_CONFIG1_DC_FREE_WHITENING | REG_PACKET_CONFIG1_CRC_ON | REG_PACKET_CONFIG1_ADDR_FILTER_NODE_OR_BRDCAST_ADDR)

typedef struct
{
	uint8_t dataModule;
	uint16_t bitrate;
	uint16_t fdev;
	uint8_t rxBw;
	uint8_t afcBw;
	uint8_t packetConfig1;
}RFM69MODEMPARMS_t;

extern const RFM69MODEMPARMS_t MODEM_CONFIG_TABLE[];

#endif /* SRC_RFM69_RFM69MODEMSETTINGS_H_ */
