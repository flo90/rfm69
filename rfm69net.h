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

#ifndef SRC_RFM69_RFM69NET_H_
#define SRC_RFM69_RFM69NET_H_

#include <stdint.h>
#include <stdbool.h>

#include "rfm69.h"


typedef struct
{
	uint8_t* encKey;
	uint8_t enc;

	uint8_t size;
	uint8_t dst;
	uint8_t src;
	uint8_t service;
	uint8_t* data;
}RFM69NETFRAME_t;



void rfm69net_init(RFM69INTERFACE_t paramInterface);
void rfm69net_poll(void);

/*
 * Connection user side <-> stack
 */
void rfm69net_sendFrame(RFM69NETFRAME_t *frame);
void rfm69net_addService(uint8_t service, void (*func)(RFM69NETFRAME_t* frame));


#endif /* SRC_RFM69_RFM69NET_H_ */
