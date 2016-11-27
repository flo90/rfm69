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

#include "rfm69net.h"
#include "rfm69.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BUFFERSIZE 200
#define MAXFRAMES 10
#define SERVICE_SIZE 10

static void (*rfm69net_serviceCallback[SERVICE_SIZE])(RFM69NETFRAME_t* frame);

void rfm69net_init(RFM69INTERFACE_t paramInterface)
{
	rfm69_init(paramInterface);
}

void rfm69net_poll()
{
	RFM69PKT_t* pkt;
	RFM69NETFRAME_t frame;
#ifdef DEBUG
	char string[200];
	char dataString[100];
#endif
	/*
	 * Get data from physical layer
	 */
	while(NULL != (pkt = rfm69_getPacket()))
	{
		frame.size = pkt->size-2;
		frame.dst = pkt->dst;
		frame.src = pkt->data[0];
		frame.service = pkt->data[1];
		frame.data = (uint8_t*) &pkt->data[2];


		if(frame.service < SERVICE_SIZE && rfm69net_serviceCallback[frame.service] != NULL)
		{
			rfm69net_serviceCallback[frame.service](&frame);
		}

#ifdef DEBUG
			memcpy(dataString, frame->packet->data, frame->packet->size);
			sprintf(string, "---\r\nSize=%d\r\nDst=%d\r\nSrc=%d\r\nService=%d\r\nData=%s\r\n---\r\n", frame->packet->size, frame->packet->dst, frame->packet->src, frame->packet->service, dataString);
			usart_puts(string);
#endif
		rfm69_clearPacket();

	}

}

void rfm69net_addService(uint8_t service, void (*func)(RFM69NETFRAME_t* frame))
{
	rfm69net_serviceCallback[service] = func;
}

void rfm69net_sendFrame(RFM69NETFRAME_t *frame)
{
	static RFM69PKT_t pkt;

	pkt.encKey = NULL;
	pkt.enc	= RF69PKTENC_DEFAULT;

	//In this case size+2 because of src and service
	pkt.size = frame->size+2;
	pkt.dst = frame->dst;
	pkt.data[0] = frame->dst;
	pkt.data[1] = frame->service;


	memcpy(&pkt.data[2], frame->data, pkt.size);

	rfm69_sendPacket(&pkt);
}
