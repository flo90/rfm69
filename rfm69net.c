/*
 * rfm69net.c
 *
 *  Created on: 26.12.2015
 *      Author: flo
 */


#include "rfm69net.h"
#include "rfm69.h"
#include "../usart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BUFFERSIZE 200
#define MAXFRAMES 10
#define SERVICE_SIZE 10

static uint8_t rxbuffer[BUFFERSIZE];
static RFM69NETFRAME_t frames[MAXFRAMES];

static uint8_t* pCurrentByte = rxbuffer;
static RFM69NETFRAME_t* pCurrentFrame = frames;
static RFM69NETFRAME_t* pNextFrame = frames;

static void (*rfm69net_serviceCallback[SERVICE_SIZE])(RFM69NETFRAME_t* frame);
static void rfm69net_cleanFrame(void);

RFM69NETFRAME_t* rfm69net_getNextFrame(void);

void rfm69net_init()
{
	uint8_t i;

	for(i=0; i< MAXFRAMES; i++)
	{
		frames[i].packet = NULL;
		frames[i].ready = false;
	}

	rfm69_init();
}

void rfm69net_poll()
{
	RFM69PKT_t* pkt;
	RFM69NETFRAME_t* frame;
#ifdef DEBUG
	char string[200];
	char dataString[100];
#endif
	/*
	 * Get data from physical layer
	 */
	while(NULL != (pkt = rfm69_getPacket()))
	{
		if(NULL != (frame = rfm69net_reqFrame(pkt->size-2)))
		{
			frame->packet->dst = pkt->dst;
			memcpy(&frame->packet->src, pkt->data, pkt->size);
#ifdef DEBUG
			memcpy(dataString, frame->packet->data, frame->packet->size);
			sprintf(string, "---\r\nSize=%d\r\nDst=%d\r\nSrc=%d\r\nService=%d\r\nData=%s\r\n---\r\n", frame->packet->size, frame->packet->dst, frame->packet->src, frame->packet->service, dataString);
			usart_puts(string);
#endif
			rfm69_clearPacket();
			frame->ready = true;
		}
		else
		{
			break;
		}
	}

	/*
	 * Proccess data in this layer
	 */
	if(NULL != (frame = rfm69net_getNextFrame()) && frame->packet->service < SERVICE_SIZE)
	{
		if(rfm69net_serviceCallback[frame->packet->service] != NULL)
		{
			rfm69net_serviceCallback[frame->packet->service](frame);
			rfm69net_cleanFrame();
		}
		else
		{

		}

	}
}

void rfm69net_addService(uint8_t service, void (*func)(RFM69NETFRAME_t* frame))
{
	rfm69net_serviceCallback[service] = func;
}

RFM69NETFRAME_t* rfm69net_getNextFrame()
{
	RFM69NETFRAME_t* tempFrame;

	if(pNextFrame->ready)
	{
		tempFrame = pNextFrame;
		return tempFrame;
	}

	return NULL;
}

static void rfm69net_cleanFrame()
{
	if(pNextFrame->ready && pNextFrame->packet != NULL)
	{
		pNextFrame->ready = false;
		pNextFrame->packet = NULL;
	}


	if((++pNextFrame) > &frames[MAXFRAMES-1])
	{
		pNextFrame = frames;
	}
}


RFM69NETFRAME_t* rfm69net_reqFrame(size_t size)
{
	RFM69NETFRAME_t* tempFrame;
	/*
	 * Check for space in Frame structure
	 */
	if(pCurrentFrame->packet != NULL)
	{
		return NULL;
	}

	/*
	 * Check size from beginning to end
	 */
	if((pCurrentByte + size) <= (rxbuffer + BUFFERSIZE-1))
	{
		pCurrentFrame->packet = (RFM69PACKET_t*) pCurrentByte;
		pCurrentByte = pCurrentByte + size;
	}

	else if((pCurrentFrame->packet == NULL) || rxbuffer + size <= (uint8_t*)pCurrentFrame->packet)
	{
		pCurrentFrame->packet = (RFM69PACKET_t*) rxbuffer;
		pCurrentByte = rxbuffer + size;
	}

	else
	{
		return NULL;
	}

	/*
	 * Enough space map everything
	 */

	tempFrame = pCurrentFrame;
	tempFrame->packet->size = size;
	if((++pCurrentFrame) > &frames[MAXFRAMES-1])
	{
		pCurrentFrame = frames;
	}

	return tempFrame;
}

void rfm69net_sendFrame(RFM69NETFRAME_t *frame)
{
	static RFM69PKT_t pkt;

	//In this case size+2 becuase of src and service
	pkt.size = frame->packet->size+2;

	pkt.dst = frame->packet->dst;

	memcpy(pkt.data, &frame->packet->src, pkt.size);

	rfm69_sendPacket(&pkt);
}
