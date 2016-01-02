/*
 * rfm69net.h
 *
 *  Created on: 26.12.2015
 *      Author: flo
 */

#ifndef SRC_RFM69_RFM69NET_H_
#define SRC_RFM69_RFM69NET_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	uint8_t size;
	uint8_t dst;
	uint8_t src;
	uint8_t service;
	uint8_t data[];
}RFM69PACKET_t;

typedef struct
{
	RFM69PACKET_t* packet;
	bool ready;
}RFM69NETFRAME_t;



void rfm69net_init(void);
void rfm69net_poll(void);


/*
 * Connection Module <-> stack
 */
RFM69NETFRAME_t* rfm69net_reqFrame(uint16_t size);



/*
 * Connection user side <-> stack
 */
void rfm69net_sendFrame(RFM69NETFRAME_t *frame);
void rfm69net_addService(uint8_t service, void (*func)(RFM69NETFRAME_t* frame));


#endif /* SRC_RFM69_RFM69NET_H_ */
