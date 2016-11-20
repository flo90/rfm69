/*
 * powerOutlet.h
 *
 *  Created on: 05.11.2016
 *      Author: flo
 */

#ifndef SRC_RFM69_POWEROUTLET_H_
#define SRC_RFM69_POWEROUTLET_H_

#include <stdint.h>
#include <stdbool.h>

int8_t powerOutlet_hx2262(const char* cmd);

int8_t powerOutlet_elroab440(const char* devCode, bool state);



#endif /* SRC_RFM69_POWEROUTLET_H_ */
