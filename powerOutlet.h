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



/**
 * Generates and sends an packet matching the protocol of the HX2262 used in many RC power outlets.
 * @param cmd 12 character long command. Three states are possible: Zero(0), One(1), Float(f/F) representet by the characters
 * @return 0 = OK, -1 = Too few characters, -2 = Unsupported character
 */
int8_t powerOutlet_hx2262(const char* cmd);

/**
 * Wrapper which creates a matching sequence for Elro AB440 power outlets.
 * The Elro AB440 useses the symbols Zero and Float only, settable by the DIP switches.
 * Whereas a Zero is used if the DIP switch is on the ON (Up) position and Float otherwise.
 * On the Remote A B C D are coded by the 6th 7th, 8th and 9th Symbol. Pressing A on or off sets the 6th Symbol to 0.
 * Tip: If you don't need to use the original remote, you can leave the concept described before and set all 10 DIP switches
 * on the power outlet as you like.
 * @param devCode 10 character device code which consist of 0 (DIP switch off / Down) or 1 (DIP switch on / Up)
 * @param state true = Power outlet on, false = Power outlet off
 * @return 0 = OK, -1 = Too few characters, -2 = Unsupported character
 */
int8_t powerOutlet_elroab440(const char* devCode, bool state);



#endif /* SRC_RFM69_POWEROUTLET_H_ */
