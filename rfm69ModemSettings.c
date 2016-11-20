/*
 * rfm69ModemSettings.c
 *
 *  Created on: 20.11.2016
 *      Author: flo
 */

#include "rfm69ModemSettings.h"
#include "rfm69.h"

/*
 * This configuration is from http://www.airspayce.com/mikem/arduino/RadioHead/
 */

const RFM69MODEMPARMS_t MODEM_CONFIG_TABLE[] =
{
    //  02,        03,   04,   05,   06,   19,   1a,  37
    // FSK, No Manchester, no shaping, whitening, CRC, no address filtering
    // AFC BW == RX BW == 2 x bit rate
    // Low modulation indexes of ~ 1 at slow speeds do not seem to work very well. Choose MI of 2.
    { CONFIG_FSK,  0x3e80, 0x0052, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb2Fd5
    { CONFIG_FSK,  0x3415, 0x004f, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb2_4Fd4_8
    { CONFIG_FSK,  0x1a0b, 0x009d, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb4_8Fd9_6

    { CONFIG_FSK,  0x0d05, 0x013b, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb9_6Fd19_2
    { CONFIG_FSK,  0x0683, 0x0275, 0xf3, 0xf3, CONFIG_WHITE}, // FSK_Rb19_2Fd38_4
    { CONFIG_FSK,  0x0341, 0x04ea, 0xf2, 0xf2, CONFIG_WHITE}, // FSK_Rb38_4Fd76_8

    { CONFIG_FSK,  0x022c, 0x07ae, 0xe2, 0xe2, CONFIG_WHITE}, // FSK_Rb57_6Fd120
    { CONFIG_FSK,  0x0100, 0x0800, 0xe1, 0xe1, CONFIG_WHITE}, // FSK_Rb125Fd125
    { CONFIG_FSK,  0x0080, 0x1000, 0xe0, 0xe0, CONFIG_WHITE}, // FSK_Rb250Fd250
    { CONFIG_FSK,  0x0240, 0x0333, 0x42, 0x42, CONFIG_WHITE}, // FSK_Rb55555Fd50

    //  02,        03,   04,   05,   06,   19,   1a,  37
    // GFSK (BT=1.0), No Manchester, whitening, CRC, no address filtering
    // AFC BW == RX BW == 2 x bit rate
    { CONFIG_GFSK, 0x3e80, 0x0052, 0xf4, 0xf5, CONFIG_WHITE}, // GFSK_Rb2Fd5
    { CONFIG_GFSK, 0x3415, 0x004f, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb2_4Fd4_8
    { CONFIG_GFSK, 0x1a0b, 0x009d, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb4_8Fd9_6

    { CONFIG_GFSK, 0x0d05, 0x013b, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb9_6Fd19_2
    { CONFIG_GFSK, 0x0683, 0x0275, 0xf3, 0xf3, CONFIG_WHITE}, // GFSK_Rb19_2Fd38_4
    { CONFIG_GFSK, 0x0341, 0x04ea, 0xf2, 0xf2, CONFIG_WHITE}, // GFSK_Rb38_4Fd76_8

    { CONFIG_GFSK, 0x022c, 0x07ae, 0xe2, 0xe2, CONFIG_WHITE}, // GFSK_Rb57_6Fd120
    { CONFIG_GFSK, 0x0100, 0x0800, 0xe1, 0xe1, CONFIG_WHITE}, // GFSK_Rb125Fd125
    { CONFIG_GFSK, 0x0080, 0x1000, 0xe0, 0xe0, CONFIG_WHITE}, // GFSK_Rb250Fd250
    { CONFIG_GFSK, 0x0240, 0x0333, 0x42, 0x42, CONFIG_WHITE}, // GFSK_Rb55555Fd50

    //  02,        03,   04,   05,   06,   19,   1a,  37
    // OOK, No Manchester, no shaping, whitening, CRC, no address filtering
    // with the help of the SX1231 configuration program
    // AFC BW == RX BW
    // All OOK configs have the default:
    // Threshold Type: Peak
    // Peak Threshold Step: 0.5dB
    // Peak threshiold dec: ONce per chip
    // Fixed threshold: 6dB
    { CONFIG_OOK,  0x7d00, 0x0010, 0x88, 0x88, CONFIG_WHITE}, // OOK_Rb1Bw1
    { CONFIG_OOK,  0x682b, 0x0010, 0xf1, 0xf1, CONFIG_WHITE}, // OOK_Rb1_2Bw75
    { CONFIG_OOK,  0x3415, 0x0010, 0xf5, 0xf5, CONFIG_WHITE}, // OOK_Rb2_4Bw4_8
    { CONFIG_OOK,  0x1a0b, 0x0010, 0xf4, 0xf4, CONFIG_WHITE}, // OOK_Rb4_8Bw9_6
    { CONFIG_OOK,  0x0d05, 0x0010, 0xf3, 0xf3, CONFIG_WHITE}, // OOK_Rb9_6Bw19_2
    { CONFIG_OOK,  0x0683, 0x0010, 0xf2, 0xf2, CONFIG_WHITE}, // OOK_Rb19_2Bw38_4
    { CONFIG_OOK,  0x03e8, 0x0010, 0xe2, 0xe2, CONFIG_WHITE}, // OOK_Rb32Bw64

//    { CONFIG_FSK,  0x68, 0x2b, 0x00, 0x52, 0x55, 0x55, CONFIG_WHITE}, // works: Rb1200 Fd 5000 bw10000, DCC 400
//    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x52, 0x52, CONFIG_WHITE}, // works 10/40/80
//    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x53, 0x53, CONFIG_WHITE}, // works 10/40/40

};
