/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0_portio.c
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Input/Output configuration for Proximal/Distal boards

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

#include <avr/io.h>

#include "portio.h"

/************************************************************************
* configurePortIODistal()
* This function configures the peripheral I/O ports.  The direction,
* default out state, and drivers are set up here.
************************************************************************/
void configurePortIODistal(void)
{
    //First setup the GPIOs by port.

    //PA0 - VREF
    //PA1 - DISTAL1
    //PA2 - DISTAL2
    //PA3 - DISTAL3
    //PA4 - DISTAL4
    //PA5 - SPARE1
    //PA6 - NC
    //PA7 - NC
    PORTA.OUT = 0x00;
    PORTA.DIR = 0x00;

    //PB0 - nCS1
    //PB1 - nCS2
    //PB2 - nCS3
    //PB3 - nCS4
    PORTB.OUT = 0x0F;
    PORTB.DIR = 0x0F;

    //PC0 - LED
    //PC1 - nCS5
    //PC2 - nCS6
    //PC3 - nCS7
    //PC4 - nCS8
    //PC5 - MOSI
    //PC6 - MISO
    //PC7 - SCK
    PORTC.OUT = 0x1e;
    PORTC.DIR = 0xBF;


    //For both TX_EN and RX_EN lines, set HIGH to TX and LOW to RX

        //PD0 - INBOUND_RX_nEN
    //PD1 - INBOUND_TX_EN
    //PD2 - RX_INBOUND
    //PD3 - TX_INBOUND
    //PD4 - NC
    //PD5 - ACC MOSI
    //PD6 - ACC MISO
    //PD7 - ACC_SCK
    PORTD.OUT = 0x02;
    PORTD.DIR = 0xAB;

    //PE0 - NC
    //PE1 - nCS9
    //PE2 - nCS10
    //PE3 - nCS11
    PORTE.OUT = 0x0E;
    PORTE.DIR = 0x0E;
    
    //PR0 - INT1
    //PR1 - INT2
    PORTR.OUT = 0x00;
    PORTR.DIR = 0x00;

}
