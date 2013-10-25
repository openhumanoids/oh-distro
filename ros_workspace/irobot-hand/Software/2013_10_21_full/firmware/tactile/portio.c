/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-TACMCU-0_portio.c
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Input/Output configuration for Palm board

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
* configurePortIO()
* This function configures the peripheral I/O ports.  The direction,
* default out state, and drivers are set up here.
************************************************************************/
void configurePortIO(void)
{
    //First setup the GPIOs by port.

    //PA0 - VREF
    //PA1 - NC
    //PA2 - NC
    //PA3 - NC
    //PA4 - NC
    //PA5 - NC
    //PA6 - NC
    //PA7 - NC
    PORTA.OUT = 0x00;
    PORTA.DIR = 0x00;

    //PB0 - SPARE2
    //PB1 - NC
    //PB2 - NC
    //PB3 - LED
    PORTB.OUT = 0x00;
    PORTB.DIR = 0x08;

    //For both TX_EN and RX_EN lines, set HIGH to TX and LOW to RX
    //Wake-up state is RX on both lines

    //PC0 - RX_PROX1_EN
    //PC1 - TX_PROX1_EN
    //PC2 - RX_PROX1
    //PC3 - TX_PROX1
    //PC4 - SPARE1 -- NOTE: This is the physical SS signal.  It cannot be a general purpose input without breaking SPI.  It can be GP output
    //PC5 - MOSI_1
    //PC6 - MISO_1
    //PC7 - SCK_1
    PORTC.OUT = 0x0A;
    PORTC.DIR = 0xBB;


    //PD0 - A0
    //PD1 - A1
    //PD2 - A2
    //PD3 - nCSS1
    //PD4 - nCSS2
    //PD5 - MOSI_2
    //PD6 - MISO_2
    //PD7 - SCK_2
    PORTD.OUT = 0x18;
    PORTD.DIR = 0xBF;

    //PE0 - nCSS3
    //PE1 - nCSS4
    //PE2 - nCSS5
    //PE3 - nCSS6
    PORTE.OUT = 0x0F;
    PORTE.DIR = 0x0F;

    //PR0 - NC
    //PR1 - SPARE3
    PORTR.OUT = 0x00;
    PORTR.DIR = 0x00;

}
