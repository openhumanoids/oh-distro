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
* configurePortIOProximal()
* This function configures the peripheral I/O ports.  The direction,
* default out state, and drivers are set up here.
************************************************************************/
void configurePortIOProximal(void)
{
    //First setup the GPIOs by port.

    //PA0 - VREF
    //PA1 - PROXIMAL1
    //PA2 - PROXIMAL2
    //PA3 - PROXIMAL3
    //PA4 - PROXIMAL4
    //PA5 - nCS6                 nCS12
    //PA6 - MUX nE1              nCS13
    //PA7 - ACC nCS
    PORTA.OUT = 0xE0; //E0
    PORTA.DIR = 0xE0; //E0

    //PB0 - nCS1
    //PB1 - nCS2
    //PB2 - nCS3
    //PB3 - nCS4
    PORTB.OUT = 0x0F;
    PORTB.DIR = 0x0F;

    //PC0 - nLED
    //PC1 - nCS5
    //PC2 - MUX A0              nCS6
    //PC3 - MUX A1              nCS7
    //PC4 - MUX A2              nCS8
    //PC5 - MOSI
    //PC6 - MISO
    //PC7 - SCK
    PORTC.OUT = 0x1D; //01
    PORTC.DIR = 0xBF; //BF


    //For both TX_EN and RX_EN lines, set HIGH to TX and LOW to RX

    //PD0 - OUTBOUND_nEN_RX
    //PD1 - OUTBOUND_EN_TX
    //PD2 - RX_OUTBOUND
    //PD3 - TX_OUTBOUND
    //PD4 - INBOUND_nEN_RX
    //PD5 - ACC MOSI                     INBOUND_EN_TX
    //PD6 - ACC MISO                     RX_INBOUND
    //PD7 - ACC SCK                      TX_INBOUND
    PORTD.OUT = 0x02; //22
    PORTD.DIR = 0xBB; //BB

    //PE0 - NC                    LED_PWM
    //PE1 - INBOUND_EN_TX         nCS9
    //PE2 - RX_INBOUND            nCS10
    //PE3 - TX_INBOUND            nCS11
    PORTE.OUT = 0x02; //0F
    PORTE.DIR = 0x0A; //0F

    //PR0 - ACC INT2
    //PR1 - ACC INT1
    PORTR.OUT = 0x00;
    PORTR.DIR = 0x00;

}

/************************************************************************
* configurePortIODistal()
* This function configures the peripheral I/O ports.  The direction,
* default out state, and drivers are set up here.
************************************************************************/
void configurePortIODistal(void)
{
  configurePortIOProximal();
}
