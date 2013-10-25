/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PLMMCU-0_portio.c
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega128A1
 // Description:    Input/Output configuration for Palm board

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            03/06/12    ZAC            Initial Release
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
    //PA1 - 3.3VMON
    //PA2 - NC
    //PA3 - 12VMON
    //PA4 - EXTMP_MON
    //PA5 - NC
    //PA6 - NC
    //PA7 - 48VMON
    PORTA.OUT = 0x00;
    PORTA.DIR = 0x00;

    //PB0 - NC
    //PB1 - NC
    //PB2 - DAC1
    //PB3 - NC
    //PB4 - NC
    //PB5 - SPARE6
    //PB6 - SPARE1
    //PB7 - SPARE2
    PORTB.OUT = 0x00;
    PORTB.DIR = 0x04;

//For both TX_EN and RX_EN lines, set HIGH to TX and LOW to RX
//Wake-up state is RX on both lines

    //PC0 - RX_TAC4_EN
    //PC1 - TX_TAC4_EN
    //PC2 - RX_TAC4
    //PC3 - TX_TAC4
    //PC4 - RX_PROX2_EN
    //PC5 - TX_PROX2_EN
    //PC6 - RX_PROX2
    //PC7 - TX_PROX2
    PORTC.OUT = 0x22;
    PORTC.DIR = 0xBB;


    //PD0 - RX_PROX3_EN
    //PD1 - TX_PROX3_EN
    //PD2 - RX_PROX3
    //PD3 - TX_PROX3
    //PD4 - RX_PROX1_EN
    //PD5 - TX_PROX1_EN
    //PD6 - RX_PROX1
    //PD7 - TX_PROX1
    PORTD.OUT = 0x22;
    PORTD.DIR = 0xBB;

    //PE0 - RX_MTR5_EN
    //PE1 - TX_MTR5_EN
    //PE2 - RX_MTR5
    //PE3 - TX_MTR5
    //PE4 - RX_MTR6_EN
    //PE5 - TX_MTR6_EN
    //PE6 - RX_MTR6
    //PE7 - TX_MTR6
    PORTE.OUT = 0x22;
    PORTE.DIR = 0xBB;

    //PF0 - MOTOR_DIR
    //PF1 - MOTOR_nDIR
    //PF2 - COMM_RX
    //PF3 - COMM_TX
    //PF4 - nCS
    //PF5 - MOSI
    //PF6 - MISO
    //PF7 - SCK
    PORTF.OUT = 0x00;
    PORTF.DIR = 0xDB;

    //PH0 - SPARE3
    //PH1 - SPARE4
    //PH2 - SPARE5
    //PH3 - NC
    //PH4 - NC
    //PH5 - NC
    //PH6 - NC
    //PH7 - NC
    PORTH.OUT = 0x00;
    PORTH.DIR = 0x00;

    //PJ0 - NC
    //PJ1 - NC
    //PJ2 - NC
    //PJ3 - NC
    //PJ4 - NC
    //PJ5 - NC
    //PJ6 - NC
    //PJ7 - NC
    PORTJ.OUT = 0x00;
    PORTJ.DIR = 0x00;

    //PK0 - NC
    //PK1 - NC
    //PK2 - NC
    //PK3 - LEDC
    //PK4 - NC
    //PK5 - NC
    //PK6 - NC
    //PK7 - NC
    PORTK.OUT = 0x00;
    PORTK.DIR = 0x08;

    //PQ0 - NC
    //PQ1 - NC
    //PQ2 - NC
    //PQ3 - NC
    PORTQ.OUT = 0x00;
    PORTQ.DIR = 0x00;

    //PR0 - NC
    //PR1 - NC
    PORTR.OUT = 0x00;
    PORTR.DIR = 0x00;

}
