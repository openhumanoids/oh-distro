/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0_encoder.c
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Encoder driver

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

#include "distal.h"
#include <avr/io.h>
#include "accel.h"

//The system can take 1 MHz.  Peripheral clock is set to 32 MHz internally, so a divide by 32 prescaler should set to 1 MHz
//In this chip, the division is accomplished by a divide by 64 with a doubler

#define ENCODERSPI SPIC

void configureSPIModulesEncoder(void)
{
    //This part expects an idling high SCLK, and it wants to setup data on the RISING clock edge and sample on the FALLING.  This is Mode 2

    ENCODERSPI.INTCTRL = SPI_INTLVL_OFF_gc;
    ENCODERSPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_2_gc | SPI_PRESCALER_DIV64_gc | SPI_CLK2X_bm;
}

uint16_t readEncoder(void)
{
    uint8_t temp;
    uint8_t temp2;
    configureSPIModulesEncoder();
    _delay_us(1);

    ENCODERSPI.DATA = 0x00;
    while(!(ENCODERSPI.STATUS & SPI_IF_bm)); //wait for it to finish
    temp = ENCODERSPI.DATA;

    ENCODERSPI.DATA = 0x00;
    while(!(ENCODERSPI.STATUS & SPI_IF_bm)); //wait for it to finish
    temp2 = ENCODERSPI.DATA;

    //The serial port is likely strange because the encoder will likely have a dummy bit at the front.
    return ((temp << 3) & 0x3F8) | ((temp2 >> 5) & 0x007); //BA: shift left 3 instead of 4
}
