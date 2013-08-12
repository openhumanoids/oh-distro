/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PLMMCU-0_encoder.c
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

#include <avr/io.h>
#define F_CPU 32000000UL
#include <util/delay.h>
#include <stdlib.h>

int16_t lastEncoder = 0;
int16_t adjustedEncoder = 0;
uint8_t encoderInitialized = 0;
int16_t rawEncoder = 0;

//The system can take 1 MHz.  Peripheral clock is set to 32 MHz internally, so a divide by 32 prescaler should set to 1 MHz
//In this chip, the division is accomplished by a divide by 64 with a doubler

#define ENCODERSPI SPIF

void configureSPIModulesEncoder(void)
{
    //This part expects an idling high SCLK, and it wants to setup data on the RISING clock edge and sample on the FALLING.  This is Mode 2

    ENCODERSPI.INTCTRL = SPI_INTLVL_OFF_gc;
    ENCODERSPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_2_gc | SPI_PRESCALER_DIV64_gc | SPI_CLK2X_bm;
}

int16_t readEncoder(void)
{
    uint8_t temp;
    uint8_t temp2;
    configureSPIModulesEncoder();
    PORTF.OUTCLR = 0x10;
    _delay_us(1); // do we need this?

    ENCODERSPI.DATA = 0x00;
    while(!(ENCODERSPI.STATUS & SPI_IF_bm)); //wait for it to finish
    temp = ENCODERSPI.DATA;

    ENCODERSPI.DATA = 0x00;
    while(!(ENCODERSPI.STATUS & SPI_IF_bm)); //wait for it to finish
    temp2 = ENCODERSPI.DATA;

    PORTF.OUTSET = 0x10;

    //The serial port is strange because the encoder will likely have a dummy bit at the front.
    rawEncoder = ((temp << 3) & 0x3F8) | ((temp2 >> 5) & 0x007);

    if (!encoderInitialized)
    {
        lastEncoder = rawEncoder;
        adjustedEncoder = 0;
        encoderInitialized = 1;
    }
    
    int16_t delta = rawEncoder - lastEncoder;

    if (abs(delta) < 512)
    {
        lastEncoder = rawEncoder;
        adjustedEncoder = adjustedEncoder + delta;
    }
    else
    {
        int16_t bottom = lastEncoder + (1024 - rawEncoder);
        int16_t top = rawEncoder + (1024 - lastEncoder);
        lastEncoder = rawEncoder;
        if (bottom<top) // crossing 0
            adjustedEncoder = adjustedEncoder - bottom;
        else // crossing 1024
            adjustedEncoder = adjustedEncoder + top;
    }
    
    return adjustedEncoder;
}
