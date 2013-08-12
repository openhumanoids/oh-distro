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

#include "proximal.h"
#include <avr/io.h>
#include <stdlib.h>

int16_t encoderOffset = 0;

//The system can take 1 MHz.  Peripheral clock is set to 32 MHz internally, so a divide by 32 prescaler should set to 1 MHz
//In this chip, the division is accomplished by a divide by 64 with a doubler

#define ENCODERSPI SPIC
//#define ENCODER_CS13_PORT PORTA
//#define ENCODER_CS13_PIN 0x40
#define PORTA_MUX_EN 0x40
#define PORTC_MUX_MASK 0x1C
#define ENC_PORTC_MUX_ADDR 0x07
#define PORTC_MUX_SHIFT 0x02

void configureSPIModulesEncoder(void)
{
    //This part expects an idling high SCLK, and it wants to setup data on the RISING clock edge and sample on the FALLING.  This is Mode 2

    ENCODERSPI.INTCTRL = SPI_INTLVL_OFF_gc;
    ENCODERSPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_2_gc | SPI_PRESCALER_DIV64_gc | SPI_CLK2X_bm;
}

int16_t readRawEncoder(void)
{
    uint8_t temp;
    uint8_t temp2;

    configureSPIModulesEncoder();
    _delay_us(1);
    //ENCODER_CS13_PORT.OUTCLR = ENCODER_CS13_PIN; //Chip select
    PORTC.OUTSET = ENC_PORTC_MUX_ADDR << PORTC_MUX_SHIFT;
    PORTA.OUTCLR = PORTA_MUX_EN;
    //_delay_us(5);

    ENCODERSPI.DATA = 0x00;
    while(!(ENCODERSPI.STATUS & SPI_IF_bm)); //wait for it to finish
    temp = ENCODERSPI.DATA;

    ENCODERSPI.DATA = 0x00;
    while(!(ENCODERSPI.STATUS & SPI_IF_bm)); //wait for it to finish
    temp2 = ENCODERSPI.DATA;

    //ENCODER_CS13_PORT.OUTSET = ENCODER_CS13_PIN; //Chip select
    PORTA.OUTSET = PORTA_MUX_EN;
    PORTC.OUTCLR = PORTC_MUX_MASK;
    
    //The serial port is likely strange because the encoder will likely have a dummy bit at the front.
    return ((temp << 3) & 0x3F8) | ((temp2 >> 5) & 0x007);
}

int16_t readEncoder(void)
{
    int16_t rawEncoder = readRawEncoder();
    
    int16_t top = (1023 + encoderOffset) - rawEncoder;
    int16_t middle = encoderOffset - rawEncoder;
    int16_t bottom = (encoderOffset - 1023) - rawEncoder;
    
    // find the minimum of the 3 absolute values, then return that value as non-absolute.
    // this only works because the valid range for the sensor is less than 512.
    // this should let you calibrate the joint in any angle and still get valid readings, even negative values.
    if (abs(top) < abs(middle))
    {
        if (abs(top) < abs(bottom)) 
             return top; 
        else 
             return bottom;
    }
    else if (abs(middle) < abs(bottom))
        return middle;
    else 
        return bottom;
}
