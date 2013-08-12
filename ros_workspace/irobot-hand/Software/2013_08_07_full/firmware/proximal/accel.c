/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0_tactsense.c
 // Creation Date:    5 March, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Accelerometer sensor driver

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
#include "accel.h"
#include "proximal.h"

//The system can take 5 MHz.  Peripheral clock is set to 32 MHz internally, so a divide by 8 prescaler should set to 4 MHz
//In this chip, the division is accomplished by a divide by 16 with a doubler

#define ACCELSPI SPID
#define ACCELSPI_PORT PORTD
#define ACCEL_CS_PORT PORTA     // chip select
#define ACCEL_CS_PIN_MASK 0x80  // chip select pin

static void writeRegisterSPI(uint8_t addr, uint8_t data);

void configureSPIModulesAccel(void)
{
  ACCEL_CS_PORT.OUTSET = ACCEL_CS_PIN_MASK; // default chip select to high
  ACCEL_CS_PORT.DIRSET = ACCEL_CS_PIN_MASK; // make sure it is an output
  ACCELSPI_PORT.DIRCLR = 0x40;
  ACCELSPI_PORT.DIRSET = 0xB0;
  //ACCELSPI_PORT.OUTSET = 0x10;
  ACCELSPI.INTCTRL = SPI_INTLVL_OFF_gc;
  ACCELSPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_3_gc | SPI_PRESCALER_DIV16_gc | SPI_CLK2X_bm;
}

/*
 * Abstraction Function for ReadWriting Registers On The Accelerometer
 */
static void writeRegisterSPI(uint8_t addr, uint8_t data)
{
  uint8_t dummy;
  ACCEL_CS_PORT.OUTCLR = ACCEL_CS_PIN_MASK; // chip select

  //ADDR should be 5 bits in length.  Bit 6 should be low for single write
  //bit 7 should be low to indicate WRITE
  ACCELSPI.DATA = addr;
  while(!(ACCELSPI.STATUS & SPI_IF_bm)) {
    dummy = ACCELSPI.DATA; } //wait for it to finish

  // the dummy read is needed for the SPI_IF flag to clear

  ACCELSPI.DATA = data;
  while(!(ACCELSPI.STATUS & SPI_IF_bm)) {
    dummy = ACCELSPI.DATA; } //wait for it to finish

  ACCEL_CS_PORT.OUTSET = ACCEL_CS_PIN_MASK; // chip select
  return;
}

/*
 * Configure ADXL345 Accelerometer for 16G Measurement Mode

 called once in main
 */
void configAccel(void)
{
    writeRegisterSPI(0x2D, 0x08); //switch from standby to measuring
    writeRegisterSPI(0x31, 0x0F); //set to fullscale, full resolution
}

/*
 * Clockout Six Databytes From ADXL345, Two Bytes Per Axis
 */
void readAxes(uint8_t *dataOut)
{
    //configureSPIModulesAccel();
    ACCEL_CS_PORT.OUTCLR = ACCEL_CS_PIN_MASK;

    ACCELSPI.DATA = (0x32 | 0xC0); //start at the x axis register, addr 0x32, set multibyte read with 0xC0
    while(!(ACCELSPI.STATUS & SPI_IF_bm)); //wait for it to finish

    for (uint8_t i = 0; i < 6; i++)
    {
        ACCELSPI.DATA = 0x00; //clockout
        while(!(ACCELSPI.STATUS & SPI_IF_bm)); //wait for clockout to finish
        dataOut[i] = ACCELSPI.DATA; //store
    }

    ACCEL_CS_PORT.OUTSET = ACCEL_CS_PIN_MASK;
    return;
}

