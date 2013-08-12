/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-TACMCU-0_tactsense.c
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Tactile sensor driver

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

#include "tactile.h"
#include "tactsense.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define SPI0 SPIC
#define SPI1 SPID

#define TACT_ADDR_MASK 0x07
#define TACT_CS_MASK 0x18
#define TACT_CS1_CS4 0x00
#define TACT_CS2_CS5 0x08
#define TACT_CS3_CS6 0x10

#define TACT_TIMEOUT 96

#define CONVERT_OPCODE 0x24
#define READ_PRESSURE_MSB_OPCODE 0x80
#define READ_PRESSURE_LSB_OPCODE 0x82
#define READ_TEMPERATURE_MSB_OPCODE 0x84
#define READ_TEMPERATURE_LSB_OPCODE 0x86
#define READ_COEFFA0_MSB_OPCODE 0x88
#define READ_COEFFA0_LSB_OPCODE 0x8A
#define READ_COEFFB1_MSB_OPCODE 0x8C
#define READ_COEFFB1_LSB_OPCODE 0x8E
#define READ_COEFFB2_MSB_OPCODE 0x90
#define READ_COEFFB2_LSB_OPCODE 0x92
#define READ_COEFFC12_MSB_OPCODE 0x94
#define READ_COEFFC12_LSB_OPCODE 0x96


int16_t pressureData[NUMBER_OF_TACTILE_SENSORS];
int16_t pressureDataOffset[NUMBER_OF_TACTILE_SENSORS];
int16_t pressureTempData[NUMBER_OF_TACTILE_SENSORS];

// int16_t a0coeff[NUMBER_OF_TACTILE_SENSORS];
// int16_t b1coeff[NUMBER_OF_TACTILE_SENSORS];
// int16_t b2coeff[NUMBER_OF_TACTILE_SENSORS];
// int16_t c12coeff[NUMBER_OF_TACTILE_SENSORS];

//float floatingPressureData[NUMBER_OF_TACTILE_SENSORS];
// uint16_t rawpressure[NUMBER_OF_TACTILE_SENSORS];
// uint16_t rawtemp[NUMBER_OF_TACTILE_SENSORS];

uint8_t pressureCalibrated[NUMBER_OF_TACTILE_SENSORS];
uint16_t lastPressure[NUMBER_OF_TACTILE_SENSORS];
int16_t adjustedPressure[NUMBER_OF_TACTILE_SENSORS];

void compensatePressure(uint16_t tempReading, uint16_t pressureReading,uint8_t sensorIndex);
static void convertTactSensorPair(uint8_t tactAddress);
static void readTactSensorPair(uint8_t tactAddress, int16_t *firstSensor, int16_t *secondSensor);
static void selectSensorPair(uint8_t tactAddress);

volatile uint8_t tactReady = 0;

ISR(TACT_TC_vect)
{
    tactReady = 1;
    TACT_TC.CTRLA = TC_CLKSEL_OFF_gc;
    return;
}

//The system can take 8 MHz.  Peripheral clock is set to 32 MHz internally, so a divide by 4 prescaler is correct

void configureSPIModules(void)
{
    SPI0.INTCTRL = SPI_INTLVL_OFF_gc;
    SPI0.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc;

    SPI1.INTCTRL = SPI_INTLVL_OFF_gc;
    SPI1.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc;

}

//initialization
void initTactileModule(void)
{
    for(uint8_t i=0; i<NUMBER_OF_TACTILE_SENSORS; i++)
    {
        pressureCalibrated[i] = 0;
        pressureData[i] = 0;
        pressureDataOffset[i] = 0;
        pressureTempData[i] = 0;
        // a0coeff[i] = 0;
        // b1coeff[i] = 0;
        // b2coeff[i] = 0;
        // c12coeff[i] = 0;
        // rawpressure[i] = 0;
        // rawtemp[i] = 0;
        lastPressure[i] = 0;
        adjustedPressure[i] = 0;
    }
}

static void getDoubleWord(uint8_t startingOpcode, uint16_t *firstResponse, uint16_t *secondResponse)
{
    uint8_t dummyRead;

    //Send the data.
    SPI0.DATA = startingOpcode;
    SPI1.DATA = startingOpcode;

    while(!(SPI0.STATUS & SPI_IF_bm));
    while(!(SPI1.STATUS & SPI_IF_bm));

    //Read the dummy data
    dummyRead = SPI0.DATA;
    dummyRead = SPI1.DATA;

    //Send the dummy byte
    SPI0.DATA = 0x00;
    SPI1.DATA = 0x00;

    while(!(SPI0.STATUS & SPI_IF_bm));
    while(!(SPI1.STATUS & SPI_IF_bm));

    //Read the MSB data
    dummyRead = SPI0.DATA;
    *firstResponse = dummyRead << 8;
    dummyRead = SPI1.DATA;
    *secondResponse = dummyRead << 8;

    //Send the data.
    SPI0.DATA = startingOpcode+2;
    SPI1.DATA = startingOpcode+2;

    while(!(SPI0.STATUS & SPI_IF_bm));
    while(!(SPI1.STATUS & SPI_IF_bm));

    //Read the dummy data
    dummyRead = SPI0.DATA;
    dummyRead = SPI1.DATA;

    //Send the dummy byte
    SPI0.DATA = 0x00;
    SPI1.DATA = 0x00;

    while(!(SPI0.STATUS & SPI_IF_bm));
    while(!(SPI1.STATUS & SPI_IF_bm));

    //Read the LSB data
    dummyRead = SPI0.DATA;
    *firstResponse |= dummyRead;
    dummyRead = SPI1.DATA;
    *secondResponse |= dummyRead;

    return;
}

/**
 * readTactSensorPair(uint8_t tactAddress)
 * Reads a tactile sensor pair given by tactAddress.
 * It assumes that the pair had been previously told to begin a conversion
 * the lower three bits of tactAddress denote the Address line status to be used.  The next two bits
 * set the proper chip selects to be used according to the following table:
 * 00 - nCS1 and nCS4
 * 01 - nCS2 and nCS5
 * 10 - nCS3 and nCS6
 * 11 - Invalid
 *
 * The remaining bits are don't cares.  This means that the addressing is all adjacent and that a reader can
 * feed incrementing addresses up to decimal 24 to read all 48 tactile sensors.
 */
static void readTactSensorPair(uint8_t tactAddress, int16_t *firstSensor, int16_t *secondSensor)
{
    uint8_t dummyRead;
    uint16_t firstPressureReading,secondPressureReading;
    uint16_t firstTempReading,secondTempReading;

    selectSensorPair(tactAddress);

    getDoubleWord(READ_PRESSURE_MSB_OPCODE,&firstPressureReading,&secondPressureReading);

    getDoubleWord(READ_TEMPERATURE_MSB_OPCODE,&firstTempReading,&secondTempReading);

    //Do a final dummy write

    //Send the data.
    SPI0.DATA = 0x00;
    SPI1.DATA = 0x00;

    while(!(SPI0.STATUS & SPI_IF_bm));
    while(!(SPI1.STATUS & SPI_IF_bm));

    //Read the dummy data
    dummyRead = SPI0.DATA;
    dummyRead = SPI1.DATA;

    //Reset the chip select and address lines
    PORTE.OUTSET = 0xFF;
    PORTD.OUTSET = 0x18;
    PORTD.OUTCLR = TACT_ADDR_MASK;

    // protect against bad data if reading accelerometer
    if (firstPressureReading != 0)
    {
        //floatingPressureData[tactAddress] = compensatePressure(firstTempReading,firstPressureReading,tactAddress);
        compensatePressure(firstTempReading, firstPressureReading, tactAddress);
        //rawtemp[tactAddress] = firstTempReading;
        //rawpressure[tactAddress] = firstPressureReading;
    }

    // protect against bad data if reading accelerometer
    if (secondPressureReading != 0)
    {
        //floatingPressureData[tactAddress+24] = compensatePressure(secondTempReading,secondPressureReading,tactAddress+24);
        compensatePressure(secondTempReading, secondPressureReading, tactAddress+24);
        //rawtemp[tactAddress+24] = secondTempReading;
        //rawpressure[tactAddress+24] = secondPressureReading;
    }

    return;
}

/**
 * convertTactSensorPair(uint8_t tactAddress)
 * Reads a tactile sensor pair given by tactAddress.
 * It assumes that the pair had been previously told to begin a conversion
 * the lower three bits of tactAddress denote the Address line status to be used.  The next two bits
 * set the proper chip selects to be used according to the following table:
 * 00 - nCS1 and nCS4
 * 01 - nCS2 and nCS5
 * 10 - nCS3 and nCS6
 * 11 - Invalid
 *
 * The remaining bits are don't cares.  This means that the addressing is all adjacent and that a reader can
 * feed incrementing addresses up to decimal 24 to read all 48 tactile sensors.
 */

//For simplicity, the Tactile sensor address pins are all on PD0 - PD2
static void selectSensorPair(uint8_t tactAddress)
{
    uint8_t dummyRead;
    //First set the address properly
    PORTD.OUTSET = tactAddress & TACT_ADDR_MASK;

    //Now assert the proper chip selects
    switch(tactAddress & TACT_CS_MASK)
    {
        case TACT_CS1_CS4:
            PORTD.OUTCLR = 0x08;
            PORTE.OUTCLR = 0x02;
            break;
        case TACT_CS2_CS5:
            PORTD.OUTCLR = 0x10;
            PORTE.OUTCLR = 0x04;
            break;
        case TACT_CS3_CS6:
            PORTE.OUTCLR = 0x09;
            break;
        default:
            //Should never get here.  Invalid address
            //Reset address lines and get out
            PORTD.OUTCLR = TACT_ADDR_MASK;
            return;
    }

    //Make sure transmitters are OK
    if(SPI0.STATUS & SPI_IF_bm)
    {
        dummyRead = SPI0.DATA;
    }

    if(SPI1.STATUS & SPI_IF_bm)
    {
        dummyRead = SPI1.DATA;
    }

    return;
}

static void convertTactSensorPair(uint8_t tactAddress)
{
    uint8_t dummyRead;
    selectSensorPair(tactAddress);

    //Send the data.
    SPI0.DATA = CONVERT_OPCODE;
    SPI1.DATA = CONVERT_OPCODE;

    while(!(SPI0.STATUS & SPI_IF_bm));
    while(!(SPI1.STATUS & SPI_IF_bm));

    //Read the dummy data
    dummyRead = SPI0.DATA;
    dummyRead = SPI1.DATA;

    //Send the dummy byte
    SPI0.DATA = 0x00;
    SPI1.DATA = 0x00;

    while(!(SPI0.STATUS & SPI_IF_bm));
    while(!(SPI1.STATUS & SPI_IF_bm));

    //Read the dummy data
    dummyRead = SPI0.DATA;
    dummyRead = SPI1.DATA;

    //Reset the chip select and address lines
    PORTE.OUTSET = 0xFF;
    PORTD.OUTSET = 0x18;
    PORTD.OUTCLR = TACT_ADDR_MASK;

    return;
}

uint8_t activeSensor = 0;

void doTactSensors(void)
{
    uint8_t i;

    //Set all chip selects high
    PORTE.OUTSET = 0xFF;
    PORTD.OUTSET = 0x18;

    //Lower all address lines
    PORTD.OUTCLR = TACT_ADDR_MASK;

    if(activeSensor == 24)
    {
        //Set the Tactile Timer to expire at 3 ms and begin conversion
        cli();
        tactReady = 0;
        sei();
        for(i=0;i<24;i++)
        {
            convertTactSensorPair(i);
        }

        TACT_TC.CTRLB = TC_WGMODE_NORMAL_gc;
        TACT_TC.CTRLC = 0x00;
        TACT_TC.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
        TACT_TC.CTRLE = 0x00;
        TACT_TC.PERBUF = TACT_TIMEOUT;
        TACT_TC.CNT = 0x0000;
        TACT_TC.INTCTRLA = TC_OVFINTLVL_LO_gc;
        TACT_TC.CTRLA = TC_CLKSEL_DIV1024_gc;
        activeSensor = 0;
        return;
    }

    //readTactSensor(activeSensor, &pressureData[activeSensor]);
    readTactSensorPair(activeSensor, &pressureData[activeSensor], &pressureData[activeSensor+24]);
    activeSensor++;
    //Leave tactReady asserted so that we can quickly return after giving command a chance
}

/************************************************************************
 * collectAllTactSensors(void)
 * Blocking call to read all tactile sensors right now.
 ***********************************************************************
void collectAllTactSensors(void)
{
    int i=0;
    //First set the pins to known values to tighten inner reading loops

    //Set all chip selects high
    PORTE.OUTSET = 0xFF;
    PORTD.OUTSET = 0x18;

    //Lower all address lines
    PORTD.OUTCLR = TACT_ADDR_MASK;

    for(i=0;i<24;i++)
    {
        convertTactSensorPair(i);
    }

    //Each sensor takes two bytes at 8MHz, so the delay per sensor is 2 usec.
    //There are 24 groups receiving the conversion signals, so there are at a minimum
    //48 usecs of delay.  Measurement in the system will confirm how fast this really is.
    //For now, assume that only 50 usecs have passed since the first conversion start signal was sent.
    //
    //The sensors take 3 ms to convert.  This means that the system must sleep for 2.95 ms.
    _delay_ms(2.95);

    //Now read the pressure sensors.
    for(i=0;i<24;i++)
    {
        readTactSensorPair(i, &pressureData[i], &pressureData[i+24]);
    }

    return;

}
*/


// static void readCoefficientPair(uint8_t tactAddress)
// {
//     uint8_t dummyRead;

//     selectSensorPair(tactAddress);
//     getDoubleWord(READ_COEFFA0_MSB_OPCODE,(uint16_t *)&a0coeff[tactAddress],(uint16_t *)&a0coeff[tactAddress+24]);
//     getDoubleWord(READ_COEFFB1_MSB_OPCODE,(uint16_t *)&b1coeff[tactAddress],(uint16_t *)&b1coeff[tactAddress+24]);
//     getDoubleWord(READ_COEFFB2_MSB_OPCODE,(uint16_t *)&b2coeff[tactAddress],(uint16_t *)&b2coeff[tactAddress+24]);
//     getDoubleWord(READ_COEFFA0_MSB_OPCODE,(uint16_t *)&c12coeff[tactAddress],(uint16_t *)&c12coeff[tactAddress+24]);

//     //Send the dummy byte
//     SPI0.DATA = 0x00;
//     SPI1.DATA = 0x00;

//     while(!(SPI0.STATUS & SPI_IF_bm));
//     while(!(SPI1.STATUS & SPI_IF_bm));

//     //Read the dummy data
//     dummyRead = SPI0.DATA;
//     dummyRead = SPI1.DATA;

//     //Reset the chip select and address lines
//     PORTE.OUTSET = 0xFF;
//     PORTD.OUTSET = 0x18;
//     PORTD.OUTCLR = TACT_ADDR_MASK;

//     return;
// }

void collectAllCalibrationValues(void)
{
    int i=0;
    //First set the pins to known values to tighten inner reading loops

    //Set all chip selects high
    PORTE.OUTSET = 0xFF;
    PORTD.OUTSET = 0x18;

    //Lower all address lines
    PORTD.OUTCLR = TACT_ADDR_MASK;

    // for(i=0;i<24;i++)
    // {
    //     readCoefficientPair(i);
    // }

    //Set the Tactile Timer to expire at 3 ms
    TACT_TC.CTRLB = TC_WGMODE_NORMAL_gc;
    TACT_TC.CTRLC = 0x00;
    TACT_TC.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
    TACT_TC.CTRLE = 0x00;
    TACT_TC.PERBUF = TACT_TIMEOUT;
    TACT_TC.CNT = 0x0000;
    TACT_TC.INTCTRLA = TC_OVFINTLVL_LO_gc;
    TACT_TC.CTRLA = TC_CLKSEL_DIV1024_gc;

    return;
}

// typedef int16_t S16;
// typedef int32_t S32;

/************************************************************************
 * compensatePressure()
 *
 * This routine was pulled from Freescale app note AN3785 and modified
 * for this microprocessor.  The original code assumed two extra parameters
 * that have now been removed.
 ************************************************************************/
void compensatePressure(uint16_t tempReading, uint16_t pressureReading, uint8_t sensorIndex)
{
    //===================================================
    //Coefficient 9 equation compensation
    //===================================================
    //
    //Variable sizes:
    //For placing high and low bytes of the Memory addresses for each of the 6 coefficients:
    //signed char (S8) sia0MSB, sia0LSB, sib1MSB,sib1LSB, sib2MSB,sib2LSB, sic12MSB,sic12LSB, sic11MSB,sic11LSB, sic22MSB,sic22LSB;
    //
    //Variable for use in the compensation, this is the 6 coefficients in 16bit form, MSB+LSB.
    //signed int (S16) sia0, sib1, sib2, sic12, sic11, sic22;
    //
    //Variable used to do large calculation as 3 temp variables in the process below
    //signed long (S32) lt1, lt2, lt3;
    //
    //Variables used for Pressure and Temperature Raw.
    //unsigned int (U16) uiPadc, uiTadc.
    //signed (N=number of bits in coefficient, F-fractional bits)
    //s(N,F)
    //The below Pressure and Temp or uiPadc and uiTadc are shifted from the MSB+LSB values to remove the zeros in the LSB since this
    // 10bit number is stored in 16 bits. i.e 0123456789XXXXXX becomes 0000000123456789

    // int32_t lt1,lt2,lt3;
    uint16_t pressureLocal;
    uint16_t tempLocal;

    // //#define S32 uint32_t
    // //#define S16 int16_t

    // int32_t si_a2x2,si_y1,si_a1x1,si_c12x2,si_a1;

    // int16_t siPcomp;
    // //float decPcomp;

    pressureLocal=pressureReading>>6; //Note that the PressCntdec is the raw value from the MPL115A data address. Its shifted >>6 since its 10 bit.
    tempLocal=tempReading>>6; //Note that the TempCntdec is the raw value from the MPL115A data address. Its shifted >>6 since its 10 bit.

    // since we are operating the sensor outside its normal range, it seems to behave as if it has 11 bytes of data.
    // however, its range is still 10 bits, (0-1023).  So we need to track the "actual" value of the sensor across multiple overflows.
    {
        if (!pressureCalibrated[sensorIndex])
        {
            lastPressure[sensorIndex] = pressureLocal;
            adjustedPressure[sensorIndex] = pressureLocal;
            pressureCalibrated[sensorIndex] = 1;
        }

        int16_t delta = pressureLocal - lastPressure[sensorIndex];

        if (abs(delta) < 512)
        {
            lastPressure[sensorIndex] = pressureLocal;
            adjustedPressure[sensorIndex] = adjustedPressure[sensorIndex] + delta;
        }
        else
        {
            int16_t bottom = lastPressure[sensorIndex] + (1024 - pressureLocal);
            int16_t top = pressureLocal + (1024 - lastPressure[sensorIndex]);
            lastPressure[sensorIndex] = pressureLocal;
            if (bottom<top) // crossing 0
                adjustedPressure[sensorIndex] = adjustedPressure[sensorIndex] - bottom;
            else // crossing 1024
                adjustedPressure[sensorIndex] = adjustedPressure[sensorIndex] + top;
        }
    }

    // /*
    // // ******* STEP 1 c11x1= c11 * Padc
    // lt1 = (S32)sic11; // s(16,27) s(N,F+zeropad) goes from s(11,10)+11ZeroPad = s(11,22) => Left Justified = s(16,27)
    // lt2 = (S32)pressureLocal; // u(10,0) s(N,F)
    // lt3 = lt1 * lt2; // s(26,27) /c11*Padc
    // si_c11x1 = (S32)(lt3); // s(26,27)- EQ 1 =c11x1 /checked
    // //divide this hex number by 2^30 to get the correct decimal value.
    // //b1 =s(14,11) => s(16,13) Left justified
    // */

    // //si_c11x1 is zero

    // /*
    // // ******* STEP 2 a11= b1 + c11x1
    // lt1 = ((S32)b1coeff[sensorIndex]<<14); // s(30,27) b1=s(16,13) Shift b1 so that the F matches c11x1(shift by 14)
    // lt2 = (S32)si_c11x1; // s(26,27) //ensure fractional bits are compatible
    // lt3 = lt1 + lt2; // s(30,27) /b1+c11x1
    // si_a11 = (S32)(lt3>>14); // s(16,13) - EQ 2 =a11 Convert this block back to s(16,X)
    // */

    // //si_a11 is just b1

    // // ******* STEP 3 c12x2= c12 * Tadc
    // // sic12 is s(14,13)+9zero pad = s(16,15)+9 => s(16,24) left justified
    // lt1 = (S32)c12coeff[sensorIndex]; // s(16,24)
    // lt2 = (S32)tempLocal; // u(10,0)
    // lt3 = lt1 * lt2; // s(26,24)
    // si_c12x2 = (S32)(lt3); // s(26,24) - EQ 3 =c12x2 /checked


    // //Changed si_a11 to b1coeff

    // // ******* STEP 4 a1= a11 + c12x2
    // lt1 = ((S32)b1coeff[sensorIndex]<<11); // s(27,24) This is done by s(16,13) <<11 goes to s(27,24) to match c12x2's F part
    // lt2 = (S32)si_c12x2; // s(26,24)
    // lt3 = lt1 + lt2; // s(27,24) /a11+c12x2
    // si_a1 =(S32)(lt3>>11); // s(16,13) - EQ 4 =a1 /check

    // /*
    // // ******* STEP 5 c22x2= c22 * Tadc
    // // c22 is s(11,10)+9zero pad = s(11,19) => s(16,24) left justified
    // lt1 = (S32)sic22; // s(16,30) This is done by s(11,10) + 15 zero pad goes to s(16,15)+15, to s(16,30)
    // lt2 = (S32)tempLocal; // u(10,0)
    // lt3 = lt1 * lt2; // s(26,30) /c22*Tadc
    // si_c22x2 = (S32)(lt3); // s(26,30) - EQ 5 /=c22x2
    // */

    // //si_c22x2 is zero

    // /*
    // // ******* STEP 6 a2= b2 + c22x2
    // //WORKS and loses the least in data. One extra execution. Note how the 31 is really a 32 due to possible overflow.
    // // b2 is s(16,14) User shifted left to => s(31,29) to match c22x2 F value
    // lt1 = ((S32)b2coeff[sensorIndex]<<15); //s(31,29)
    // lt2 = ((S32)si_c22x2>>1); //s(25,29) s(26,30) goes to >>16 s(10,14) to match F from sib2
    // lt3 = lt1+lt2; //s(32,29) but really is a s(31,29) due to overflow the 31 becomes a 32.
    // si_a2 = ((S32)lt3>>16); //s(16,13)
    // */

    // //si_a2 is b2coeff shifted right by 1 bit

    // // ******* STEP 7 a1x1= a1 * Padc
    // lt1 = (S32)si_a1; // s(16,13)
    // lt2 = (S32)adjustedPressure[sensorIndex];  //lt2 = (S32)pressureLocal; // u(10,0)
    // lt3 = lt1 * lt2; // s(26,13) /a1*Padc
    // si_a1x1 = (S32)(lt3); // s(26,13) - EQ 7 /=a1x1 /check

    // // ******* STEP 8 y1= a0 + a1x1
    // // a0 = s(16,3)
    // lt1 = ((S32)a0coeff[sensorIndex]<<10); // s(26,13) This is done since has to match a1x1 F value to add. So S(16,3) <<10 = S(26,13)
    // lt2 = (S32)si_a1x1; // s(26,13)
    // lt3 = lt1 + lt2; // s(26,13) /a0+a1x1
    // si_y1 = (S32)(lt3>>10); // s(16,3) - EQ 8 /=y1 /check

    // //changed si_a2 to b2coeff.
    // //no other changes here

    // // ******* STEP 9 a2x2= a2 *Tadc
    // lt1 = (S32)b2coeff[sensorIndex]; // s(16,13)
    // lt2 = (S32)tempLocal; // u(10,0)
    // lt3 = lt1 * lt2; // s(26,13) /a2*Tadc
    // si_a2x2 = (S32)(lt3); // s(26,13) - EQ 9 /=a2x2

    // //Since I didn't shift b2coeff right by 1 above, si_y1 needs to be shifted left by 11 instead of 10 to match si_a2x2

    // // ******* STEP 10 pComp = y1 +a2x2
    // // y1= s(16,3)
    // lt1 = ((S32)si_y1<<11); // s(26,13) This is done to match a2x2 F value so addition can match. s(16,3) <<10
    // lt2 = (S32)si_a2x2; // s(26,13)
    // lt3 = lt1 + lt2; // s(26,13) /y1+a2x2

    // //The above addition has an extra F bit, so round down by 14

    // // FIXED POINT RESULT WITH ROUNDING:
    // siPcomp = (int16_t)(lt3>>14); //&0x3FF); // goes to no fractional parts since this is an ADC count.
    // //decPcomp is defined as a floating point number.
    // //Conversion to Decimal value from 1023 ADC count value. ADC counts are 0 to 1023. Pressure is 50 to 115kPa correspondingly.
    // //decPcomp = ((65.0/1023.0)*(float)siPcomp)+50.0;
    
    pressureTempData[sensorIndex] = tempLocal;
    //pressureData[sensorIndex] = siPcomp - pressureDataOffset[sensorIndex];
    pressureData[sensorIndex] = adjustedPressure[sensorIndex] - pressureDataOffset[sensorIndex];
    return;
}

