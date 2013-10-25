/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0.c
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Proximal/Distal Top Level Implementation

****************************************************/

/******************************************************************************
    Project Build Summary:
-------------------------------------------------------------------------------
Source File                                Cur. Rev. Level        Prev. Rev. Level
--------------------------------        ----------------    ----------------
C1482-SRC-PXDMCU-0.c                    00                    00


-------------------------------------------------------------------------------

    Project Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00          MM/DD/YYYY    ZAC        Initial Release
300         03/11/2013    BA         PE0 now NC
-------------------------------------------------------------------------------

******************************************************************************/

#include "distal.h"
#include "portio.h"
#include "../common/daisycomm.h"
#include "tactsense.h"
#include "adc.h"
#include "accel.h"
#include "encoder.h"
#include "../common/nvm.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#define FIRMWARE_VERSION 302

//int isProximal;
uint8_t numPressureSensors;

uint16_t testADC[8];

static void configureClocks(void);
static void WriteIntToEEPROM(uint8_t address, uint8_t* value);
static void ReadIntFromEEPROM(uint8_t address, uint8_t* destination);
static void verifyVersion(void);
static void initStateFromEEPROM(void);

/************************************************************************
* Private function definitions
************************************************************************/

/************************************************************************
* configureClocks()
*
* Configure system clocks
* The RTC is being used to generate PID ticks, so ensure that the 32 kHz oscillator
* is running.  This will feed a 1.024 kHz clock to the RTC.
* The system clock will use the onboard 32MHz oscillator
************************************************************************/
static void configureClocks(void)
{
    OSC.CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;            //enable the 32MHz ring oscillator and the 32kHz RTC oscillator
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));                //wait for 32MHz to stabilize
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));                //wait for 32kHz to stabilize

    DFLLRC32M.CTRL = DFLL_ENABLE_bm;
    DFLLRC2M.CTRL = DFLL_ENABLE_bm;

    CCP = CCP_IOREG_gc;                                    //clock is protected by Configuration Change Protection - must be properly disabled to change clock source
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;                    //switch to 32MHz oscillator
    CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;    //Select 1.024 kHz from internal oscillator for RTC clock source
}

static void detectProximalDistal(void)
{
    //isProximal = 0;
  numPressureSensors = 10;
}

static int handleCollectionCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint8_t responseSize = 0;
    uint16_t collectionBitfield;
    int16_t adcData[7];
    uint8_t accelData[6];

    //collectionBitfield = (uint16_t) (commandPacket[PAYLOAD_OFFSET]);
    memcpy(&collectionBitfield,&commandPacket[PAYLOAD_OFFSET],2);

    if (collectionBitfield & (DATA_COLLECTION_DISTALJOINT_BITMASK | DATA_COLLECTION_DYNAMIC_BITMASK))
    {
        captureSweep(adcData);
    }

    if(collectionBitfield & DATA_COLLECTION_ACCELERATION_BITMASK)
      {
        readAxes(accelData);
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize],&accelData,6);
        responseSize += 6;
      }

    if(collectionBitfield & DATA_COLLECTION_DYNAMIC_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize],&adcData[DYNAMIC_OFFSET],6);
        responseSize += 6;
    }

    if(collectionBitfield & DATA_COLLECTION_DISTALJOINT_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize],&adcData[DISTALJOINT_OFFSET],8);
        responseSize += 8;
    }

    // if(isProximal)
    // {
    //     if(collectionBitfield & DATA_COLLECTION_PROXIMALJOINT_BITMASK)
    //     {
    //         encoderData = readEncoder();
    //         memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize],&encoderData,6);
    //         responseSize += 2;
    //     }
    // }

    if(collectionBitfield & DATA_COLLECTION_TACTILE_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize], &pressureData, numPressureSensors*2);
        responseSize += numPressureSensors*2;
    }
    
    if(collectionBitfield & DATA_COLLECTION_DEBUG_BITMASK)
    {
        // memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize],&RxCheckSumErrCnt,2);
        // responseSize += 2;

        outputBuffer[RESPONSE_PAYLOAD_OFFSET+0] = nvm_fuses_read(FUSEBYTE0);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+1] = nvm_fuses_read(FUSEBYTE1);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+2] = nvm_fuses_read(FUSEBYTE2);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+3] = nvm_fuses_read(FUSEBYTE4);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+4] = nvm_fuses_read(FUSEBYTE5);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+5] = NVM_LOCKBITS;
        responseSize += 6;
    }
    
    if(collectionBitfield & DATA_COLLECTION_TACTILE_TEMP_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize], &pressureTempData, numPressureSensors*2);
        responseSize += numPressureSensors*2;
    }
    
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3+responseSize;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = DATA_COLLECTION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
    outputBuffer[3+responseSize] = computeChecksum(outputBuffer,3+responseSize); //this is the checksum
    //LEDoff();
    return 4+responseSize;
}

static void initStateFromEEPROM(void)
{
    uint32_t zero_check = 0;
    ReadIntFromEEPROM(EEPROM_ADDRESS_LED, (uint8_t*)&zero_check);
    
    if(zero_check)
        LEDon();
    else
        LEDoff();
}

static void WriteIntToEEPROM(uint8_t address, uint8_t* value)
{
    for(uint8_t i=0; i<4; i++)
        eeprom_write_byte((uint8_t *) (address*4+i), *(value+i));
}

static void ReadIntFromEEPROM(uint8_t address, uint8_t* destination)
{
    for(uint8_t i=0; i<4; i++)
        *(destination+i) = eeprom_read_byte((uint8_t *) (address*4+i));
}

static void verifyVersion()
{
    uint32_t readversion = 0;
    ReadIntFromEEPROM(EEPROM_ADDRESS_FIRMWARE_VERSION, (uint8_t*)&readversion);
    if (readversion != FIRMWARE_VERSION)
    {
        readversion = FIRMWARE_VERSION;
        WriteIntToEEPROM(EEPROM_ADDRESS_FIRMWARE_VERSION, (uint8_t*)&readversion);
    }
}

static int handleEEPROMCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint8_t opcode;
    uint8_t address;
    uint8_t responseSize = 4;

    opcode = commandPacket[COMMAND_OFFSET] & OPCODE_BITMASK;
    address = commandPacket[COMMAND_OFFSET] & 0x1F;

    switch(opcode)
    {
        case MOTOR_PARAMETER_RE_L_OPCODE:
        case MOTOR_PARAMETER_RE_H_OPCODE:
            // Read 4 bytes of data
            ReadIntFromEEPROM(address, &outputBuffer[RESPONSE_PAYLOAD_OFFSET]);
            // finish up the packet
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3+responseSize;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = commandPacket[COMMAND_OFFSET];
            outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
            outputBuffer[3+responseSize] = computeChecksum(outputBuffer,3+responseSize); //this is the checksum
            return 4+responseSize;
        case MOTOR_PARAMETER_WR_L_OPCODE:
        case MOTOR_PARAMETER_WR_H_OPCODE:
            if(address==EEPROM_ADDRESS_LED) {
                if(commandPacket[PAYLOAD_OFFSET+0]
                   |commandPacket[PAYLOAD_OFFSET+1]
                   |commandPacket[PAYLOAD_OFFSET+2]
                   |commandPacket[PAYLOAD_OFFSET+3]) {
                    LEDon();
                }
                else {
                    LEDoff();
                }
            }
            // Write 32 bits of data
            WriteIntToEEPROM(address, &commandPacket[PAYLOAD_OFFSET]);
            // ack the command
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = commandPacket[COMMAND_OFFSET];
            outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
            outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
            return 4;
        default:
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = commandPacket[COMMAND_OFFSET];
            outputBuffer[RESPONSE_STATUSCODE_OFFSET] = UNKNOWN_COMMAND_ERROR;
            outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
            return 4;
    }
}


static int handleCalibrationCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint16_t collectionBitfield;
    
    memcpy(&collectionBitfield, &commandPacket[PAYLOAD_OFFSET], 2);
    
    if(collectionBitfield & DATA_COLLECTION_TACTILE_BITMASK)
    {
        // zero out pressure readings with an offset
        for(uint8_t i=0; i<MAXIMUM_NUMBER_OF_TACTILE_SENSORS; i++)
        {
            pressureDataOffset[i] = pressureData[i] + pressureDataOffset[i];
        }
    }
    
    if (collectionBitfield & DATA_COLLECTION_DEBUG_BITMASK)
    {
        RxCheckSumErrCnt[0] = 0;
        RxCheckSumErrCnt[1] = 0;
    }
    
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = CALIBRATION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
    outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
    return 4;
}
static int handleBootloaderCommand(uint8_t *commandPacket)
{
  // self bootload
  wdt_enable(WDT_PER_8CLK_gc);
  while(1);  // use watchdog timer to trigger a reset

  return 5; // never get here, but keep compiler happy
}


int processCommand(uint8_t *commandPacket,uint8_t *outputBuffer)
{
  uint8_t opcode;

  //LEDtoggle();

  opcode = commandPacket[COMMAND_OFFSET] & OPCODE_BITMASK;

  switch(opcode)
  {
      case DATA_COLLECTION_OPCODE:
          return handleCollectionCommand(commandPacket,outputBuffer);

      case MOTOR_PARAMETER_RE_L_OPCODE:
      case MOTOR_PARAMETER_RE_H_OPCODE:
      case MOTOR_PARAMETER_WR_L_OPCODE:
      case MOTOR_PARAMETER_WR_H_OPCODE:
          return handleEEPROMCommand(commandPacket,outputBuffer);

      case CALIBRATION_OPCODE:
          return handleCalibrationCommand(commandPacket, outputBuffer);

      case BOOTLOADER_OPCODE:
	  return handleBootloaderCommand(commandPacket);

      default:
          outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
          outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = opcode;
          outputBuffer[RESPONSE_STATUSCODE_OFFSET] = UNKNOWN_COMMAND_ERROR;
          outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
          return 4;
  }
}

void LEDon(void)
{
    //PORTC.DIRSET = 0x01;
    PORTC.OUTSET = 0x01;
}

void LEDoff(void)
{
    //PORTC.DIRSET = 0x01;
    PORTC.OUTCLR = 0x01;
}

void LEDtoggle(void)
{
    //PORTC.DIRSET = 0x01;
    PORTC.OUTTGL = 0x01;
}

/************************************************************************
* MAIN function.
*
* This initializes the other port modules and polls a few volatile status
* bits set in ISRs.
************************************************************************/
int main(void)
{
    cli();                                    //disable all interrupts for clock reset
    _delay_ms(1); // for stability of supplies
    configureClocks();
    _delay_ms(1); // for stability of clocks
    detectProximalDistal();
    configurePortIODistal();

    LEDon();

    configureADC();

    configureDaisyUSART();

    initTactileModule(); // initialize variables of the tactile buffer

    PMIC.CTRL |= PMIC_LOLVLEN_bm;            //tell event system to pay attention to low-priority interrupts

    nvm_blbb_lock_bits_write(NVM_BLBB_WLOCK_gc); // lock bootloader from being written
    
    sei();

    configureSPIModulesPressure();
    collectAllCalibrationValues();

    configureSPIModulesAccel();
    configAccel();

    LEDoff();

    initStateFromEEPROM();
    verifyVersion();
    
    while(1){
        
        if(notifyDaisy)
        {
            doDaisyTask();
        }

        if(tactReady)
        {
            doTactSensors();
        }
    }
    return 0;
}
