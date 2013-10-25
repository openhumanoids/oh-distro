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
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/
#define F_CPU 32000000
#include <util/delay.h>
#include "daisyconfig.h"
#define COMMAND_PACKET_SIZE 7


#include "proximal.h"
#include "portio.h"
#include "../common/daisycomm.h"
#include "tactsense.h"
#include "adc.h"
#include "encoder.h"
#include "accel.h"
#include "../common/nvm.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#define FIRMWARE_VERSION 302

uint8_t numPressureSensors = 12;

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

static int handleCollectionCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint8_t responseSize = 0;
    uint16_t collectionBitfield;
    int16_t adcData[7];
    uint8_t accelData[6];
    int16_t encoderData;

    memcpy(&collectionBitfield, &commandPacket[PAYLOAD_OFFSET], 2);

    if (collectionBitfield & (DATA_COLLECTION_DISTALJOINT_BITMASK | DATA_COLLECTION_DYNAMIC_BITMASK))
    {
        captureSweep(adcData);
    }

    if(collectionBitfield & DATA_COLLECTION_ACCELERATION_BITMASK)
    {
        readAxes(accelData);
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize], &accelData, 6);
        responseSize += 6;
    }

    if(collectionBitfield & DATA_COLLECTION_DYNAMIC_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize], &adcData[DYNAMIC_OFFSET], 6);
        responseSize += 6;
    }

    if(collectionBitfield & DATA_COLLECTION_DISTALJOINT_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize], &adcData[DISTALJOINT_OFFSET], 8);
        responseSize += 8;
    }

    if(collectionBitfield & DATA_COLLECTION_PROXIMALJOINT_BITMASK)
    {
        encoderData = readEncoder();
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize], &encoderData, 2);
        responseSize += 2;
    }

    if(collectionBitfield & DATA_COLLECTION_TACTILE_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize], &pressureData, numPressureSensors*2);
        responseSize += numPressureSensors*2;
    }

    if(collectionBitfield & DATA_COLLECTION_DEBUG_BITMASK)
    {
        // memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize], &RxCheckSumErrCnt, 4);
        // responseSize += 4;

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
    return 4+responseSize;
}

static void initStateFromEEPROM(void)
{
    uint32_t temp = 0;
    
    ReadIntFromEEPROM(EEPROM_ADDRESS_LED, (uint8_t*)&temp);
    if(temp)
        LEDon();
    else
        LEDoff();
    
    ReadIntFromEEPROM(EEPROM_ADDRESS_ENCODER_OFFSET, (uint8_t*)&temp);
    encoderOffset = temp;
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

static void verifyVersion(void)
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

            // Write 32 bits of data
            WriteIntToEEPROM(address, &commandPacket[PAYLOAD_OFFSET]);
            
            // re-init our state
            initStateFromEEPROM();
            
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
    
    if (collectionBitfield & DATA_COLLECTION_PROXIMALJOINT_BITMASK)
    {
        encoderOffset = readRawEncoder();
        uint32_t temp = encoderOffset;
        WriteIntToEEPROM(EEPROM_ADDRESS_ENCODER_OFFSET, (uint8_t*)&temp);
    }
    
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = CALIBRATION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
    outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
    return 4;
}

void BootRelay(uint8_t *commandPacket)
{
  // overview: 1)send a bootload command to the target
  // overview: 2)configure for 115200 baud passthrough
  // overview: 3)start a 20 second timer
  // overview: 4)do the serial passthrough, blink LED
  // overview: 5)wait for timer to finish, then reset/reboot

  cli(); // disable all interrupts, nothing else going on
  DMA.CTRL = 0; // disable the DMA controller
  
  // repeat the command to the target
  DOWNSTREAM_USART.CTRLA = 0x00; // disable interrupts
  DOWNSTREAM_USART.CTRLB |= USART_RXEN_bm | USART_TXEN_bm; //enable RX and TX
  for(int i=0;i<COMMAND_PACKET_SIZE;i++)
    {
      while(!(DOWNSTREAM_USART.STATUS & USART_DREIF_bm));
      DOWNSTREAM_USART.DATA = commandPacket[i];
    }
  // dont bother waiting for a response verification
  while(!(DOWNSTREAM_USART.STATUS & USART_DREIF_bm)); // wait for the data to be read
  DOWNSTREAM_USART.STATUS = USART_TXCIF_bm; // clear the TX done flag
  while(!(DOWNSTREAM_USART.STATUS & USART_TXCIF_bm)); // wait for the TX to be done
  
  // Reconfigure the serial ports
  //int bsel = 1047;       // 115200 @ 32Mhz as calculated from ProtoTalk.net
  //uint8_t bscale = 10;      // 115200 @ 32Mhz as calculated from ProtoTalk.net
  int bsel = 3269;       // 38,400 @ 32Mhz as calculated from ProtoTalk.net
  uint8_t bscale = -6;      // 38,400 @ 32Mhz as calculated from ProtoTalk.net
  UPSTREAM_USART.CTRLA = 0x00; // disable interrupts
  UPSTREAM_USART.BAUDCTRLA = (uint8_t) bsel;
  UPSTREAM_USART.BAUDCTRLB = (bscale << 4) | (bsel >> 8);
  UPSTREAM_USART.CTRLB = USART_RXEN_bm | USART_TXEN_bm;    //enable RX and TX
  DOWNSTREAM_USART.CTRLA = 0x00; // disable interrupts
  DOWNSTREAM_USART.BAUDCTRLA = (uint8_t) bsel;
  DOWNSTREAM_USART.BAUDCTRLB = (bscale << 4) | (bsel >> 8);
  DOWNSTREAM_USART.CTRLB = USART_RXEN_bm | USART_TXEN_bm;    //enable RX and TX
  
  int sec_cnt=0;
  uint32_t loop_cnt=0;
  uint8_t up_fifo[16];
  uint8_t up_head = 0;
  uint8_t up_tail = 0;
  uint8_t down_fifo[16];
  uint8_t down_head = 0;
  uint8_t down_tail = 0;

  // relay chars while waiting for timeout
  while(sec_cnt < 200) 
  {
      loop_cnt++;
      if(loop_cnt > 90000L) 
      { // approximately 0.1sec experimentally
          loop_cnt=0;
          sec_cnt++;
          if((sec_cnt&0x03)==0x03)
              LEDon();
          else
              LEDoff();
      }
      if (DOWNSTREAM_USART.STATUS & USART_RXCIF_bm) {
          up_fifo[up_head++] = DOWNSTREAM_USART.DATA;
          up_head %= sizeof(up_fifo);
      }
      if (UPSTREAM_USART.STATUS & USART_RXCIF_bm) {
          down_fifo[down_head++] = UPSTREAM_USART.DATA;
          down_head %= sizeof(down_fifo);
      }
      if (up_head != up_tail && (UPSTREAM_USART.STATUS & USART_DREIF_bm)) {
          UPSTREAM_USART.DATA = up_fifo[up_tail++];
          up_tail %= sizeof(up_fifo);
      }
      if (down_head != down_tail && (DOWNSTREAM_USART.STATUS & USART_DREIF_bm)) {
          DOWNSTREAM_USART.DATA = down_fifo[down_tail++];
          down_tail %= sizeof(down_fifo);
      }
  }

  // 30sec timer finished, reset/reboot
  wdt_enable(WDT_PER_8CLK_gc);
  while(1);  // use watchdog timer to trigger a reset
}

static int handleBootloaderCommand(uint8_t *commandPacket)
{

  if(0x0f & commandPacket[PAYLOAD_OFFSET]){
    BootRelay(commandPacket);
  }
  else {
    // self bootload
    wdt_enable(WDT_PER_8CLK_gc);
    while(1);  // use watchdog timer to trigger a reset
  }
  return 5; // never get here, but keep compiler happy
}

int processCommand(uint8_t *commandPacket,uint8_t *outputBuffer)
{
  uint8_t opcode;

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
    PORTC.OUT &= ~0x01;
}

void LEDoff(void)
{
    PORTC.OUT |= 0x01;
}

void LEDtoggle(void)
{
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
    configurePortIOProximal();

    LEDon();

    //PORTE.OUT &= ~0x01; //BA: turn on sensor board

    configureADC();

    configureDaisyUSART();

    initTactileModule(); // initialize variables of the tactile buffer

    PMIC.CTRL |= PMIC_LOLVLEN_bm; //tell event system to pay attention to low-priority interrupts
    
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
        handleTC();

        if(tactReady)
        {
            doTactSensors();
        }
        
    }
    return 0;
}
