/****************************************************
 // Hardware:        ATxmega128A1
 // Description:    ARM-H Palm Board Top Level Implementation
 // Mostly a communications router to the other micros
******************************************************************************/

#include "palm.h"
#include "portio.h"
#include "router.h"
#include "adc.h"
#include "routerconfig.h"
#include "dac.h"
#include "encoder.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define FIRMWARE_VERSION 301

volatile uint8_t freerun_flag = 0;

SPREAD_CONTROL_MODE_t spreadMotorMode = SPREAD_MODE_NONE;
int16_t targetEncoder = 0;
int16_t spreadDeadband = 0;
int16_t spreadP = 0;

/************************************************************************
* Private function definitions
************************************************************************/
static void configureClocks(void);
static int handleFingerCommand(uint8_t *commandPacket, uint8_t *outputBuffer);
static int handleSetSamplePeriodCommand(uint8_t *commandPacket, uint8_t *outputBuffer);
static int handleSetSampleArgumentCommand(uint8_t *commandPacket, uint8_t *outputBuffer);
static int handleStartCollectionCommand(uint8_t *outputBuffer);
static int handleStopCollectionCommand(uint8_t *outputBuffer);
static int handleCollectionCommand(uint8_t *commandPacket, uint8_t *outputBuffer);
static int handleSetChainMaskCommand(uint8_t *commandPacket, uint8_t *outputBuffer);
static void WriteIntToEEPROM(uint8_t address, uint8_t* value);
static void ReadIntFromEEPROM(uint8_t address, uint8_t* destination);
static void verifyVersion(void);
static void initStateFromEEPROM(void);
static void spreadMotorStop(void);
static void spreadMotorForward(uint16_t dac);
static void spreadMotorReverse(uint16_t dac);

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
    OSC.CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;    //enable the 32MHz ring oscillator and the 32kHz RTC oscillator
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));        //wait for 32MHz to stabilize
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));        //wait for 32kHz to stabilize

    DFLLRC32M.CTRL = DFLL_ENABLE_bm;
    DFLLRC2M.CTRL = DFLL_ENABLE_bm;

    CCP = CCP_IOREG_gc;                //clock is protected by Configuration Change Protection - must be properly disabled to change clock source
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;        //switch to 32MHz oscillator
    CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;    //Select 1.024 kHz from internal oscillator for RTC clock source
}

ISR(FREERUN_TC_vect)
{
    freerun_flag = 1;
}

static void spreadMotorStop(void)
{
    //MOTOR_EN_PORT.OUTCLR = MOTOR_EN_BITMASK;
    MOTOR_DIR_PORT.OUTCLR = MOTOR_DIR_BITMASK | MOTOR_nDIR_BITMASK;
}

static void spreadMotorForward(uint16_t dac)
{
    DACB.CH0DATA = dac;
    
    MOTOR_DIR_PORT.OUTCLR = MOTOR_DIR_BITMASK;
    MOTOR_DIR_PORT.OUTSET = MOTOR_nDIR_BITMASK;
    //MOTOR_EN_PORT.OUTSET = MOTOR_EN_BITMASK;   
}

static void spreadMotorReverse(uint16_t dac)
{
    DACB.CH0DATA = dac;
    
    MOTOR_DIR_PORT.OUTCLR = MOTOR_nDIR_BITMASK;
    MOTOR_DIR_PORT.OUTSET = MOTOR_DIR_BITMASK;
    //MOTOR_EN_PORT.OUTSET = MOTOR_EN_BITMASK;   
}



/************************************************************************
* handleFingerCommand(commandPacket,outputBuffer)
*
* Handles a command to drive the Finger Motor either Forward, Reverse,
* or Stopped.  Generates a proper reply in outputBuffer
************************************************************************/
static int handleFingerCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint8_t status = STATUS_OK;
    uint16_t dac = 0;

    // position control
    if ((commandPacket[COMMAND_OFFSET] & MOTOR_COMMAND_SCHEME_BITMASK) == MOTOR_COMMAND_POSITION)
    {
        // don't go into position mode if stop direction
        if ((commandPacket[COMMAND_OFFSET] & MOTOR_COMMAND_DIRECTION_BITMASK) == MOTOR_COMMAND_STOP)
        {
            spreadMotorStop();
            spreadMotorMode = SPREAD_MODE_NONE;
        }
        else
        {
            memcpy(&targetEncoder, &commandPacket[PAYLOAD_OFFSET], 2);
            
            if ((commandPacket[COMMAND_OFFSET] & MOTOR_COMMAND_DIRECTION_BITMASK) == MOTOR_COMMAND_REVERSE)
                targetEncoder = -targetEncoder;
            
            spreadMotorMode = SPREAD_MODE_POSITION;
        }
    }
    else
    {
        spreadMotorMode = SPREAD_MODE_NONE;
        
        switch(commandPacket[COMMAND_OFFSET] & MOTOR_COMMAND_DIRECTION_BITMASK)
        {
            case MOTOR_COMMAND_STOP:
                spreadMotorStop();
                break;
            case MOTOR_COMMAND_REVERSE:
                memcpy(&dac, &commandPacket[PAYLOAD_OFFSET], 2);
                spreadMotorReverse(dac);
                break;
            case MOTOR_COMMAND_FORWARD:
                memcpy(&dac, &commandPacket[PAYLOAD_OFFSET], 2);
                spreadMotorForward(dac);
                break;
            default:
                status = UNKNOWN_COMMAND;
                break;
        }
    }
    
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = FINGER_COMMAND_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = status;
    outputBuffer[4] = computeChecksum(outputBuffer,4); //this is the checksum
    return 5;
}

static void upstreamTX(volatile uint8_t *targetBuffer, int numBytes)
{
  int16_t i;

  for(i=0;i<numBytes;i++)
    {
      while(!(UPSTREAM_USART.STATUS & USART_DREIF_bm));
      UPSTREAM_USART.DATA = targetBuffer[i];
    }
}

void BootRelay(USART_t *targetUSART, uint8_t *commandPacket)
{
  // overview: 1)send a bootload command to the target
  // overview: 2)configure for 115200 baud passthrough
  // overview: 3)start a 20 second timer
  // overview: 4)do the serial passthrough, blink LED
  // overview: 5)wait for timer to finish, then reset/reboot

  cli(); // disable all interrupts, nothing else going on
  DMA.CTRL = 0; // disable the DMA controller
  
  // repeat the command to the target
  targetUSART->CTRLA = 0x00; // disable interrupts
  targetUSART->CTRLB |= USART_RXEN_bm | USART_TXEN_bm; //enable RX and TX
  for(int i=0;i<COMMAND_PACKET_SIZE;i++)
    {
      while(!(targetUSART->STATUS & USART_DREIF_bm));
      targetUSART->DATA = commandPacket[i];
    }
  // dont bother waiting for a response verification
  while(!(targetUSART->STATUS & USART_DREIF_bm)); // wait for the data to be read
  targetUSART->STATUS = USART_TXCIF_bm; // clear the TX done flag
  while(!(targetUSART->STATUS & USART_TXCIF_bm)); // wait for the TX to be done
  
  // Reconfigure the serial ports
  //int bsel = 1047;       // 115200 @ 32Mhz as calculated from ProtoTalk.net
  //uint8_t bscale = 10;      // 115200 @ 32Mhz as calculated from ProtoTalk.net
  int bsel = 3269;       // 38,400 @ 32Mhz as calculated from ProtoTalk.net
  uint8_t bscale = -6;      // 38,400 @ 32Mhz as calculated from ProtoTalk.net
  UPSTREAM_USART.CTRLA = 0x00; // disable interrupts
  UPSTREAM_USART.BAUDCTRLA = (uint8_t) bsel;
  UPSTREAM_USART.BAUDCTRLB = (bscale << 4) | (bsel >> 8);
  UPSTREAM_USART.CTRLB = USART_RXEN_bm | USART_TXEN_bm;    //enable RX and TX
  targetUSART->CTRLA = 0x00; // disable interrupts
  targetUSART->BAUDCTRLA = (uint8_t) bsel;
  targetUSART->BAUDCTRLB = (bscale << 4) | (bsel >> 8);
  targetUSART->CTRLB = USART_RXEN_bm | USART_TXEN_bm;    //enable RX and TX
  
  int sec_cnt=0;
  uint32_t loop_cnt=0;
  uint8_t up_fifo[16];
  uint8_t up_head = 0;
  uint8_t up_tail = 0;
  uint8_t down_fifo[16];
  uint8_t down_head = 0;
  uint8_t down_tail = 0;

  // relay chars while waiting for timeout
  while(sec_cnt < 200) {
    loop_cnt++;
    if(loop_cnt > 90000L) { // approximately 0.1sec experimentally
      loop_cnt=0;
      sec_cnt++;
      if((sec_cnt&0x03)==0x03)
	PORTK.OUTSET = 0x08; //LED debug
      else
	PORTK.OUTCLR = 0x08; //LED debug
    }
    if (targetUSART->STATUS & USART_RXCIF_bm) {
      up_fifo[up_head++] = targetUSART->DATA;
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
    if (down_head != down_tail && (targetUSART->STATUS & USART_DREIF_bm)) {
      targetUSART->DATA = down_fifo[down_tail++];
      down_tail %= sizeof(down_fifo);
    }
  }

  // 30sec timer finished, reset/reboot
  wdt_enable(WDT_PER_256CLK_gc);
  while(1);  // use watchdog timer to trigger a reset

#if 0 // a less reliable medthod to kick off the app and skip the bootloader
  CCP = CCP_IOREG_gc; // make sure interrupt vectors are configured for app
  PMIC.CTRL = 0;      // disable the interrupt controller
  // Jump into main code                                                       
  asm("jmp 0");
#endif

}

static int handleBootloaderCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
  // prepare to echo back the command for verify
  outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 4;
  outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
  outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = BOOTLOADER_OPCODE;
  outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
  outputBuffer[4] = commandPacket[PAYLOAD_OFFSET];
  outputBuffer[5] = computeChecksum(outputBuffer,5); //this is the checksum
  upstreamTX(outputBuffer, 6);

  switch(0xf0 & commandPacket[PAYLOAD_OFFSET])
  {
  case PALM_CHAINADDRESS:
    // self bootload
    wdt_enable(WDT_PER_256CLK_gc);
    while(1);  // use watchdog timer to trigger a reset                                     
    break;
  case FINGER1_CHAINADDRESS:
    BootRelay(&PROX1_USART,commandPacket);
    break;
  case FINGER2_CHAINADDRESS:
    BootRelay(&PROX2_USART,commandPacket);
    break;
  case FINGER3_CHAINADDRESS:
    BootRelay(&PROX3_USART,commandPacket);
    break;
  case MOTOR1_CHAINADDRESS:
    BootRelay(&MTR1_USART,commandPacket);
    break;
  case MOTOR2_CHAINADDRESS:
    BootRelay(&MTR2_USART,commandPacket);
    break;
  case TACTILE_CHAINADDRESS:
    BootRelay(&TACT_USART,commandPacket);
    break;
  default:
    //do nothing
    break;
  }
  return 5;
}

/************************************************************************
* handleSetSamplePeriodCommand(commandPacket,outputBuffer)
*
* Handles a command to set the untriggered roll period.
* Generates a proper reply in outputBuffer
************************************************************************/
static int handleSetSamplePeriodCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    memcpy(&samplePeriod, &commandPacket[PAYLOAD_OFFSET], 2);
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = SET_SAMPLE_PERIOD_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
    outputBuffer[4] = computeChecksum(outputBuffer,4); //this is the checksum
    return 5;
}

/************************************************************************
* handleSetSampleArgumentCommand(commandPacket,outputBuffer)
*
* Handles a command to set the untriggered roll argument.
* Generates a proper reply in outputBuffer
************************************************************************/
static int handleSetSampleArgumentCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    memcpy(&sampleArgument,&commandPacket[PAYLOAD_OFFSET],2);
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = SET_SAMPLE_ARGS_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
    outputBuffer[4] = computeChecksum(outputBuffer,4); //this is the checksum
    return 5;
}

/************************************************************************
* handleStartCollectionCommand(outputBuffer)
*
* Handles a command to begin an untriggered roll
* Generates a proper reply in outputBuffer
************************************************************************/
static int handleStartCollectionCommand(uint8_t *outputBuffer)
{
    //Activate the freerun timer for generating data collection packets
    //At 32 MHz internal oscillator with 64 prescaler, each tick of the clock is half a microsecond.

    freerun_flag = 1;

    FREERUN_TC.CTRLB = TC_WGMODE_NORMAL_gc;
    FREERUN_TC.CTRLC = 0x00;
    FREERUN_TC.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
    FREERUN_TC.CTRLE = 0x00;
    FREERUN_TC.PERBUF = samplePeriod >> 1; //BA: fix bit shift direction
    FREERUN_TC.INTCTRLA = TC_OVFINTLVL_LO_gc;
    FREERUN_TC.CTRLA = TC_CLKSEL_DIV64_gc;


    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = START_COLLECTION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
    outputBuffer[4] = computeChecksum(outputBuffer,4); //this is the checksum
    return 5;
}

/************************************************************************
* handleStopCollectionCommand(outputBuffer)
*
* Handles a command to stop the untriggered roll.
* Generates a proper reply in outputBuffer
************************************************************************/
static int handleStopCollectionCommand(uint8_t *outputBuffer)
{
    //Shutdown the freerun timer for generating data collection packets
    FREERUN_TC.INTCTRLA = TC_OVFINTLVL_OFF_gc;
    FREERUN_TC.CTRLA = TC_CLKSEL_OFF_gc;

    freerun_flag = 0;


    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = STOP_COLLECTION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
    outputBuffer[4] = computeChecksum(outputBuffer,4); //this is the checksum
    return 5;
}

/************************************************************************
* handleCollectionCommand(commandPacket,outputBuffer)
*
* Handles a command to poll this device's sensors.
* For the Palm this just consists of some ADCs.
* Generates a proper reply in outputBuffer
************************************************************************/
static int handleCollectionCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint16_t collectionBitfield;
    float thermistorVoltage;
    float thermistorTemperature;
    uint8_t responseSize = 0;

    int16_t adcData[6];
    uint16_t roundedTemperature;

    memcpy(&collectionBitfield, &commandPacket[PAYLOAD_OFFSET], 2);

    if (collectionBitfield & (DATA_COLLECTION_EXTERNALSUPPLY_BITMASK
                              //| DATA_COLLECTION_MOTORSTATORTEMP_BITMASK
                              //| DATA_COLLECTION_MOTORCURRENT_BITMASK
                              | DATA_COLLECTION_AIRTEMPERATURE_BITMASK))

    {
      captureSweep(adcData); // gather all the adc data, then pick out the specifics below
    }

    if(collectionBitfield & DATA_COLLECTION_FINGERROTATION_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM+responseSize], &adjustedEncoder, 2);
        responseSize += 2;
    }

    // if(collectionBitfield & DATA_COLLECTION_MOTORCURRENT_BITMASK)
    // {
    //     memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM+responseSize], &adcData[MOTORCURRENT_MONITOR_OFFSET], 2);
    //     responseSize += 2;
    // }

    // if(collectionBitfield & DATA_COLLECTION_MOTORSTATORTEMP_BITMASK)
    // {
    //     thermistorVoltage = ADC_CODES_TO_VOLTS_SIGNED(adcData[MOTORTEMP_MONITOR_OFFSET]);
    //     thermistorTemperature = Thermistor_RtoT(Thermistor_VtoR(thermistorVoltage));
    //     roundedTemperature = round(thermistorTemperature * 100.0);

    //     memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM+responseSize], &roundedTemperature, 2);
    //     responseSize += 2;
    // }

    if(collectionBitfield & DATA_COLLECTION_EXTERNALSUPPLY_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM+responseSize],&adcData[EXTERNAL_MONITOR_OFFSET], 6);
        responseSize += 6;
    }

    if(collectionBitfield & DATA_COLLECTION_AIRTEMPERATURE_BITMASK)
    {
        thermistorVoltage = ADC_CODES_TO_VOLTS_SIGNED(adcData[EXTERNALTEMP_MONITOR_OFFSET]);
        thermistorTemperature = Thermistor_RtoT(Thermistor_VtoR(thermistorVoltage));
        roundedTemperature = round(thermistorTemperature * 100.0);
        
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM+responseSize],&roundedTemperature, 2);
        responseSize += 2;
    }

    if(collectionBitfield & DATA_COLLECTION_DEBUG_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM+responseSize], &RxCheckSumErrCnt, 14);
        responseSize += 14;
    }

    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3+responseSize;
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = DATA_COLLECTION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
    outputBuffer[4+responseSize] = computeChecksum(outputBuffer, 4+responseSize); //this is the checksum
    return 5+responseSize;

}

static int handleCalibrationCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint16_t collectionBitfield;
    memcpy(&collectionBitfield, &commandPacket[PAYLOAD_OFFSET], 2);

    if (collectionBitfield & DATA_COLLECTION_DEBUG_BITMASK)
    {
        RxCheckSumErrCnt[0] = 0;
        RxCheckSumErrCnt[1] = 0;
        RxCheckSumErrCnt[2] = 0;
        RxCheckSumErrCnt[3] = 0;
        RxCheckSumErrCnt[4] = 0;
        RxCheckSumErrCnt[5] = 0;
        RxCheckSumErrCnt[6] = 0;
    }
    
    if (collectionBitfield & DATA_COLLECTION_FINGERROTATION_BITMASK)
    {
        uint32_t temp = rawEncoder;
        WriteIntToEEPROM(EEPROM_ADDRESS_ENCODER_OFFSET, (uint8_t*)&temp);
        lastEncoder = rawEncoder;
        adjustedEncoder = 0;
        encoderInitialized = 1;
        targetEncoder = 0;
    }
    
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = CALIBRATION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
    outputBuffer[4] = computeChecksum(outputBuffer, 4); //this is the checksum
    return 5;
}

static int handleSetChainMaskCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    memcpy(&chainMask, &commandPacket[PAYLOAD_OFFSET], 1);
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = SET_CHAIN_MASK_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
    outputBuffer[4] = computeChecksum(outputBuffer,4); //this is the checksum
    return 5;
}

static void WriteIntToEEPROM(uint8_t address, uint8_t* value)
{
    for(uint8_t i=0; i<4; i++)
        eeprom_write_byte((uint8_t *) (address*4+i), *(value+i));
    //eeprom_write_dword((uint32_t *) (address*4), *(value));
}

static void ReadIntFromEEPROM(uint8_t address, uint8_t* destination)
{
    for(uint8_t i=0; i<4; i++)
        *(destination+i) = eeprom_read_byte((uint8_t *) (address*4+i));
    //*destination = eeprom_read_dword((uint32_t *) (address*4));
}

static void verifyVersion(void)
{
    uint32_t temp = 0;
    ReadIntFromEEPROM(EEPROM_ADDRESS_FIRMWARE_VERSION, (uint8_t*)&temp);
    if (temp != FIRMWARE_VERSION)
    {
        temp = FIRMWARE_VERSION;
        WriteIntToEEPROM(EEPROM_ADDRESS_FIRMWARE_VERSION, (uint8_t*)&temp);
    }
}

static void initStateFromEEPROM(void)
{
    uint32_t temp = 0;
    ReadIntFromEEPROM(EEPROM_ADDRESS_LED, (uint8_t*)&temp);
    if (temp)
        LEDon();
    else
        LEDoff();
    
    ReadIntFromEEPROM(EEPROM_ADDRESS_ENCODER_OFFSET, (uint8_t*)&temp);
    lastEncoder = (int16_t)temp;
    
    ReadIntFromEEPROM(EEPROM_ADDRESS_SPREAD_DEADBAND, (uint8_t*)&temp);
    spreadDeadband = (int16_t)temp;

    ReadIntFromEEPROM(EEPROM_ADDRESS_SPREAD_P, (uint8_t*)&temp);
    spreadP = (int16_t)temp;
}

static int handleEEPROMCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint8_t responseSize = 4;

    uint8_t opcode = commandPacket[COMMAND_OFFSET] & OPCODE_BITMASK;
    uint8_t address = commandPacket[COMMAND_OFFSET] & 0x1F;

    switch(opcode)
    {
        case MOTOR_PARAMETER_RE_L_OPCODE:
        case MOTOR_PARAMETER_RE_H_OPCODE:
            // Read 4 bytes of data
            ReadIntFromEEPROM(address, &outputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM]);
            // finish up the packet
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3+responseSize;
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = commandPacket[COMMAND_OFFSET];
            outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
            outputBuffer[4+responseSize] = computeChecksum(outputBuffer, 4+responseSize); //this is the checksum
            return 5+responseSize;
        case MOTOR_PARAMETER_WR_L_OPCODE:
        case MOTOR_PARAMETER_WR_H_OPCODE:
            
            // Write 32 bits of data
            WriteIntToEEPROM(address, &commandPacket[PAYLOAD_OFFSET]);
            
            // re-init state
            initStateFromEEPROM();
            
            // ack the command
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = commandPacket[COMMAND_OFFSET];
            outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = STATUS_OK;
            outputBuffer[4] = computeChecksum(outputBuffer, 4); //this is the checksum
            return 5;
        default:
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = commandPacket[COMMAND_OFFSET];
            outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = UNKNOWN_COMMAND;
            outputBuffer[4] = computeChecksum(outputBuffer, 4); //this is the checksum
            return 5;
    }
}

int processCommand(uint8_t *commandPacket,uint8_t *outputBuffer)
{
    uint8_t opcode;

    opcode = commandPacket[COMMAND_OFFSET] & OPCODE_BITMASK;

    switch(opcode)
    {

        case DATA_COLLECTION_OPCODE:
            return handleCollectionCommand(commandPacket,outputBuffer);

        case SET_SAMPLE_PERIOD_OPCODE:
            return handleSetSamplePeriodCommand(commandPacket,outputBuffer);

        case SET_SAMPLE_ARGS_OPCODE:
            return handleSetSampleArgumentCommand(commandPacket,outputBuffer);

        case START_COLLECTION_OPCODE:
            return handleStartCollectionCommand(outputBuffer);

        case STOP_COLLECTION_OPCODE:
            return handleStopCollectionCommand(outputBuffer);

        case FINGER_COMMAND_OPCODE:
            return handleFingerCommand(commandPacket,outputBuffer);

        case BOOTLOADER_OPCODE:
            return handleBootloaderCommand(commandPacket,outputBuffer);

        case CALIBRATION_OPCODE:
            return handleCalibrationCommand(commandPacket, outputBuffer);
            
        case MOTOR_PARAMETER_RE_L_OPCODE:
        case MOTOR_PARAMETER_RE_H_OPCODE:
        case MOTOR_PARAMETER_WR_L_OPCODE:
        case MOTOR_PARAMETER_WR_H_OPCODE:
            return handleEEPROMCommand(commandPacket, outputBuffer);

        case SET_CHAIN_MASK_OPCODE:
            return handleSetChainMaskCommand(commandPacket, outputBuffer);

        default:
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_LSB] = 3;
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET_PALM_MSB] = 0;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET_PALM] = opcode;
            outputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] = UNKNOWN_COMMAND;
            outputBuffer[4] = computeChecksum(outputBuffer, 4); //this is the checksum
            return 5;
    }
}

void LEDon()
{
  PORTK.OUTSET = 0x08; //LED
}
void LEDoff()
{
  PORTK.OUTCLR = 0x08; //LED
}
void LEDtoggle()
{
  PORTK.OUTTGL = 0x08; //LED
}



/************************************************************************
* MAIN function.
*
* This initializes the other port modules and polls a few volatile status
* bits set in ISRs.
************************************************************************/
int main(void)
{
    configurePortIO();
    LEDon();
    cli(); //disable all interrupts for clock reset
    _delay_ms(1); // for stability of supplies
    configureClocks();
    _delay_ms(1); // for stability of clocks
    configureADC();
    configureRouterUSARTs();
    configureDAC();

    // i don't know why a dummy read is required, but it is
    readEncoder(); //dummy read
    encoderInitialized = 0; // re-initialize

    _delay_ms(100); // for stability of inputs
    //calibrateIMON_ADCOffset();
    LEDoff();
  
    initStateFromEEPROM();
    encoderInitialized = 1;
    
    verifyVersion();

    //tell event system to pay attention to low-priority and high-priority interrupts
    PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
    PMIC.CTRL |= PMIC_RREN_bm; // enable round robin for low level interrupts
    sei();

    while(1)
    {
        readEncoder();
        
        // position mode for spread motor
        if (spreadMotorMode == SPREAD_MODE_POSITION)
        {
            if (targetEncoder - adjustedEncoder > spreadDeadband)
            {
                uint16_t speed = abs(targetEncoder - adjustedEncoder) * spreadP;
                if (speed > 4095) // 12 bit DAC
                    speed = 4095;
                spreadMotorForward(speed);
            }
            else if (adjustedEncoder - targetEncoder > spreadDeadband)
            {
                uint16_t speed = abs(targetEncoder - adjustedEncoder) * spreadP;
                if (speed > 4095) // 12 bit DAC
                    speed = 4095;
                spreadMotorReverse(speed);
            }
            else
            {
                spreadMotorStop();
            }
        }
        
        if(commandReady)
        {
            doRouterTask();
        }

        if(freerun_flag)
        {
            freerun_flag = 0;
            doFreerunTask();
        }
    }
    return 0;
}
