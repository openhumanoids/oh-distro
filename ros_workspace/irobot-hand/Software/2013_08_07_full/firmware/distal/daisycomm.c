/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-COMMON-0-daisycomm.c
 // Creation Date:    22 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Common routines for daisy chaining

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

/************************************************************************
 * This module operates the Daisy Chain protocol in the ARM-H system.
 * It assumes DMA transfers will be used on a half-duplex downstream
 * and half-duplex upstream link to move data between various devices in
 * the system.  Configuration must be provided in the file
 * C1482-SRC-COMMON-0-daisyconfig.h for each project using this module.
 *
 * Data arrives on an interrupt-driven transfer to determine the number of additional
 * bytes to arrive.  A DMA transfer then captures the remaining bytes of payload.
 ************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#define F_CPU 32000000L
#include <util/delay.h>

#include "distal.h"
#include "../common/daisycomm.h"
#include "daisyconfig.h"

void LEDtoggle(void); // function in finger.c

//#define DELAY_ROUTINE() asm("nop")
#define DELAY_ROUTINE() _delay_us(6); //enough for a full character
//#define DELAY_ROUTINE() _delay_us(10);

typedef enum SERIAL_STREAM_enum
{
    DOWNSTREAM,
    UPSTREAM
} SERIAL_STREAM_t;

typedef enum SERIAL_DIRECTION_enum
{
    INBOUND,
    OUTBOUND
} SERIAL_DIRECTION_t;

typedef enum DAISY_STATE_enum
{
    DAISY_IDLE,
    DAISY_TRANSMITTING
} DAISY_STATE_t;

volatile uint8_t notifyDaisy = 0;

static volatile uint8_t inbound_upstream_data[COMMAND_PACKET_SIZE];
static volatile uint8_t outbound_upstream_data[MAX_PACKET_SIZE];


static volatile SERIAL_DIRECTION_t UPSTREAM_CONFIG = INBOUND;

static volatile int upstreamRxDone = 0;
static volatile int upstreamTxDone = 0;
static volatile uint8_t upstreamBusy = 0;
uint16_t RxCheckSumErrCnt[2] = {0, 0};


static void configureUSARTHardware(USART_t *targetUSART, int isPC, int isDownstream);
static void prepareDMAChannel(DMA_CH_t *targetChannel, uint8_t usartTrigger, USART_t *targetUSART, volatile uint8_t *targetBuffer, uint8_t isOutbound);
static void activateDMAChannel(DMA_CH_t *targetChannel,int numBytes);
static void configureHalfDuplexLink(SERIAL_STREAM_t selectedStream, SERIAL_DIRECTION_t selectedDirection);


ISR(DAISY_TC_vect)
{
    //Check if the DMA is busy and waiting on incoming data
    if(UPSTREAM_CONFIG == OUTBOUND)
    {
        //No need to reset DMA
        upstreamBusy = 0;
    }
    else
    {
        if((UPSTREAM_DMA.CTRLB & DMA_CH_CHBUSY_bm) || !(UPSTREAM_DMA.CTRLA & DMA_CH_ENABLE_bm))
        {
            upstreamBusy++;
        }

        if(upstreamBusy > 2)
        {
            //Cancel and rearm the incoming DMA
            UPSTREAM_DMA.CTRLA = 0x00;
            //Wait for it to disable
            while(UPSTREAM_DMA.CTRLA & DMA_CH_ENABLE_bm);

            //Issue a reset
            UPSTREAM_DMA.CTRLA = DMA_CH_RESET_bm;

            //Re-arm
            prepareDMAChannel(&UPSTREAM_DMA, UPSTREAM_DIST_USART_INBOUND_TRIGGER, &UPSTREAM_DIST_USART, inbound_upstream_data,0);
            activateDMAChannel(&UPSTREAM_DMA,COMMAND_PACKET_SIZE);
        }
    }
}

/************************************************************************
 * ISRs for the DMA engines.
 * Set flags for the Daisy Chain Task
 ************************************************************************/
ISR(UPSTREAM_DMA_vect)
{
    //LEDtoggle();
    
    if(UPSTREAM_DMA.CTRLB & DMA_CH_ERRIF_bm)
    {
        //Acknowledge the error
        //It may be from an aborted transfer, so just return.  Re-arm the DMA
        //UPSTREAM_DMA.CTRLB = UPSTREAM_DMA.CTRLB | DMA_CH_ERRIF_bm;
        //Cancel and rearm the incoming DMA
        return;
    }

    if(UPSTREAM_DMA.CTRLB & DMA_CH_TRNIF_bm)
    {
        //The transfer is complete and should be acked
        UPSTREAM_DMA.CTRLB = UPSTREAM_DMA.CTRLB | DMA_CH_TRNIF_bm;
        upstreamBusy = 0;
        notifyDaisy = 1;
        upstreamRxDone = 1;
    }
}

/*
ISR(UPSTREAM_PROX_USART_TXDONE_vect)
{
    upstreamBusy = 0;
    notifyDaisy = 1;
    upstreamTxDone = 1;
    UPSTREAM_PROX_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_OFF_gc;
    UPSTREAM_PROX_USART.STATUS = USART_TXCIF_bm;
}
*/

ISR(UPSTREAM_DIST_USART_TXDONE_vect)
{
    upstreamBusy = 0;
    notifyDaisy = 1;
    upstreamTxDone = 1;
    UPSTREAM_DIST_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_OFF_gc;
    UPSTREAM_DIST_USART.STATUS = USART_TXCIF_bm;
}




/************************************************************************
* configureUSART(targetUSART, isPC, useDMA):
* Responsible for configuring the USART module.
*
* targetUSART is a pointer to the USART_t struct to be configured
* when isPC is 1, set to communicate at 115200 baud
* using 8 data bits, 1 stop bit.
*
* When isPC is 0, set to communicate at 2 MBaud
* using 8 data bits, 1 stop bit.
*
************************************************************************/
static void configureUSARTHardware(USART_t *targetUSART, int isPC, int isDownstream)
{
    if(isDownstream)
    {
        targetUSART->CTRLA = USART_RXCINTLVL_LO_gc;            //Enable RX interrupt at LO priority
    }


    if(isPC)
    {
        //COMMAND_USART.BAUDCTRLA = 34;                            //Configure for 57600 baud rate with normal baud rate generation
        //COMMAND_USART.BAUDCTRLB = 0;
        targetUSART->BAUDCTRLA = 33;                            //Configure for 115200 with fractional baud rate generation
        targetUSART->BAUDCTRLB = 0xF0;
    } else {
        targetUSART->BAUDCTRLA = 0;                                //Configure for 2 MBaud by setting baud rate to zero
        targetUSART->BAUDCTRLB = 0x00;                            //and BSEL to zero
    }
    targetUSART->CTRLC = USART_CMODE_ASYNCHRONOUS_gc | \
                         USART_PMODE_DISABLED_gc | \
                          USART_CHSIZE_8BIT_gc;                //Configure port settings for 8 bits of data, 1 stop bit.
    targetUSART->CTRLB |= USART_RXEN_bm | USART_TXEN_bm;    //enable RX and TX
    return;
}

/************************************************************************
 * configureHalfDuplexLink(selectedStream,selectedDirection)
 *
 * Configures hardware and DMA channels to support the requested stream configuration
 ************************************************************************/

static void configureHalfDuplexLink(SERIAL_STREAM_t selectedStream, SERIAL_DIRECTION_t selectedDirection)
{
    cli();
    switch(selectedStream)
    {
        case UPSTREAM:
            //Abort any pending DMA transfers
            if(UPSTREAM_DMA.CTRLB & DMA_CH_CHBUSY_bm)
            {
                UPSTREAM_DMA.CTRLA &= ~DMA_CH_ENABLE_bm;
            }

            upstreamRxDone = 0;
            //upstreamTxDone = 0;
            upstreamBusy = 0;

            //Configure the transceivers and prepare DMA
            if(selectedDirection == INBOUND)
            {
                prepareDMAChannel(&UPSTREAM_DMA, UPSTREAM_DIST_USART_INBOUND_TRIGGER, &UPSTREAM_DIST_USART, inbound_upstream_data,0);

                activateDMAChannel(&UPSTREAM_DMA,COMMAND_PACKET_SIZE);
                UPSTREAM_CONFIG = INBOUND;

            } else {
                UPSTREAM_DIST_USART.STATUS = USART_TXCIF_bm;
                UPSTREAM_DIST_USART.CTRLA = USART_TXCINTLVL_LO_gc | USART_RXCINTLVL_OFF_gc;
                prepareDMAChannel(&UPSTREAM_DMA, UPSTREAM_DIST_USART_OUTBOUND_TRIGGER, &UPSTREAM_DIST_USART, outbound_upstream_data + 1,1);
                //Do not send all data through DMA.  Instead send outbound upstream data + 1.  This allows for the insertion of a small delay between size and data
                UPSTREAM_CONFIG = OUTBOUND;
            }
            break;
        default:
            break;
    }

    sei();
    return;
}

/************************************************************************
* prepareDMAChannel(targetChannel,usartTrigger, targetUSART,targetBuffer)
*
* Configures targetChannel (a DMA.CHX structure) of the DMA module
* to collect data on usartTrigger signal produced by targetUSART and place
* it into the buffer targetBuffer.
*
* This should be called before fireDMAChannel()
************************************************************************/
static void prepareDMAChannel(DMA_CH_t *selectedChannel, uint8_t usartTrigger, USART_t *targetUSART, volatile uint8_t *targetBuffer, uint8_t isOutbound)
{
    //Configure the chosen channel to read from a fixed UART into a linear array
    if(isOutbound)
    {
        selectedChannel->ADDRCTRL = DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc | \
                       DMA_CH_DESTRELOAD_BURST_gc | DMA_CH_DESTDIR_FIXED_gc;
    } else {
        selectedChannel->ADDRCTRL = DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_FIXED_gc | \
                       DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc;
    }

    //Set to perform one block transfer corresponding to one command packet
    //Technically not necessary because REPEAT will not be set in CTRLA
    selectedChannel->REPCNT = 0x01;

    //This part has 16 bit pointers, so cast appropriately.
    //Memory addresses are 24 bits to allow for external memory to be mapped into one address space
    //Same with internal EEPROM Data memory.
    //Since these locations are in lower memory, fix the high order address bits to zero.
    //Mapping external memory will require more care.
    if(isOutbound)
    {
        selectedChannel->DESTADDR0 = (uint16_t) &targetUSART->DATA & 0x00FF;
        selectedChannel->DESTADDR1 = (uint16_t) &targetUSART->DATA >> 8;
        selectedChannel->DESTADDR2 = 0;

        selectedChannel->SRCADDR0 = (uint16_t)targetBuffer & 0x00FF;
        selectedChannel->SRCADDR1 = (uint16_t)targetBuffer >> 8;
        selectedChannel->SRCADDR2 = 0;
    } else {
        selectedChannel->DESTADDR0 = (uint16_t)targetBuffer & 0x00FF;
        selectedChannel->DESTADDR1 = (uint16_t)targetBuffer >> 8;
        selectedChannel->DESTADDR2 = 0;

        selectedChannel->SRCADDR0 = (uint16_t) &targetUSART->DATA & 0x00FF;
        selectedChannel->SRCADDR1 = (uint16_t) &targetUSART->DATA >> 8;
        selectedChannel->SRCADDR2 = 0;
    }

    //Trigger on received data
    selectedChannel->TRIGSRC = usartTrigger;

    //Interrupt on this channel when the transaction is complete, and clear any pending flags
    if(isOutbound)
    {
        selectedChannel->CTRLB = DMA_CH_TRNINTLVL_OFF_gc | DMA_CH_TRNIF_bm;
        targetUSART->STATUS = USART_TXCIF_bm;
        targetUSART->CTRLA = USART_TXCINTLVL_LO_gc | USART_RXCINTLVL_OFF_gc;
    } else
    {
        selectedChannel->CTRLB = DMA_CH_TRNINTLVL_LO_gc | DMA_CH_TRNIF_bm;
    }
}

/************************************************************************
 * activateDMAChannel(selectedChannel,numBytes)
 *
 * Activates the DMA channel identified by selectedChannel (a DMA.CHX structure)
 * to receive numBytes bytes.  The DMA channel must have been previously prepared
 * with prepareDMAChannel().
 ************************************************************************/
static void activateDMAChannel(DMA_CH_t *selectedChannel,int numBytes)
{
    //Set block size to packet size
    selectedChannel->TRFCNT = numBytes;
    //Enable the channel on single shot mode with a burst length of one.  Whenever the trigger arrives,
    //only one burst will be performed.
    selectedChannel->CTRLA = DMA_CH_BURSTLEN_1BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_ENABLE_bm;
}

/************************************************************************
 * Returns the appropriate checksum for the first packetSize bytes of packetBuffer
 * As a shortcut, feeding an entire packet to this routine should result in a return
 * value of zero for a proper packet.
 ************************************************************************/
int computeChecksum(uint8_t *packetBuffer, int packetSize)
{
    uint8_t accumulator = 0;
    for(int i=0;i<packetSize;i++)
    {
        accumulator += packetBuffer[i];
    }

    return (0x00 - accumulator);
}


/************************************************************************
 * configureDaisyUSART()
 *
 * Configures the daisy chain system based on parameters in the daisyconfig.h file
 ************************************************************************/
void configureDaisyUSART(void)
{
    //First initialize the upstream
    configureUSARTHardware(&UPSTREAM_DIST_USART,0,0);

    //Prepare DMA transfers
    DMA.CTRL = DMA_ENABLE_bm;

    //Assume default configuration with downstream OUT and upstream IN
    configureHalfDuplexLink(UPSTREAM,INBOUND);

    //Activate the heartbeat timer for resetting inbound DMA as necessary
    //Interrupt every half millisecond
    //At 32 MHz internal oscillator with 1024 prescaler,
    DAISY_TC.CTRLB = TC_WGMODE_NORMAL_gc;
    DAISY_TC.CTRLC = 0x00;
    DAISY_TC.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
    DAISY_TC.CTRLE = 0x00;
    DAISY_TC.PERBUF = 16;
    DAISY_TC.INTCTRLA = TC_OVFINTLVL_LO_gc;
    DAISY_TC.CTRLA = TC_CLKSEL_DIV1024_gc;

    //Activate the Upstream DMA for a command packet of data
    //activateDMAChannel(&UPSTREAM_DMA,COMMAND_PACKET_SIZE);
}

/************************************************************************
 * doDaisyTask()
 *
 * Main routine of the daisy chain process
 * Maintains system state and manages the transmission/reception of chained data
 * Should be called whenever notifyDaisy is set to 1.
 ************************************************************************/

void doDaisyTask(void)
{
  //This is set to IDLE when the upstream port is idle and TRANSMITTING when it is in use
  static DAISY_STATE_t daisyState = DAISY_IDLE;
  uint8_t packetSize;

  //First clear the daisy notification flag
  cli();
  notifyDaisy = 0;
  sei();

  switch(daisyState)
    {
    case DAISY_IDLE:

      if(upstreamRxDone)
    {
      //A packet arrived from upstream.  Parse it and compute some sort of reply.
      cli();
      upstreamRxDone = 0;
      sei();

      //A data packet has arrived from upstream.  Validate it
      if(computeChecksum((uint8_t *)inbound_upstream_data,COMMAND_PACKET_SIZE) != 0x00)
        {
          //Invalid checksum
          RxCheckSumErrCnt[0]++;
          outbound_upstream_data[0] = 3; //Packet size
          outbound_upstream_data[1] = inbound_upstream_data[1]; //Reflected command byte
          outbound_upstream_data[2] = CHECKSUM_ERROR; //Checksum error
          outbound_upstream_data[3] = computeChecksum((uint8_t *)outbound_upstream_data,3); //checksum
          configureHalfDuplexLink(UPSTREAM,OUTBOUND);
        UPSTREAM_DIST_USART.CTRLA = USART_TXCINTLVL_OFF_gc;
        UPSTREAM_DIST_USART.DATA = outbound_upstream_data[0];
          //_delay_us(10);
          DELAY_ROUTINE();
        UPSTREAM_DIST_USART.STATUS = USART_TXCIF_bm;
        UPSTREAM_DIST_USART.CTRLA = USART_TXCINTLVL_LO_gc;
          activateDMAChannel(&UPSTREAM_DMA,3);
          daisyState = DAISY_TRANSMITTING;
          break;
        }

      if(((inbound_upstream_data[0] & 0xF0) == 0xF0) || ((inbound_upstream_data[0] & 0x0F) == 0x00))
        {
          //This packet should be responded to
          packetSize = processCommand((uint8_t *)inbound_upstream_data,(uint8_t *)outbound_upstream_data);

          //Assume the command processor handled the packetization
          configureHalfDuplexLink(UPSTREAM,OUTBOUND);
        UPSTREAM_DIST_USART.CTRLA = USART_TXCINTLVL_OFF_gc;
        UPSTREAM_DIST_USART.DATA = outbound_upstream_data[0];

          //_delay_us(10);
          DELAY_ROUTINE();
        UPSTREAM_DIST_USART.STATUS = USART_TXCIF_bm;
        UPSTREAM_DIST_USART.CTRLA = USART_TXCINTLVL_LO_gc;
          activateDMAChannel(&UPSTREAM_DMA,packetSize-1);
          daisyState = DAISY_TRANSMITTING;
        }
    }
      break;

    case DAISY_TRANSMITTING:
      //System is pushing data upstream, but it might be done
      if(upstreamTxDone)
    {
      //The upstream transmitter is loaded with data (but possibly not done)

      cli();
      upstreamTxDone = 0;
      sei();
      daisyState = DAISY_IDLE;

      configureHalfDuplexLink(UPSTREAM,INBOUND);
    }
      break;

    default:
      break;
    }
}
