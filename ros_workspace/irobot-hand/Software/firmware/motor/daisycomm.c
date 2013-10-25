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

#include "../common/daisycomm.h"
#include "daisyconfig.h"

//#define DELAY_ROUTINE() asm("nop")
#define DELAY_ROUTINE() _delay_us(10);

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

volatile uint8_t notifyTC = 0;

static volatile uint8_t inbound_upstream_data[COMMAND_PACKET_SIZE];
static volatile uint8_t outbound_upstream_data[MAX_PACKET_SIZE];


static volatile SERIAL_DIRECTION_t UPSTREAM_CONFIG = INBOUND;

static volatile int upstreamRxDone = 0;
static volatile int upstreamTxDone = 0;
static volatile uint8_t upstreamBusy = 0;
uint16_t RxCheckSumErrCnt[2];


static void configureUSARTHardware(USART_t *targetUSART, int isPC, int isDownstream);
static void prepareDMAChannel(DMA_CH_t *targetChannel, uint8_t usartTrigger, USART_t *targetUSART, volatile uint8_t *targetBuffer, uint8_t isOutbound);
static void activateDMAChannel(DMA_CH_t *targetChannel,int numBytes);
static void configureHalfDuplexLink(SERIAL_STREAM_t selectedStream, SERIAL_DIRECTION_t selectedDirection);
void handleTC(void);


#if USE_DOWNSTREAM
static void passthroughToUpstream(void);
//static volatile uint8_t downstreamXmitDone = 0;
static volatile uint8_t downstreamSizeReceived = 0;
static volatile SERIAL_DIRECTION_t DOWNSTREAM_CONFIG = OUTBOUND;
static volatile uint8_t inbound_downstream_data[MAX_PACKET_SIZE];
static volatile uint8_t outbound_downstream_data[COMMAND_PACKET_SIZE];

//This bit is set when a passthrough is to be expected from downstream
//It is cleared when the passed-through packet reply has been returned or timed out.
static volatile uint8_t passthroughWaiting = 0;
static volatile uint8_t downstreamBusy = 0;
static volatile uint8_t downstreamTimedout = 0;
static volatile uint8_t downstreamBusyCounter = 0;
#endif

#if USE_DOWNSTREAM
/************************************************************************
 * ISRs for the Downstream USART
 *
 * Only an RX ISRs is required on Downstream. This reads in the "packet size" field header
 * and arms the DMA engines to capture the amount of data to follow.
 *
 * The Upstream USART only receives fixed COMMAND_PACKET_SIZE packets, so it simply needs an
 * armed DMA channel
 ************************************************************************/
ISR(DOWNSTREAM_USART_RX_vect)
{
    uint8_t packetSize;
    //Status bits must be polled before reading the data or they are invalidated
    if(DOWNSTREAM_USART.STATUS & (USART_FERR_bm | USART_BUFOVF_bm))
    {
        //Framing or overflow error. Discard and do nothing else
        packetSize = DOWNSTREAM_USART.DATA;
        return;
    }

    //Packetsize is defined as the number of additional bytes to arrive
    //The buffer is of size MAX_PACKET_SIZE, so packetSize can be up to
    //MAX_PACKET_SIZE - 1.  If it is equal to MAX_PACKET_SIZE or greater, reject

    packetSize = DOWNSTREAM_USART.DATA;
    if((packetSize >= MAX_PACKET_SIZE) || (packetSize < MIN_PACKET_SIZE))
    {
        //Packet size is invalid somehow.  Reject it
        return;
    }

    //Packet size looks good.  Arm DMA

    prepareDMAChannel(&DOWNSTREAM_DMA,DOWNSTREAM_USART_INBOUND_TRIGGER,&DOWNSTREAM_USART,inbound_downstream_data+1,0);
    activateDMAChannel(&DOWNSTREAM_DMA,packetSize);
    inbound_downstream_data[0] = packetSize;

    //Disable all interrupts and let the DMA take command.
    DOWNSTREAM_USART.CTRLA = 0x00;

    downstreamBusy = 0;

    //Packet size looks good.  Arm DMA
    //activateDMAChannel(&DOWNSTREAM_DMA,packetSize);
    //inbound_downstream_data[0] = packetSize;

    //Disable the RX interrupt for now and let the DMA take control.
    //DOWNSTREAM_USART.CTRLA &= ~USART_RXCINTLVL_OFF_gc;            //Clear the interrupt (assumes that OFF is zeroing it out)
}
#endif

#define DOWNSTREAM_TIMEOUT_MS 15
#define DOWNSTREAM_OUTBOUND_TIMEOUT_MS 5

ISR(DAISY_TC_vect)
{
  notifyTC=1;
}

void handleTC(void)
{
  if(notifyTC) // set by the timer interrupt
    {
      notifyTC=0; // clear the flag

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
	      prepareDMAChannel(&UPSTREAM_DMA, UPSTREAM_USART_INBOUND_TRIGGER, &UPSTREAM_USART, inbound_upstream_data,0);
	      activateDMAChannel(&UPSTREAM_DMA,COMMAND_PACKET_SIZE);
	    }
	}
#if USE_DOWNSTREAM
      //Check if downstream might be stalled
      if(DOWNSTREAM_CONFIG == OUTBOUND)
	{
	  if (downstreamBusyCounter++ > (2 * DOWNSTREAM_OUTBOUND_TIMEOUT_MS)) //loop is 0.5 ms
	    {
	      notifyDaisy = 1;
	      downstreamTimedout = 1;
	      //downstreamBusy = 0;
	      passthroughWaiting = 0;
	      downstreamSizeReceived = 0;
	    }
	  
	  //No need
	  downstreamBusy = 0;
	}
      else
	{
	  downstreamBusyCounter = 0;
	  
	  //if((DOWNSTREAM_DMA.CTRLB & DMA_CH_CHBUSY_bm) || (passthroughWaiting == 1))
	  //{
	  //    downstreamBusy++;
	  //}
	  
	  if(downstreamBusy++ > (2 * DOWNSTREAM_TIMEOUT_MS)) //loop is 0.5 ms
	    {
	      //Notify the daisy chain task so that the upstream can potentially be fixed up.
	      notifyDaisy = 1;
	      downstreamTimedout = 1;
	      downstreamBusy = 0;
	      passthroughWaiting = 0;
	      //downstreamXmitDone = 0;
	      downstreamSizeReceived = 0;
	    }
	}
#endif
    }
}

/************************************************************************
 * ISRs for the DMA engines.
 * Set flags for the Daisy Chain Task
 ************************************************************************/
ISR(UPSTREAM_USART_DMA_vect)
{
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
        //if(UPSTREAM_CONFIG == OUTBOUND)
        //{
        //    upstreamTxDone = 1;
        //} else
        //{
            upstreamRxDone = 1;
        //}

    }
}

ISR(UPSTREAM_USART_TXDONE_vect)
{
    upstreamBusy = 0;
    notifyDaisy = 1;
    upstreamTxDone = 1;
    UPSTREAM_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_OFF_gc;
    UPSTREAM_USART.STATUS = USART_TXCIF_bm;
}

#if USE_DOWNSTREAM

ISR(DOWNSTREAM_USART_TXDONE_vect)
{
    //prepareDMAChannel(&DOWNSTREAM_DMA, DOWNSTREAM_USART_INBOUND_TRIGGER, &DOWNSTREAM_USART, inbound_downstream_data,0);
    DOWNSTREAM_CONFIG = INBOUND;
    //Shut down the TXC interrupt
    //DOWNSTREAM_USART.CTRLA &= ~USART_TXCINTLVL_gm;
    DOWNSTREAM_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_LO_gc;
    downstreamBusy = 0;
}

/************************************************************************
 * Downstream DMA vector
 * This is substantially more automated than the Upstream task
 * When a OUTBOUND transfer finishes, enable the RX interrupt to capture the reply.
 * When an INBOUND transfer finishes, switch back to outbound mode and notify
 * userspace.
 ************************************************************************/
ISR(DOWNSTREAM_USART_DMA_vect)
{
    if(DOWNSTREAM_DMA.CTRLB & DMA_CH_ERRIF_bm)
    {
        //Acknowledge the error
        //It may be from an aborted transfer, so just return.
        DOWNSTREAM_DMA.CTRLB = DOWNSTREAM_DMA.CTRLB | DMA_CH_ERRIF_bm;
        return;
    }

    if(DOWNSTREAM_DMA.CTRLB & DMA_CH_TRNIF_bm)
    {
        //The transfer is complete and should be acked
        DOWNSTREAM_DMA.CTRLB = DOWNSTREAM_DMA.CTRLB | DMA_CH_TRNIF_bm;
        //
        //if(DOWNSTREAM_CONFIG == INBOUND)
        //{
            //Just finished receiving response into singleDownstreamBuffer.  Signal userspace
            //The number of bytes received is the size field plus 1 (to account for the size field itself)
            downstreamSizeReceived = inbound_downstream_data[0] + 1;
            notifyDaisy = 1;
            return;
        //} else
        //{
            //downstreamXmitDone = 1;
        //    DOWNSTREAM_USART.CTRLA |= USART_TXCINTLVL_LO_gc;
        //    return;
        //}

        /*
        switch(DOWNSTREAM_CONFIG)
        {
            case OUTBOUND:
                DOWNSTREAM_USART.CTRLA |= USART_RXCINTLVL_LO_gc;
                prepareDMAChannel(&DOWNSTREAM_DMA, DOWNSTREAM_USART_INBOUND_TRIGGER, &DOWNSTREAM_USART, inbound_downstream_data,1);
                DOWNSTREAM_CONFIG = INBOUND;
                break;

            case INBOUND:
                prepareDMAChannel(&DOWNSTREAM_DMA, DOWNSTREAM_USART_OUTBOUND_TRIGGER, &DOWNSTREAM_USART, outbound_downstream_data,0);
                DOWNSTREAM_CONFIG = INBOUND;
                downstreamXmitDone = 1;
                break;
        }
        */
    }
}
#endif

#if USE_DOWNSTREAM
/************************************************************************
 * passthroughToUpstream()
 *
 * Pass the data directly from the downstream INBOUND buffer to the
 * upstream OUTBOUND buffer.  Send a corrupt checksum error if necessary
 ************************************************************************/
static void passthroughToUpstream(void)
{
    //It is assumed that the upstream DMA is able to accept additional data at this point
    //A data packet has arrived from downstream.  Validate it
    //The volatile keyword can be discarded because the DMA routines are inactive while this function executes
    if(computeChecksum((uint8_t *)inbound_downstream_data,downstreamSizeReceived) != 0x00)
    {
        //Invalid checksum
      RxCheckSumErrCnt[1]++;
        outbound_upstream_data[0] = 3; //Packet size
        outbound_upstream_data[1] = inbound_upstream_data[1]; //Reflected command byte
        outbound_upstream_data[2] = CHECKSUM_ERROR; //Checksum error
        outbound_upstream_data[3] = computeChecksum((uint8_t *)outbound_upstream_data,3); //checksum
        //configureHalfDuplexLink(UPSTREAM,OUTBOUND);
        UPSTREAM_USART.CTRLA = USART_TXCINTLVL_OFF_gc;
        UPSTREAM_USART.DATA = outbound_upstream_data[0];
        //_delay_us(10);
        DELAY_ROUTINE();
        UPSTREAM_USART.STATUS = USART_TXCIF_bm;
        UPSTREAM_USART.CTRLA = USART_TXCINTLVL_LO_gc;
        prepareDMAChannel(&UPSTREAM_DMA, UPSTREAM_USART_OUTBOUND_TRIGGER, &UPSTREAM_USART, outbound_upstream_data+1,1);
        activateDMAChannel(&UPSTREAM_DMA,3);
        downstreamSizeReceived = 0;
        return;
    }
    memcpy((uint8_t *)outbound_upstream_data,(uint8_t *)inbound_downstream_data,downstreamSizeReceived);
    //for(int i=0;i<inbound_downstream_data[0];i++)
    //{
    //    outbound_upstream_data[i] = inbound_downstream_data[i];
    //}
    //configureHalfDuplexLink(UPSTREAM,OUTBOUND);
    UPSTREAM_USART.CTRLA = USART_TXCINTLVL_OFF_gc;
    UPSTREAM_USART.DATA = outbound_upstream_data[0];
    //_delay_us(10);
    DELAY_ROUTINE();
    UPSTREAM_USART.STATUS = USART_TXCIF_bm;
    UPSTREAM_USART.CTRLA = USART_TXCINTLVL_LO_gc;
    prepareDMAChannel(&UPSTREAM_DMA, UPSTREAM_USART_OUTBOUND_TRIGGER, &UPSTREAM_USART, outbound_upstream_data+1,1);
    activateDMAChannel(&UPSTREAM_DMA,downstreamSizeReceived-1);
    downstreamSizeReceived = 0;
    return;
}
#endif

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
#if USE_DOWNSTREAM
        case DOWNSTREAM:
            //Abort any pending DMA transfers
            if(DOWNSTREAM_DMA.CTRLB & DMA_CH_CHBUSY_bm)
            {
                DOWNSTREAM_DMA.CTRLA &= ~DMA_CH_ENABLE_bm;
            }

            //downstreamXmitDone = 0;
            downstreamSizeReceived = 0;
            downstreamBusy = 0;

            //Configure the transceivers and prepare DMA
            if(selectedDirection == INBOUND)
            {
                prepareDMAChannel(&DOWNSTREAM_DMA, DOWNSTREAM_USART_INBOUND_TRIGGER, &DOWNSTREAM_USART, inbound_downstream_data,0);
                DOWNSTREAM_USART.CTRLA |= USART_RXCINTLVL_LO_gc;
                DOWNSTREAM_CONFIG = INBOUND;
            } else {
                DOWNSTREAM_USART.STATUS = USART_TXCIF_bm;
                prepareDMAChannel(&DOWNSTREAM_DMA, DOWNSTREAM_USART_OUTBOUND_TRIGGER, &DOWNSTREAM_USART, outbound_downstream_data,1);

                DOWNSTREAM_CONFIG = OUTBOUND;
            }
            break;
#endif
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
                prepareDMAChannel(&UPSTREAM_DMA, UPSTREAM_USART_INBOUND_TRIGGER, &UPSTREAM_USART, inbound_upstream_data,0);
                activateDMAChannel(&UPSTREAM_DMA,COMMAND_PACKET_SIZE);
                UPSTREAM_CONFIG = INBOUND;

            } else {
                UPSTREAM_USART.STATUS = USART_TXCIF_bm;
                UPSTREAM_USART.CTRLA = USART_TXCINTLVL_LO_gc | USART_RXCINTLVL_OFF_gc;
                //Do not send all data through DMA.  Instead send outbound upstream data + 1.  This allows for the insertion of a small delay between size and data
                prepareDMAChannel(&UPSTREAM_DMA, UPSTREAM_USART_OUTBOUND_TRIGGER, &UPSTREAM_USART, outbound_upstream_data + 1,1);
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
    configureUSARTHardware(&UPSTREAM_USART,0,0);
#if USE_DOWNSTREAM
    //Now initialize the downstream
    configureUSARTHardware(&DOWNSTREAM_USART,0,1);
#endif

    //Prepare DMA transfers
    DMA.CTRL = DMA_ENABLE_bm;

    //Assume default configuration with downstream OUT and upstream IN
#if USE_DOWNSTREAM
    configureHalfDuplexLink(DOWNSTREAM,OUTBOUND);
#endif
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
    uint8_t packetSize;
    //First clear the daisy notification flag
    cli();
    notifyDaisy = 0;
    sei();

    //This is set to IDLE when the upstream port is idle and TRANSMITTING when it is in use
    static DAISY_STATE_t daisyState = DAISY_IDLE;
/*
#if USE_DOWNSTREAM
    //First handle all events that are independent of the upstream state for the Downstream side
    if(downstreamXmitDone)
    {
        cli();
        downstreamXmitDone = 0;
        downstreamBusy = 0;
        sei();
        while(!(DOWNSTREAM_USART.STATUS & USART_TXCIF_bm));
        configureHalfDuplexLink(DOWNSTREAM,INBOUND);
    }
#endif
*/

    switch(daisyState)
    {
        case DAISY_IDLE:
#if USE_DOWNSTREAM
            //The daisy chain module is not transmitting, though a passthrough may be waiting

            if(downstreamSizeReceived && passthroughWaiting)
            {
                //Since a downstream reply was heard, pass it through to upstream
                passthroughWaiting = 0;
                daisyState = DAISY_TRANSMITTING;
                if(UPSTREAM_CONFIG == INBOUND)
                {
                    configureHalfDuplexLink(UPSTREAM,OUTBOUND);
                }
                passthroughToUpstream();

                cli();
                downstreamTimedout = 0;
                sei();

            } else if(downstreamSizeReceived)
            {
                //Error condition.  Just clear it out and restart
                configureHalfDuplexLink(UPSTREAM,INBOUND);
                //activateDMAChannel(&UPSTREAM_DMA,COMMAND_PACKET_SIZE);

                configureHalfDuplexLink(DOWNSTREAM,OUTBOUND);

                cli();
                downstreamTimedout = 0;
                sei();

            }

            if(downstreamTimedout)
            {
                cli();
                downstreamTimedout = 0;
                sei();

                //Cancel waiting for a downstream
                DOWNSTREAM_DMA.CTRLA = 0x00;
                //Wait for it to disable
                while(DOWNSTREAM_DMA.CTRLA & DMA_CH_ENABLE_bm);

                //Issue a reset
                DOWNSTREAM_DMA.CTRLA = DMA_CH_RESET_bm;

                //Now set back to outbound and wait
                prepareDMAChannel(&DOWNSTREAM_DMA, DOWNSTREAM_USART_OUTBOUND_TRIGGER, &DOWNSTREAM_USART, outbound_downstream_data,1);
                DOWNSTREAM_USART.STATUS = USART_TXCIF_bm;
                DOWNSTREAM_CONFIG = OUTBOUND;
                DOWNSTREAM_USART.CTRLA = 0x00;

                //Make sure Upstream is facing the proper direction
                if(UPSTREAM_CONFIG == OUTBOUND)
                {
                    //while(!(UPSTREAM_USART.STATUS & USART_TXCIF_bm));
                    configureHalfDuplexLink(UPSTREAM,INBOUND);
                    //activateDMAChannel(&UPSTREAM_DMA,COMMAND_PACKET_SIZE);
                }
            }


#endif

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
                    UPSTREAM_USART.CTRLA = USART_TXCINTLVL_OFF_gc;
                    UPSTREAM_USART.DATA = outbound_upstream_data[0];
                    //_delay_us(10);
                    DELAY_ROUTINE();
                    UPSTREAM_USART.STATUS = USART_TXCIF_bm;
                    UPSTREAM_USART.CTRLA = USART_TXCINTLVL_LO_gc;
                    activateDMAChannel(&UPSTREAM_DMA,3);
                    daisyState = DAISY_TRANSMITTING;
#if USE_DOWNSTREAM
                    //No passthrough occurred, so do not wait on a downstream reply.
                    passthroughWaiting = 0;
#endif
                    break;
                }
#if USE_DOWNSTREAM
                if((inbound_upstream_data[0] & 0x0F) != 0)
                {
                    //This packet is going downstream

                    memcpy((uint8_t *)outbound_downstream_data,(uint8_t *)inbound_upstream_data,COMMAND_PACKET_SIZE);

                    //Adjust the Destination Header and the checksum for the new destination
                    outbound_downstream_data[0]--;
                    outbound_downstream_data[CHECKSUM_OFFSET]++;
                    configureHalfDuplexLink(DOWNSTREAM,OUTBOUND);
                    activateDMAChannel(&DOWNSTREAM_DMA,COMMAND_PACKET_SIZE);

                    //A downstream reply will be expected
                    passthroughWaiting = 1;
                }
#endif

                if(((inbound_upstream_data[0] & 0xF0) == 0xF0) || ((inbound_upstream_data[0] & 0x0F) == 0x00))
                {
                    //PORTC.OUT |= 0x80; //BA: turn on LED
                    //This packet should be responded to
                    packetSize = processCommand((uint8_t *)inbound_upstream_data,(uint8_t *)outbound_upstream_data);

                    //Assume the command processor handled the packetization
                    configureHalfDuplexLink(UPSTREAM,OUTBOUND);
                    UPSTREAM_USART.CTRLA = USART_TXCINTLVL_OFF_gc;
                    UPSTREAM_USART.DATA = outbound_upstream_data[0];
                    //_delay_us(10);
                    DELAY_ROUTINE();
                    UPSTREAM_USART.STATUS = USART_TXCIF_bm;
                    UPSTREAM_USART.CTRLA = USART_TXCINTLVL_LO_gc;
                    activateDMAChannel(&UPSTREAM_DMA,packetSize-1);
                    daisyState = DAISY_TRANSMITTING;
                    //PORTC.OUT &= ~0x80; //BA: turn off LED
                }
            }
            break;

        case DAISY_TRANSMITTING:
#if USE_DOWNSTREAM
            if(downstreamTimedout)
            {
                cli();
                downstreamTimedout = 0;
                sei();
                //Downstream timed out, so just finish transmitting upstream and take no further action
                //Cancel waiting for a downstream
                DOWNSTREAM_DMA.CTRLA = 0x00;
                //Wait for it to disable
                while(DOWNSTREAM_DMA.CTRLA & DMA_CH_ENABLE_bm);

                //Issue a reset
                DOWNSTREAM_DMA.CTRLA = DMA_CH_RESET_bm;

                //Now set back to outbound and wait
                prepareDMAChannel(&DOWNSTREAM_DMA, DOWNSTREAM_USART_OUTBOUND_TRIGGER, &DOWNSTREAM_USART, outbound_downstream_data,1);
                DOWNSTREAM_USART.STATUS = USART_TXCIF_bm;
                DOWNSTREAM_CONFIG = OUTBOUND;
                DOWNSTREAM_USART.CTRLA = 0x00;
            }
#endif
            //System is pushing data upstream, but it might be done
            if(upstreamTxDone)
            {
                //The upstream transmitter is loaded with data (but possibly not done)

                cli();
                upstreamTxDone = 0;
                sei();
#if USE_DOWNSTREAM
                if(downstreamSizeReceived && passthroughWaiting)
                {
                    //The passthrough packet has already arrived.  Just send it upstream
                    passthroughWaiting = 0;
                    daisyState = DAISY_TRANSMITTING;
                    passthroughToUpstream();
                }
                else if(passthroughWaiting)
                {
                    //Just go back to idle.  Another notification will arrive to wake up when the downstream reply is delivered
                    daisyState = DAISY_IDLE;
                } else {
                    //Nothing else is coming  Reset to idle after full transmission complete
#endif
                    daisyState = DAISY_IDLE;
                    //Now wait for the transmission to complete entirely
                    //while(!(UPSTREAM_USART.STATUS & USART_TXCIF_bm));
                    configureHalfDuplexLink(UPSTREAM,INBOUND);
                    //activateDMAChannel(&UPSTREAM_DMA,COMMAND_PACKET_SIZE);

#if USE_DOWNSTREAM
                }
#endif
            }
            break;

        default:
            break;
    }

}
