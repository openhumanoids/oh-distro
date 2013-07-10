/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-COMMON-0-router.c
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Packet router for Palm processor

 ****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00        03/06/12    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

#include "palm.h"
#include "router.h"
#include "routerconfig.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#define MAX_TOTAL_PACKET_SIZE 605
#define MAX_DEVICE_PACKET_SIZE 200
#define MIN_PACKET_SIZE 3

//Identifiers for the serial chains in this device
typedef enum SERIAL_STREAM_enum
    {
        TACT=0,
        PROX1=1,
        PROX2=2,
        PROX3=3,
        MTR1=4,
        MTR2=5,
        UPSTREAM=6,
        STREAM_NONE=7
    } SERIAL_STREAM_t;

//Configuration struct for a DMA channel or intterupt based downstream channel
//A different set of pointers map USART Streams back to the Configuraton 
typedef struct
{
    USART_t * usart;
    DMA_CH_t *dma;
    volatile uint8_t *targetBuffer;
    uint8_t inboundTrigger;
    volatile uint8_t sizeReceived;
    volatile uint8_t downstreamDoneFlag;
    SERIAL_STREAM_t activeStream;
    uint8_t isFirst;
    uint8_t isActive;
} CHANNEL_CONFIGURATION_t;

//The six Downstream configurations (4DME, 2 intterrupt)
static CHANNEL_CONFIGURATION_t DownStreamConfig[6];

//Double (ping-pong) buffer for transmission upstream to Overo
static volatile uint8_t responseToOveroA[MAX_TOTAL_PACKET_SIZE];
static volatile uint8_t responseToOveroB[MAX_TOTAL_PACKET_SIZE];

//Side buffer for directed messages
static volatile uint8_t responseToOveroC[MAX_TOTAL_PACKET_SIZE];

//Freerun related.  These capture the sample period,
//the sensors to be sampled, and whether a freerun is ready to go
uint16_t samplePeriod = 0xFFFF;
uint16_t sampleArgument = 0xFFFF;
uint8_t activePingPong = 0;

//Flag for notifying main() that a packet needs processing
volatile uint8_t commandReady = 0;

//Single buffer for reception downstream from Overo
static volatile uint8_t commandPacketBuffer[COMMAND_PACKET_SIZE];

//Two buffers for transmitting modified commands downstream to daisy chains
//See the ICD for details, but the broadcasts must be altered depending on chain length
static volatile uint8_t shortXmitBuffer[COMMAND_PACKET_SIZE];
static volatile uint8_t longXmitBuffer[COMMAND_PACKET_SIZE];

//Buffers for each slave device to be loaded with their responses to a broadcast.
static volatile uint16_t respondingDevicesBitfield;
static volatile uint8_t mtr1_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t mtr1_packet_size = 0;
static volatile uint8_t mtr2_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t mtr2_packet_size = 0;
static volatile uint8_t mtr3_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t mtr3_packet_size = 0;
static volatile uint8_t mtr4_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t mtr4_packet_size = 0;
static volatile uint8_t prox1_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t prox1_packet_size = 0;
static volatile uint8_t prox2_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t prox2_packet_size = 0;
static volatile uint8_t prox3_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t prox3_packet_size = 0;
static volatile uint8_t distal1_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t distal1_packet_size = 0;
static volatile uint8_t distal2_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t distal2_packet_size = 0;
static volatile uint8_t distal3_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t distal3_packet_size = 0;
static volatile uint8_t tactile_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t tactile_packet_size = 0;
static volatile uint8_t palm_packet[MAX_DEVICE_PACKET_SIZE];
static volatile uint8_t palm_packet_size = 0;

//Create constant tables for indexing the various buffers in the system.  This simplifies packet routing later
static USART_t * const allUSARTS[] = {&TACT_USART,&PROX1_USART,&PROX2_USART,&PROX3_USART,&MTR1_USART,&MTR2_USART};
// these are set to defaults, then set correctly at "startChain()"
static CHANNEL_CONFIGURATION_t *USARTtoCHANNELS[] = {&DownStreamConfig[0],&DownStreamConfig[1],&DownStreamConfig[2],&DownStreamConfig[3],&DownStreamConfig[4],&DownStreamConfig[5]};

static volatile uint8_t * const allFirstSlaveBuffers[] = {tactile_packet,prox1_packet,prox2_packet,prox3_packet,mtr1_packet,mtr3_packet};
static volatile uint8_t * const allFirstSlaveSizes[] = {&tactile_packet_size,&prox1_packet_size,&prox2_packet_size,&prox3_packet_size,&mtr1_packet_size,&mtr3_packet_size};
static volatile uint8_t * const allSecondSlaveBuffers[] = {NULL,distal1_packet,distal2_packet,distal3_packet,mtr2_packet,mtr4_packet};
static volatile uint8_t * const allSecondSlaveSizes[] = {NULL,&distal1_packet_size,&distal2_packet_size,&distal3_packet_size,&mtr2_packet_size,&mtr4_packet_size};

//Static tables capturing all inbound and outbound DMA Triggers in SERIAL_STREAM_t order
static const uint8_t allInboundTriggers[] = {TACT_USART_INBOUND_TRIGGER,
                                             PROX1_USART_INBOUND_TRIGGER,PROX2_USART_INBOUND_TRIGGER,PROX3_USART_INBOUND_TRIGGER,
                                             MTR1_USART_INBOUND_TRIGGER,MTR2_USART_INBOUND_TRIGGER};
static const uint8_t allOutboundTriggers[] = {TACT_USART_OUTBOUND_TRIGGER,
                                              PROX1_USART_OUTBOUND_TRIGGER,PROX2_USART_OUTBOUND_TRIGGER,PROX3_USART_OUTBOUND_TRIGGER,
                                              MTR1_USART_OUTBOUND_TRIGGER,MTR2_USART_OUTBOUND_TRIGGER};

#define DOWNSTREAM_TIMEOUT_4KHZ 5 // DMA and inerrupt transfers up and back down
#define UPSTREAM_TIMEOUT_4KHZ 32  // slower baud rate, bigger combined packet

static volatile uint8_t mtRXbyte[2] = {0,0};
//Timeout counters for Broadcast RX
static volatile uint8_t dmaTimeoutCounter[6] = {0,0,0,0,0,0};
static volatile uint8_t upRXtimeoutCounter = 0;

static void upstreamTX(volatile uint8_t *targetBuffer, uint16_t numBytes);
static void configureUSARTHardware(USART_t *targetUSART, int isPC, int isDownstream);
static inline void prepareDMAChannel(DMA_CH_t *targetChannel, uint8_t usartTrigger, USART_t *targetUSART, volatile uint8_t *targetBuffer, uint8_t isOutbound);
static inline void activateDMAChannel(DMA_CH_t *targetChannel,int numBytes);
static inline void doDownstreamDMAISR(CHANNEL_CONFIGURATION_t *targetChannelConfig);

static inline void doDownstreamUSART_RX_ISR(CHANNEL_CONFIGURATION_t *targetChannelConfig);
void startChain(uint8_t chainIndex, uint8_t DMAchannel, uint8_t *outboundBuffer);
void startMtrChain(uint8_t chainIndex);
static void simplePassthrough(CHANNEL_CONFIGURATION_t *targetChannelConfig);
static uint8_t continueChain(CHANNEL_CONFIGURATION_t *targetChannelConfig);
void doBroadcast(volatile uint8_t *activeUpstreamBuffer, volatile uint8_t *commandPacketBuffer);

static volatile uint8_t Uerr_frame;
static volatile uint8_t Uerr_busy;
static volatile uint8_t Uerr_timeout;
static volatile uint8_t Uerr_checksum;
static volatile uint8_t Derr_frame;
static volatile uint8_t Derr_packetsize;

uint16_t RxCheckSumErrCnt[7];

uint8_t chainMask = ALL_CHAINS_CHAINMASK;

ISR(ROUTER_TC_vect)
{
    // timers count down to zero. When zero, you are out of time

    // four DMA channels
    if((DownStreamConfig[0].isActive) &&
       (dmaTimeoutCounter[0]))
    {
        dmaTimeoutCounter[0]--;
    }
    if((DownStreamConfig[1].isActive) &&
       (dmaTimeoutCounter[1]))
    {
        dmaTimeoutCounter[1]--;
    }
    if((DownStreamConfig[2].isActive) &&
       (dmaTimeoutCounter[2]))
    {
        dmaTimeoutCounter[2]--;
    }
    if((DownStreamConfig[3].isActive) &&
       (dmaTimeoutCounter[3]))
    {
        dmaTimeoutCounter[3]--;
    }

    // two interrupt based motor channels
    if((DownStreamConfig[4].isActive) &&
       (dmaTimeoutCounter[4]))
    {
        dmaTimeoutCounter[4]--;
    }
    if((DownStreamConfig[5].isActive) &&
       (dmaTimeoutCounter[5]))
    {
        dmaTimeoutCounter[5]--;
    }

    // upstream interrupt based channel
    if(upRXtimeoutCounter)
    {
        upRXtimeoutCounter--;
    }

    return;
}


/************************************************************************
 * ISR for the Upstream USART
 *
 * This is how commands enter the Palm board from the Overo Gumstix
 *
 ************************************************************************/

ISR(UPSTREAM_USART_RX_vect)
{
    static uint8_t upRXbyte = 0;
    uint8_t temporary;

    //Must check any error bits before reading the data.
    if(UPSTREAM_USART.STATUS & (USART_FERR_bm | USART_BUFOVF_bm))
    {
        //Framing or overflow error. Discard and do nothing else
        temporary = UPSTREAM_USART.DATA; // read to clear the buffer
        upRXbyte=0;
        upRXtimeoutCounter=UPSTREAM_TIMEOUT_4KHZ;
        Uerr_frame++;
        //if(UPSTREAM_USART.STATUS & USART_BUFOVF_bm)
        //  LEDtoggle();
        return;
    }

    temporary = UPSTREAM_USART.DATA; // read to clear the buffer

    if(commandReady)
    {
        //Don't clobber an existing packet, discard
        upRXbyte=0;
        upRXtimeoutCounter=UPSTREAM_TIMEOUT_4KHZ;
        Uerr_busy++;
        return;
    }

    if(!(upRXtimeoutCounter))
    {
        //there was a timeout, assume this is a new packet
        upRXbyte=0;
        upRXtimeoutCounter=UPSTREAM_TIMEOUT_4KHZ;
        Uerr_timeout++;
        //return; // if you want to discard and start over
    }

    commandPacketBuffer[upRXbyte] = temporary;
    upRXbyte++;

    if(upRXbyte==1)        // our first byte
    {
        // the first byte should contain a valid "Destination Header"
        // a bsic check for validity
        // chain address should be < 8, chain index < 2
        if(temporary & 0x8e)
        {
            upRXbyte=0; // discard it
            // leave the timer running
        }
        else
        {
            upRXtimeoutCounter=UPSTREAM_TIMEOUT_4KHZ;
        }
        return;
    }
    if(upRXbyte==COMMAND_PACKET_SIZE)
    {
        commandReady = 1; // signal done
        upRXbyte=0;       // get ready for the next one
        upRXtimeoutCounter=UPSTREAM_TIMEOUT_4KHZ;
        return;
    }
}

/************************************************************************
 * prepareDMAChannel()
 *
 * Configures selectedChannel (a DMA.CHX structure) of the DMA module
 * to collect data on usartTrigger signal produced by targetUSART and place
 * it into the buffer targetBuffer.  isOutbound should be set to 1 if
 * usartTrigger is an Outbound trigger
 *
 * This should be called before activateDMAChannel()
 ************************************************************************/
static inline void prepareDMAChannel(DMA_CH_t *selectedChannel, uint8_t usartTrigger, USART_t *targetUSART, volatile uint8_t *targetBuffer, uint8_t isOutbound)
{
    //Set to perform one block transfer corresponding to one command packet
    //Technically not necessary because REPEAT will not be set in CTRLA
    selectedChannel->REPCNT = 0x01;

    //Trigger on received data
    selectedChannel->TRIGSRC = usartTrigger;

    if(isOutbound)
    {
        //Configure the chosen channel to read from a fixed UART into a linear array
        selectedChannel->ADDRCTRL = DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc |
            DMA_CH_DESTRELOAD_BURST_gc | DMA_CH_DESTDIR_FIXED_gc;
        //This part has 16 bit pointers, so cast appropriately.
        //Memory addresses are 24 bits to allow for external memory to be mapped into one address space
        //Same with internal EEPROM Data memory.
        //Since these locations are in lower memory, fix the high order address bits to zero.
        //Mapping external memory will require more care.
        selectedChannel->DESTADDR0 = (uint16_t) &targetUSART->DATA & 0x00FF;
        selectedChannel->DESTADDR1 = (uint16_t) &targetUSART->DATA >> 8;
        selectedChannel->DESTADDR2 = 0;

        selectedChannel->SRCADDR0 = (uint16_t)targetBuffer & 0x00FF;
        selectedChannel->SRCADDR1 = (uint16_t)targetBuffer >> 8;
        selectedChannel->SRCADDR2 = 0;
        //Interrupt on this channel when the transaction is complete
        //selectedChannel->CTRLB = DMA_CH_TRNINTLVL_LO_gc;
        selectedChannel->CTRLB = DMA_CH_TRNINTLVL_OFF_gc | DMA_CH_TRNIF_bm;
        targetUSART->STATUS = USART_TXCIF_bm;
        targetUSART->CTRLA |= USART_TXCINTLVL_LO_gc;    }
    else
    {
        //Configure the chosen channel to read from a fixed UART into a linear array
        selectedChannel->ADDRCTRL = DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_FIXED_gc |
            DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc;

        selectedChannel->DESTADDR0 = (uint16_t)targetBuffer & 0x00FF;
        selectedChannel->DESTADDR1 = (uint16_t)targetBuffer >> 8;
        selectedChannel->DESTADDR2 = 0;

        selectedChannel->SRCADDR0 = (uint16_t) &targetUSART->DATA & 0x00FF;
        selectedChannel->SRCADDR1 = (uint16_t) &targetUSART->DATA >> 8;
        selectedChannel->SRCADDR2 = 0;
        //Interrupt on this channel when the transaction is complete
        //selectedChannel->CTRLB = DMA_CH_TRNINTLVL_LO_gc;
        selectedChannel->CTRLB = DMA_CH_TRNINTLVL_LO_gc | DMA_CH_TRNIF_bm;
    }
}

/************************************************************************
 * activateDMAChannel(selectedChannel,numBytes)
 *
 * Activates the DMA channel identified by selectedChannel (a DMA.CHX structure)
 * to move numBytes bytes.  The DMA channel must have been previously prepared
 * with prepareDMAChannel().
 ************************************************************************/
static inline void activateDMAChannel(DMA_CH_t *selectedChannel,int numBytes)
{
    //Set block size to packet size
    selectedChannel->TRFCNT = numBytes;
    //Enable the channel on single shot mode with a burst length of one.  Whenever the trigger arrives,
    //only one burst will be performed.
    selectedChannel->CTRLA = DMA_CH_BURSTLEN_1BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_ENABLE_bm;
}


/************************************************************************
 * ISRs for the Downstream USART RX interrupts (DMA setup)
 *
 * This routine takes the Configuration struct for the given USART.  It uses
 * the contents of that structure to properly load an assigned DMA channel to receive
 * the rest of the bytes to follow.
 ************************************************************************/
static inline void doDownstreamUSART_RX_ISR(CHANNEL_CONFIGURATION_t *targetChannelConfig)
{
    uint8_t packetSize;
    USART_t *targetUSART = targetChannelConfig->usart;

    //Status bits must be polled before reading the data or they are invalidated
    if(targetUSART->STATUS & (USART_FERR_bm | USART_BUFOVF_bm))
    {
        //Framing or overflow error. Discard and do nothing else
        Derr_frame++;
        packetSize = targetUSART->DATA;
        //if(targetUSART->STATUS & USART_FERR_bm)
        //if(targetUSART->STATUS & USART_BUFOVF_bm)
        //  LEDtoggle();
        return;
    }

    //Packetsize is defined as the number of additional bytes to arrive
    //The buffer is of size MAX_DEVICE_PACKET_SIZE, so packetSize can be up to
    //MAX_DEVICE_PACKET_SIZE - 1.  If it is equal to MAX_DEVICE_PACKET_SIZE or greater, reject

    packetSize = targetUSART->DATA;
    if((packetSize >= MAX_DEVICE_PACKET_SIZE) || (packetSize < MIN_PACKET_SIZE))
    {
        //Packet size is invalid somehow.  Reject it
        Derr_packetsize++;
        return;
    }

    //Packet size looks good.  Arm DMA
    prepareDMAChannel(targetChannelConfig->dma,targetChannelConfig->inboundTrigger,targetUSART,targetChannelConfig->targetBuffer+2,0);
    activateDMAChannel(targetChannelConfig->dma,packetSize);
    targetChannelConfig->targetBuffer[0] = packetSize;

    //Widen the packetSize field
    targetChannelConfig->targetBuffer[1] = 0;

    //Disable all interrupts and let the DMA take command.
    targetUSART->CTRLA = 0x00;
}

/************************************************************************
 * ISRs for the Motor USART RX interrupts (non-DMA)
 *
 * This routine takes the Configuration struct for the given USART.
 * It uses the contents of that structure to properly receive
 * the rest of the bytes to follow.
 ************************************************************************/
static inline void doMotorUSART_RX_ISR(CHANNEL_CONFIGURATION_t *targetChannelConfig, uint8_t mtr)
{
    static uint8_t packetSize = 0;
    uint8_t temporary;
    USART_t *targetUSART = targetChannelConfig->usart;

    //Must check any error bits before reading the data.
    if(targetUSART->STATUS & (USART_FERR_bm | USART_BUFOVF_bm))
    {
        //Framing or overflow error. Discard and do nothing else
        temporary = targetUSART->DATA;// read to clear the buffer
        mtRXbyte[mtr]=0;
        dmaTimeoutCounter[mtr+4]=DOWNSTREAM_TIMEOUT_4KHZ;
        //Uerr_frame++;
        return;
    }

    temporary = targetUSART->DATA;// read to clear the buffer

    if(!(dmaTimeoutCounter[mtr+4]))
    {
        //timeout, assume it is the first byte after an old packet timed out
        mtRXbyte[mtr]=0;
        dmaTimeoutCounter[mtr+4]=DOWNSTREAM_TIMEOUT_4KHZ;
        //ertrr_timeout++;
        //return; // return if we wish to discard this byte instead of treating it as a new packet
    }

    targetChannelConfig->targetBuffer[mtRXbyte[mtr]+1] = temporary;
    mtRXbyte[mtr]++;

    if(mtRXbyte[mtr]==1)        // our first byte
    {
        packetSize=temporary;
        targetChannelConfig->targetBuffer[0] = packetSize;
        targetChannelConfig->targetBuffer[1] = 0;
        if((packetSize >= MAX_DEVICE_PACKET_SIZE) || (packetSize < MIN_PACKET_SIZE))
        {
            //Packet size is invalid somehow.  Reject it
            //Derr_packetsize++;
            return;
        }
        dmaTimeoutCounter[mtr+4]=DOWNSTREAM_TIMEOUT_4KHZ;
        return;
    }
    if(mtRXbyte[mtr]==(packetSize+1))  // last byte
    {
        targetChannelConfig->sizeReceived = packetSize + 2;

        mtRXbyte[mtr]=0;       // get ready for the next packet
        dmaTimeoutCounter[mtr+4]=DOWNSTREAM_TIMEOUT_4KHZ;
        return;
    }
}


ISR(TACT_USART_RX_vect)
{
    doDownstreamUSART_RX_ISR(USARTtoCHANNELS[TACT]);
}

ISR(PROX1_USART_RX_vect)
{
    doDownstreamUSART_RX_ISR(USARTtoCHANNELS[PROX1]);
}

ISR(PROX2_USART_RX_vect)
{
    doDownstreamUSART_RX_ISR(USARTtoCHANNELS[PROX2]);
}

ISR(PROX3_USART_RX_vect)
{
    doDownstreamUSART_RX_ISR(USARTtoCHANNELS[PROX3]);
}

ISR(MTR1_USART_RX_vect)
{
    doMotorUSART_RX_ISR(USARTtoCHANNELS[MTR1],0);
}

ISR(MTR2_USART_RX_vect)
{
    doMotorUSART_RX_ISR(USARTtoCHANNELS[MTR2],1);
}

static inline void doDownstreamUSART_TXDONE_ISR(CHANNEL_CONFIGURATION_t *targetChannelConfig)
{
    // This interrupt is called when the DMA is done stuffing the TX buffer
    // and the last charachter has been sent.
    // Reconfigure to capture the response.
    targetChannelConfig->downstreamDoneFlag = 1;
    targetChannelConfig->usart->CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_MED_gc;
}

ISR(TACT_USART_TXDONE_vect)
{
    //TACT_USART.CTRLA |= USART_RXCINTLVL_MED_gc; //redundant?
    doDownstreamUSART_TXDONE_ISR(USARTtoCHANNELS[TACT]);
}

ISR(PROX1_USART_TXDONE_vect)
{
    //PROX1_USART.CTRLA |= USART_RXCINTLVL_MED_gc;
    doDownstreamUSART_TXDONE_ISR(USARTtoCHANNELS[PROX1]);
}

ISR(PROX2_USART_TXDONE_vect)
{
    //PROX2_USART.CTRLA |= USART_RXCINTLVL_MED_gc;
    doDownstreamUSART_TXDONE_ISR(USARTtoCHANNELS[PROX2]);
}

ISR(PROX3_USART_TXDONE_vect)
{
    //PROX3_USART.CTRLA |= USART_RXCINTLVL_MED_gc;
    doDownstreamUSART_TXDONE_ISR(USARTtoCHANNELS[PROX3]);
}

ISR(MTR1_USART_TXDONE_vect)
{
    //MTR1_USART.CTRLA |= USART_RXCINTLVL_MED_gc;
    doDownstreamUSART_TXDONE_ISR(USARTtoCHANNELS[MTR1]);
}

ISR(MTR2_USART_TXDONE_vect)
{
    //MTR2_USART.CTRLA |= USART_RXCINTLVL_MED_gc;
    doDownstreamUSART_TXDONE_ISR(USARTtoCHANNELS[MTR2]);
}

/************************************************************************
 * ISRs for the Upstream TX
 *
 ************************************************************************/
/*ISR(UPSTREAM_USART_TXDONE_vect)
  {
  //todo: transmit a filled buffer
  UPSTREAM_USART.STATUS = USART_TXCIF_bm;
  }
*/

/************************************************************************
 * Handler for downstream DMA ISRs
 * Use the CHANNEL_CONFIGURATION to identify whether the DMA was inbound
 * or outbound.  In each case, notify the correct userspace routine by either
 * setting sizeReceived or setting downstreamDone.
 ************************************************************************/
static inline void doDownstreamDMAISR(CHANNEL_CONFIGURATION_t *targetChannelConfig)
{
    DMA_CH_t *targetDMA = targetChannelConfig->dma;
    if(targetDMA->CTRLB & DMA_CH_ERRIF_bm)
    {
        //Acknowledge the DMA channel error
        targetDMA->CTRLB = targetDMA->CTRLB | DMA_CH_ERRIF_bm;
        //It may be from an aborted transfer, so just return.
        return;
    }

    if(targetDMA->CTRLB & DMA_CH_TRNIF_bm)
    {
        //The transfer is complete and should be acked
        targetDMA->CTRLB = targetDMA->CTRLB | DMA_CH_TRNIF_bm;

        targetChannelConfig->isActive = 0; // BA

        //Just finished receiving response into singleDownstreamBuffer.  Signal userspace
        //The number of bytes received is the size field plus 2 (to account for the size field itself and the padded packetSize
        targetChannelConfig->sizeReceived = targetChannelConfig->targetBuffer[0] + 2;
        return;
    }
}


/************************************************************************
 * Downstream DMA vectors
 ************************************************************************/
ISR(DOWNSTREAM0_USART_DMA_vect)
{
    doDownstreamDMAISR(&DownStreamConfig[0]);
}
ISR(DOWNSTREAM1_USART_DMA_vect)
{
    doDownstreamDMAISR(&DownStreamConfig[1]);
}
ISR(DOWNSTREAM2_USART_DMA_vect)
{
    doDownstreamDMAISR(&DownStreamConfig[2]);
}
ISR(DOWNSTREAM3_USART_DMA_vect)
{
    doDownstreamDMAISR(&DownStreamConfig[3]);
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
    if(isPC)
    {
        //COMMAND_USART.BAUDCTRLA = 34;    //Configure for 57600 baud rate with normal baud rate generation
        //COMMAND_USART.BAUDCTRLB = 0;

        //targetUSART->BAUDCTRLA = 33;    //Configure for 115200 with fractional baud rate generation
        //targetUSART->BAUDCTRLB = 0xF0;

        targetUSART->BAUDCTRLA = 41;    //Configure for 1219512.2 Baud for 1152000 Baud for PC
        targetUSART->BAUDCTRLB = 0xA0;    // BSEL = 41, BSCALE = -6 = 0xC in twos complement

        //targetUSART->BAUDCTRLA = 73;    //Configure for 931372 Baud  for 1000000 Baud for PC
        //targetUSART->BAUDCTRLB = 0xA0;    // BSEL = 73, BSCALE = -6 = 0xA in twos complement

        //targetUSART->BAUDCTRLA = 6;    //Configure for 1818181.8 Baud  for 2000000 Baud for PC
        //targetUSART->BAUDCTRLB = 0xA0;    // BSEL = 6, BSCALE = -6 = 0xA in twos complement
    }
    else
    {
        targetUSART->BAUDCTRLA = 0;    //Configure for 2 MBaud by setting baud rate to zero
        targetUSART->BAUDCTRLB = 0x00;    //and BSEL to zero
    }

    targetUSART->CTRLC = (USART_CMODE_ASYNCHRONOUS_gc
                          | USART_PMODE_DISABLED_gc
                          | USART_CHSIZE_8BIT_gc);    //Configure port settings for 8 bits of data, 1 stop bit.
    targetUSART->CTRLB |= USART_RXEN_bm | USART_TXEN_bm;    //enable RX and TX

    if(isDownstream)
    {
        targetUSART->CTRLA = USART_RXCINTLVL_MED_gc; //Enable RX interrupt at LO priority
    }
    else
    {
        targetUSART->CTRLA = USART_RXCINTLVL_HI_gc; //Enable RX interrupt at HI priority
    }

    return;
}


static void upstreamTX(volatile uint8_t *targetBuffer, uint16_t numBytes)
{
    // -pavlo todo make this an intterrupt driven process

    //UPSTREAM_USART.CTRLA &= ~USART_TXCINTLVL_LO_gc;
    for(uint16_t i=0; i<numBytes; i++)
    {
        while(!(UPSTREAM_USART.STATUS & USART_DREIF_bm));
        UPSTREAM_USART.DATA = targetBuffer[i];
        //if(!(UPSTREAM_USART.STATUS & USART_DREIF_bm))
        //while(UPSTREAM_USART.STATUS & USART_TXCIF_bm);
        //UPSTREAM_USART.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
    }
    //UPSTREAM_USART.CTRLA |= USART_TXCINTLVL_LO_gc;
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
 * configureRouterUSARTs()
 *
 * Configures the router system
 ************************************************************************/
void configureRouterUSARTs(void)
{
    // set the RX buffers to a reasonable value
    for(uint8_t i=0;i<6;i++)
        startMtrChain(i);

    //First initialize the upstream to the Overo or PC
    configureUSARTHardware(&UPSTREAM_USART,USE_PC_BITRATE,0);

    //Now initialize the various downstream units
    configureUSARTHardware(&TACT_USART,0,1);
    configureUSARTHardware(&PROX1_USART,0,1);
    configureUSARTHardware(&PROX2_USART,0,1);
    configureUSARTHardware(&PROX3_USART,0,1);
    configureUSARTHardware(&MTR1_USART,0,1);
    configureUSARTHardware(&MTR2_USART,0,1);

    //Enable DMA transfers, but do not prepare any for now.
    DMA.CTRL = DMA_ENABLE_bm;

    //Assume default configuration with downstream OUT and upstream IN
    SET_TACT_INBOUND();
    SET_PROX1_INBOUND();
    SET_PROX2_INBOUND();
    SET_PROX3_INBOUND();
    SET_MTR1_INBOUND();
    SET_MTR2_INBOUND();
    SET_TACT_OUTBOUND();
    SET_PROX1_OUTBOUND();
    SET_PROX2_OUTBOUND();
    SET_PROX3_OUTBOUND();
    SET_MTR1_OUTBOUND();
    SET_MTR2_OUTBOUND();

    //Setup the router timeout counter to about 0.25 ms periods
    // 32MHz/256/32: 4KHz (0.25ms)
    ROUTER_TC.CTRLB = TC_WGMODE_NORMAL_gc;
    ROUTER_TC.CTRLC = 0x00;
    ROUTER_TC.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
    ROUTER_TC.CTRLE = 0x00;
    ROUTER_TC.PERBUF = 32;
    ROUTER_TC.CNT = 0x0000;
    ROUTER_TC.INTCTRLA = TC_OVFINTLVL_LO_gc;
    ROUTER_TC.CTRLA = TC_CLKSEL_DIV256_gc;
}

static void waitDMAChain(CHANNEL_CONFIGURATION_t *targetChannelConfig)
{
    // The Transmit has been initated before this function was called
    // when the transmit DMA is done, downstreamDoneFlag is set
    while( (targetChannelConfig->downstreamDoneFlag == 0) );

    //Now the DMA is done, but the transmission is not yet done
    targetChannelConfig->downstreamDoneFlag = 0;
}

/************************************************************************
 * simplePassthrough()
 *
 * Uses a previously configured targetChannelConfig to send data down a
 * USART Stream.  It then waits (with timeout) for a reply from the
 * downstream device.  It then either passes the device back upstream or
 * generate an appropriate error response.
 ************************************************************************/
static void simplePassthrough(CHANNEL_CONFIGURATION_t *targetChannelConfig)
{
    // set up the receive timeout
    cli();
    upRXtimeoutCounter=UPSTREAM_TIMEOUT_4KHZ;
    sei();
    while((targetChannelConfig->sizeReceived == 0) && (upRXtimeoutCounter));

    if(!(upRXtimeoutCounter))
    {
        //Abort abort: // presumption of DMA1 being always used
        DOWNSTREAM1_DMA.CTRLA &= ~DMA_CH_ENABLE_bm;

        // NEW FROM ZACH

        DownStreamConfig[1].usart->CTRLA = 0x00;
        DownStreamConfig[1].isActive = 0; // BA

        responseToOveroC[0] = 3; //Packet size
        responseToOveroC[1] = 0; //Packet size MSB
        responseToOveroC[2] = commandPacketBuffer[COMMAND_OFFSET]; //Reflected command byte
        responseToOveroC[3] = TIMEOUT_ERROR; //Timeout error
        responseToOveroC[4] = computeChecksum((uint8_t *)responseToOveroC,4); //checksum
        upstreamTX(responseToOveroC, 5);
        return;
    }

    if(computeChecksum((uint8_t *)targetChannelConfig->targetBuffer,targetChannelConfig->sizeReceived) != 0x00)
    {
        //Invalid checksum
        RxCheckSumErrCnt[targetChannelConfig->activeStream]++;
        responseToOveroC[0] = 3; //Packet size
        responseToOveroC[1] = 0; //Packet size MSB
        responseToOveroC[2] = commandPacketBuffer[COMMAND_OFFSET]; //Reflected command byte
        responseToOveroC[3] = CHECKSUM_ERROR; //Checksum error
        responseToOveroC[4] = computeChecksum((uint8_t *)responseToOveroC,4); //checksum
        upstreamTX(responseToOveroC, 5);
        return;
    }

    //Data is ready for shipping upstream
    upstreamTX(targetChannelConfig->targetBuffer,targetChannelConfig->sizeReceived);
    return;
}

/************************************************************************
 * startChain()
 *
 * Begins a Stream transmission.  Used in both passthroughs and broadcasts
 ************************************************************************/
void startChain(uint8_t chainIndex, uint8_t DMAchannel, uint8_t *outboundBuffer)
{
    DMA_CH_t *targetDMA;
    CHANNEL_CONFIGURATION_t *targetChannelConfig;

    switch( DMAchannel)
    {
        case 0:
            targetDMA = &DOWNSTREAM0_DMA;
            targetChannelConfig = &DownStreamConfig[0];
            break;
        case 1:
            targetDMA = &DOWNSTREAM1_DMA;
            targetChannelConfig = &DownStreamConfig[1];
            break;
        case 2:
            targetDMA = &DOWNSTREAM2_DMA;
            targetChannelConfig = &DownStreamConfig[2];
            break;
        case 3:
            targetDMA = &DOWNSTREAM3_DMA;
            targetChannelConfig = &DownStreamConfig[3];
            break;
        default:
            // should record an error here, or block
            return;
    }

    //Configure the CHANNEL_CONFIGURATION_t structure
    targetChannelConfig->usart = allUSARTS[chainIndex];
    targetChannelConfig->sizeReceived = 0;
    targetChannelConfig->downstreamDoneFlag = 0;

    targetChannelConfig->dma = targetDMA;
    targetChannelConfig->targetBuffer = allFirstSlaveBuffers[chainIndex];
    targetChannelConfig->inboundTrigger = allInboundTriggers[chainIndex];
    targetChannelConfig->activeStream = chainIndex;
    targetChannelConfig->isFirst = 1;
    targetChannelConfig->isActive = 1;

    USARTtoCHANNELS[chainIndex] = targetChannelConfig;
    allUSARTS[chainIndex]->STATUS = USART_TXCIF_bm;

    prepareDMAChannel(targetDMA, allOutboundTriggers[chainIndex], allUSARTS[chainIndex], outboundBuffer,1);
    activateDMAChannel(targetDMA,COMMAND_PACKET_SIZE);

    cli();
    dmaTimeoutCounter[DMAchannel] = DOWNSTREAM_TIMEOUT_4KHZ;
    sei();
}

void startMtrChain(uint8_t chainIndex)
{
    CHANNEL_CONFIGURATION_t *targetChannelConfig;

    targetChannelConfig = &DownStreamConfig[chainIndex];

    //Configure the CHANNEL_CONFIGURATION_t structure
    targetChannelConfig->usart = allUSARTS[chainIndex];
    targetChannelConfig->sizeReceived = 0;
    targetChannelConfig->downstreamDoneFlag = 0;

    targetChannelConfig->targetBuffer = allFirstSlaveBuffers[chainIndex];
    targetChannelConfig->inboundTrigger = allInboundTriggers[chainIndex];
    targetChannelConfig->activeStream = chainIndex;
    targetChannelConfig->isFirst = 1;
    targetChannelConfig->isActive = 1;

    USARTtoCHANNELS[chainIndex] = targetChannelConfig;
    allUSARTS[chainIndex]->STATUS = USART_TXCIF_bm;

}

/************************************************************************
 * continueChain
 *
 * Returns 1 if the chain is already done and 0 if an additional reply is
 * coming.  In either case it will set up for the next reply or properly shut
 * down the stream.
 ************************************************************************/
static uint8_t continueChain(CHANNEL_CONFIGURATION_t *targetChannelConfig)
{
    SERIAL_STREAM_t activeStream = targetChannelConfig->activeStream;

    //Store the size received
    if(targetChannelConfig->isFirst)
    {
        *(allFirstSlaveSizes[activeStream]) = targetChannelConfig->sizeReceived;
    }
    else
    {
        *(allSecondSlaveSizes[activeStream]) = targetChannelConfig->sizeReceived;
    }

    //Clean out the structure
    targetChannelConfig->sizeReceived = 0;
    targetChannelConfig->downstreamDoneFlag = 0;

    if((allSecondSlaveBuffers[activeStream] == NULL) || (targetChannelConfig->isFirst == 0))
    {
        //No continuation.  Chain is done
        return 1;
    }

    //Another reply is expected.  Rearm the channel
    targetChannelConfig->isFirst = 0;
    targetChannelConfig->isActive = 1;
    targetChannelConfig->targetBuffer = allSecondSlaveBuffers[activeStream];
    USARTtoCHANNELS[activeStream] = targetChannelConfig;
    allUSARTS[activeStream]->STATUS = USART_TXCIF_bm;
    targetChannelConfig->usart->CTRLA |= USART_RXCINTLVL_MED_gc;
    return 0;

}

/************************************************************************
 * stuffOutput()
 *
 * Used in Broadcasts to stuff the response to the Overo based on the collected
 * data in the broadcast.  It confirms an appropriate size and checksum before
 * loading the data.
 ************************************************************************/
static inline uint8_t stuffOutput(uint8_t * outputBuffer, uint8_t *inputBuffer, uint8_t inputSize, uint16_t deviceBitmask)
{
    //inputSize is the number of bytes actually received (so the packetSize header plus 2)
#if 0
    if(   (inputSize >= 5)
          && (inputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] == STATUS_OK)
          && (computeChecksum(inputBuffer,inputSize) == 0))
    {
        memcpy(outputBuffer,&inputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM],inputSize-5);
        respondingDevicesBitfield |= deviceBitmask;
        return inputSize-5;
    }
    return 0;
#endif
    unsigned r = 0;
    uint16_t bitmsk = deviceBitmask;
    while (bitmsk >>= 1)  // find index from bitmask
    {
        r++;
    }

    if (inputSize >= 5) {
        if (inputBuffer[RESPONSE_STATUSCODE_OFFSET_PALM] == STATUS_OK) 
        {
            if (computeChecksum(inputBuffer,inputSize) == 0) 
            {
                // normal valid packet
                memcpy(outputBuffer,&inputBuffer[RESPONSE_PAYLOAD_OFFSET_PALM],inputSize-5);
                respondingDevicesBitfield |= deviceBitmask;
                return inputSize-5;
            }
            else {
                // bad checksum
                RxCheckSumErrCnt[r]++;
                return 0;
            }
        }
        else{
            // bad status
            return 0;
        }
    }
    else{
        // input size<5
        return 0;
    }

}

/************************************************************************
 * doFreerunTask()
 *
 * This is called from main() whenever the freerun timer indicates that
 * another broadcast should be performed.  It synthesizes a DATA_COLLECTION
 * command and passes it to the broadcast routine for delivery downstream
 ************************************************************************/
void doFreerunTask(void)
{
    uint8_t smallCommandPacketBuffer[COMMAND_PACKET_SIZE];

    smallCommandPacketBuffer[DESTINATION_HEADER_OFFSET] = DESTINATION_BROADCAST;
    smallCommandPacketBuffer[COMMAND_OFFSET] = DATA_COLLECTION_OPCODE;
    memcpy(&smallCommandPacketBuffer[PAYLOAD_OFFSET],&sampleArgument,2);
    smallCommandPacketBuffer[CHECKSUM_OFFSET] = computeChecksum(smallCommandPacketBuffer,6);

    if(activePingPong)
    {
        doBroadcast(responseToOveroB,smallCommandPacketBuffer);
        activePingPong = 0;
    }
    else
    {
        doBroadcast(responseToOveroA,smallCommandPacketBuffer);
        activePingPong = 1;
    }
    return;
}

uint8_t tendDMAchain(uint8_t i)
{
    if(DownStreamConfig[i].downstreamDoneFlag == 1)
    {
        // DMA is done transmitting,
        // automagically starts receiving, see doDownstreamUSART_TXDONE_ISR()
        // clear the flag, reset the timeout counter
        DownStreamConfig[i].downstreamDoneFlag = 0;
        cli();
        dmaTimeoutCounter[i] = DOWNSTREAM_TIMEOUT_4KHZ;
        sei();
    }
  
    if(DownStreamConfig[i].sizeReceived > 0)
    {
        //DMA is done receiving
        // continueChain() will zero out .sizeReceived[]
        // check if a second response is expected
        if(continueChain(&DownStreamConfig[i]))
        {
            //This chain is done.
            return(1);
        }
        else
        {
            // wait for the second response
            cli();
            dmaTimeoutCounter[i] = DOWNSTREAM_TIMEOUT_4KHZ;
            sei();
        }
    }
  
    if(!(dmaTimeoutCounter[i]))
    {
        //DMA timed out.  Reset it and mark done
        DownStreamConfig[i].isActive = 0;
        dmaTimeoutCounter[i] = DOWNSTREAM_TIMEOUT_4KHZ;
        switch(i)
        {
            case 0 :
                DOWNSTREAM0_DMA.CTRLA &= ~DMA_CH_ENABLE_bm;
                break;
            case 1 :
                DOWNSTREAM1_DMA.CTRLA &= ~DMA_CH_ENABLE_bm;
                break;
            case 2 :
                DOWNSTREAM2_DMA.CTRLA &= ~DMA_CH_ENABLE_bm;
                break;
            case 3 :
                DOWNSTREAM3_DMA.CTRLA &= ~DMA_CH_ENABLE_bm;
                break;
        }
        //Disable interrupts on the USART
        DownStreamConfig[i].usart->CTRLA = 0x00;
        return(1);
    }

    return(0);
}

uint8_t tendMTRchain(uint8_t i)
{
    // are the motors done?
    if(DownStreamConfig[i].sizeReceived > 0)
    {
        //preserve the length
        if(DownStreamConfig[i].isFirst)
        {
            *(allFirstSlaveSizes[i]) = DownStreamConfig[i].sizeReceived;
        }
        else
        {
            *(allSecondSlaveSizes[i]) = DownStreamConfig[i].sizeReceived;
        }
      
        //Clean out the structure
        DownStreamConfig[i].sizeReceived=0;
        DownStreamConfig[i].downstreamDoneFlag=0;
      
        //is another reply expected?
        if((allSecondSlaveBuffers[i] == NULL) || (DownStreamConfig[i].isFirst == 0))
        {
            //No continuation.  Chain is done
            DownStreamConfig[i].isActive = 0;
            return(1);
        }
        else
        {
            //Another reply is expected.  Rearm the channel
            DownStreamConfig[i].isFirst = 0;
            DownStreamConfig[i].isActive = 1;
            DownStreamConfig[i].targetBuffer = allSecondSlaveBuffers[i];
            USARTtoCHANNELS[i] = &DownStreamConfig[i];
            allUSARTS[i]->STATUS = USART_TXCIF_bm; // clear the transmission done bit
            //DownStreamConfig[i].usart->CTRLA |= USART_RXCINTLVL_MED_gc;
            DownStreamConfig[i].usart->CTRLA = USART_RXCINTLVL_LO_gc;
            cli();
            mtRXbyte[i-4]=0;       // get ready for start of packet
            dmaTimeoutCounter[i]=DOWNSTREAM_TIMEOUT_4KHZ;
            sei();
            return(0);
        }
    }
  
    if(!(dmaTimeoutCounter[i]))
    {
        DownStreamConfig[i].isActive = 0;
        dmaTimeoutCounter[i]=DOWNSTREAM_TIMEOUT_4KHZ;
        // timeout: mark chain done, move on.
        return(1);
    }
    return(0);
}

/************************************************************************
 * doBroadcast()
 *
 * Actually sends data downstream.
 * Three DMA engines are used to pass data to three active streams
 * When these finish or timeout, the DMA engines are re-allocated to the remaining
 * streams for transmission.  When all are finished, the final response to the Overo
 * is combined and delivered by Upstream DMA.  The response itself is ping-ponged
 * at a higher level.
 ************************************************************************/
void doBroadcast(volatile uint8_t *activeUpstreamBuffer, volatile uint8_t *commandPacketBuffer)
{
    uint16_t packetSize;
    uint8_t chainsDone = 0;

    //The packet is a broadcast.  Create the two different variants for transmission (to length 1 and length 2 chains)
    memcpy((uint8_t *)shortXmitBuffer,(uint8_t *)commandPacketBuffer,COMMAND_PACKET_SIZE);
    memcpy((uint8_t *)longXmitBuffer,(uint8_t *)commandPacketBuffer,COMMAND_PACKET_SIZE);

    shortXmitBuffer[DESTINATION_HEADER_OFFSET] = 0xF0;
    //Now fix the checksum
    shortXmitBuffer[CHECKSUM_OFFSET] += 0x0F;

    longXmitBuffer[DESTINATION_HEADER_OFFSET] = 0xF1;
    //Now fix the checksum
    longXmitBuffer[CHECKSUM_OFFSET] += 0x0E;

    //Now send down finger chains
    respondingDevicesBitfield = 0x0000;

    //Zero out the various flags
    for(int i=0;i<NUMCHAINS;i++)
    {
        *(allFirstSlaveSizes[i]) = 0;
        if(allSecondSlaveSizes[i] != NULL)
        {
            *(allSecondSlaveSizes[i]) = 0;
        }
        allUSARTS[i]->STATUS = USART_TXCIF_bm;
    }

#define NOT_PARALLEL 1
#ifdef NOT_PARALLEL
    // fire off all four DMA channels
    if (chainMask & TACTILE_CHAINMASK)
    {
        startChain(0,0,(uint8_t *)shortXmitBuffer); //TACT 0
        while(0==tendDMAchain(0));
    }
    if (chainMask & FINGER_1_CHAINMASK)
    {
        startChain(1,1,(uint8_t *)longXmitBuffer); //DIST1 1
        while(0==tendDMAchain(1));
    }
    if (chainMask & FINGER_2_CHAINMASK)
    {
        startChain(2,2,(uint8_t *)longXmitBuffer); //DIST2 2
        while(0==tendDMAchain(2));
    }
    if (chainMask & FINGER_3_CHAINMASK)
    {
        startChain(3,3,(uint8_t *)longXmitBuffer); //DIST3 3
        while(0==tendDMAchain(3));
    }
#else
    // fire off all four DMA channels
    startChain(0,0,(uint8_t *)shortXmitBuffer); //TACT 0
    startChain(1,1,(uint8_t *)longXmitBuffer); //DIST1 1
    startChain(2,2,(uint8_t *)longXmitBuffer); //DIST2 2
    startChain(3,3,(uint8_t *)longXmitBuffer); //DIST3 3
    //Loop through DMA channels and tend until done
    chainsDone = 0;
    while(chainsDone != 0x0f)
    {
        for(uint8_t i=0; i<4; i++)
            if(!(chainsDone & (1<<i))) // only check chains which are not yet done
                if(tendDMAchain(i))
                    chainsDone |= (1<<i);
    } // block until all four DMAs are done
#endif

#ifdef NOT_PARALLEL
    if (chainMask & MOTORS_1_2_CHAINMASK)
    {
        startMtrChain(4);
        for(int i=0;i<COMMAND_PACKET_SIZE;i++)
        {
            while(!(MTR1_USART.STATUS & USART_DREIF_bm));
            MTR1_USART.DATA = longXmitBuffer[i];
        }
        while(!(MTR1_USART.STATUS & USART_DREIF_bm)); // make sure the byte goes out
        chainsDone = MTR1_USART.DATA; // dummy read to clear the RXCIF bit
        MTR1_USART.STATUS = USART_RXCIF_bm;//flush any left over RX byte
        //MTR1_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_MED_gc;
        MTR1_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_LO_gc;
        cli();
        mtRXbyte[0]=0;       // get ready for start of packet
        dmaTimeoutCounter[4]=DOWNSTREAM_TIMEOUT_4KHZ;
        sei();
        while(0==tendMTRchain(4));
    }
    
    if (chainMask & MOTORS_3_4_CHAINMASK)
    {
        startMtrChain(5);
        for(int i=0;i<COMMAND_PACKET_SIZE;i++)
        {
            while(!(MTR2_USART.STATUS & USART_DREIF_bm));
            MTR2_USART.DATA = longXmitBuffer[i];
        }
        while(!(MTR2_USART.STATUS & USART_DREIF_bm)); // make sure the byte goes out
        chainsDone = MTR2_USART.DATA; // dummy read to clear the RXCIF bit
        MTR2_USART.STATUS = USART_RXCIF_bm;//flush any left over RX byte
        //MTR2_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_MED_gc;
        MTR2_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_LO_gc;
        cli();
        mtRXbyte[1]=0;       // get ready for start of packet
        dmaTimeoutCounter[5]=DOWNSTREAM_TIMEOUT_4KHZ;
        sei();
        while(0==tendMTRchain(5));
    }
    
#else
    startMtrChain(4);
    startMtrChain(5);
    for(int i=0;i<COMMAND_PACKET_SIZE;i++)
    {
        while(!(MTR1_USART.STATUS & USART_DREIF_bm));
        MTR1_USART.DATA = longXmitBuffer[i];
        while(!(MTR2_USART.STATUS & USART_DREIF_bm));
        MTR2_USART.DATA = longXmitBuffer[i];
    }
    cli();
    mtRXbyte[0]=0;       // get ready for start of packet
    dmaTimeoutCounter[4]=DOWNSTREAM_TIMEOUT_4KHZ;
    mtRXbyte[1]=0;       // get ready for start of packet
    dmaTimeoutCounter[5]=DOWNSTREAM_TIMEOUT_4KHZ;
    sei();
    while(!(MTR1_USART.STATUS & USART_DREIF_bm)); // make sure the byte goes out
    while(!(MTR2_USART.STATUS & USART_DREIF_bm)); // make sure the byte goes out
    chainsDone = MTR1_USART.DATA; // dummy read to clear the RXCIF bit
    MTR1_USART.STATUS = USART_RXCIF_bm;//flush any left over RX byte
    chainsDone = MTR2_USART.DATA; // dummy read to clear the RXCIF bit
    MTR2_USART.STATUS = USART_RXCIF_bm;//flush any left over RX byte
    MTR1_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_LO_gc;
    MTR2_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_LO_gc;

    chainsDone = 0;
    //Loop through all channels and make sure they are kept busy as they become done
    while(chainsDone != 0x30)
    {
        for(uint8_t i=4;i<6;i++)
            if(!(chainsDone & (1<<i))) // only check chains which are not yet done
                if(tendMTRchain(i))
                    chainsDone |= (1<<i);
    }
#endif

    //collect the data from the palm sensor
    palm_packet_size = processCommand((uint8_t *)commandPacketBuffer,(uint8_t *)palm_packet);

    //Now compute checksums and collect input

    packetSize = 0;
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)palm_packet, palm_packet_size, RESPONDING_DEVICES_PALM_BITMASK);

    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)prox1_packet, prox1_packet_size, RESPONDING_DEVICES_FIRST_PROX_BITMASK);
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)distal1_packet, distal1_packet_size, RESPONDING_DEVICES_FIRST_DIST_BITMASK);
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)prox2_packet, prox2_packet_size, RESPONDING_DEVICES_SECOND_PROX_BITMASK);
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)distal2_packet, distal2_packet_size, RESPONDING_DEVICES_SECOND_DIST_BITMASK);
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)prox3_packet, prox3_packet_size, RESPONDING_DEVICES_THIRD_PROX_BITMASK);
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)distal3_packet, distal3_packet_size, RESPONDING_DEVICES_THIRD_DIST_BITMASK);

    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)mtr1_packet, mtr1_packet_size, RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK);
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)mtr2_packet, mtr2_packet_size, RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK);
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)mtr3_packet, mtr3_packet_size, RESPONDING_DEVICES_SECOND_MOTOR1_BITMASK);
    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)mtr4_packet, mtr4_packet_size, RESPONDING_DEVICES_SECOND_MOTOR2_BITMASK);

    packetSize += stuffOutput((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PAYLOAD + packetSize], 
                              (uint8_t *)tactile_packet, tactile_packet_size, RESPONDING_DEVICES_TACTILE_BITMASK);

    //This area is where the packetSize is widened to 16 bits and other parts are pushed down appropriately.
    //Packetsize only contains payload bytes now.
    // plus 2 for the responding devices, plus 1 for the reflected opcode, plus 1 for the checksum
    packetSize += 4;

    memcpy((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_PACKETSIZE_LSB],&packetSize,2);

    activeUpstreamBuffer[RESPONSE_BROADCAST_REFLECTEDOPCODE] = commandPacketBuffer[COMMAND_OFFSET];
    memcpy((uint8_t *)&activeUpstreamBuffer[RESPONSE_BROADCAST_RESPONDINGDEVICES],(uint8_t *)&respondingDevicesBitfield,2);

    //Plus 1 required here to cover the two bytes of packetSize field
    activeUpstreamBuffer[packetSize+1] = computeChecksum((uint8_t *)activeUpstreamBuffer,packetSize+1);

    upstreamTX(activeUpstreamBuffer, packetSize+2); //this blocks till TX is complete

    return;
}

/************************************************************************
 * doRouterTask()
 *
 * Main routine of the Router process
 * Maintains system state and manages the transmission/reception of chained data
 * Should be called whenever commandReady is set to 1.
 ************************************************************************/

void doRouterTask(void)
{
    uint8_t packetSize;
    uint8_t dummy;

    //First clear the commandReady flag that got us here
    commandReady = 0;

    //A data packet has arrived from upstream.  Validate it
    if(computeChecksum((uint8_t *)commandPacketBuffer,COMMAND_PACKET_SIZE) != 0x00)
    {
        //Invalid checksum
        RxCheckSumErrCnt[6]++;
        responseToOveroC[0] = 3; //Packet size
        responseToOveroC[1] = 3; //Packet size MSB
        responseToOveroC[2] = commandPacketBuffer[COMMAND_OFFSET]; //Reflected command byte
        responseToOveroC[3] = CHECKSUM_ERROR; //Checksum error
        responseToOveroC[4] = computeChecksum((uint8_t *)responseToOveroC,4); //checksum
        upstreamTX(responseToOveroC, 5);
        return;
    }

    //Command is valid.  Identify the destination
    if(commandPacketBuffer[DESTINATION_HEADER_OFFSET] == DESTINATION_BROADCAST)
    {
        doBroadcast(responseToOveroC,commandPacketBuffer);
        return;
    }

    // set up the receive timeout
    cli();
    upRXtimeoutCounter=UPSTREAM_TIMEOUT_4KHZ;
    sei();

    switch(commandPacketBuffer[DESTINATION_HEADER_OFFSET] & CHAINADDRESS_BITMASK)
    {
        case PALM_CHAINADDRESS:
            packetSize = processCommand((uint8_t *)commandPacketBuffer,(uint8_t *)responseToOveroC);
            upstreamTX(responseToOveroC, packetSize);
            return;
        case TACTILE_CHAINADDRESS:
            startChain(TACT,1,(uint8_t *)commandPacketBuffer);
            waitDMAChain(&DownStreamConfig[1]);
            simplePassthrough(&DownStreamConfig[1]);
            break;
        case FINGER1_CHAINADDRESS:
            startChain(PROX1,1,(uint8_t *)commandPacketBuffer);
            waitDMAChain(&DownStreamConfig[1]);
            simplePassthrough(&DownStreamConfig[1]);
            break;
        case FINGER2_CHAINADDRESS:
            startChain(PROX2,1,(uint8_t *)commandPacketBuffer);
            waitDMAChain(&DownStreamConfig[1]);
            simplePassthrough(&DownStreamConfig[1]);
            break;
        case FINGER3_CHAINADDRESS:
            startChain(PROX3,1,(uint8_t *)commandPacketBuffer);
            waitDMAChain(&DownStreamConfig[1]);
            simplePassthrough(&DownStreamConfig[1]);
            break;
        case MOTOR1_CHAINADDRESS:
            //startChain(MTR1,1,(uint8_t *)commandPacketBuffer);
            startMtrChain(4);
            for(int i=0;i<COMMAND_PACKET_SIZE;i++)
            {
                while(!(MTR1_USART.STATUS & USART_DREIF_bm));
                MTR1_USART.DATA = commandPacketBuffer[i];
            }
            while(!(MTR1_USART.STATUS & USART_DREIF_bm)); // make sure the byte goes out
            dummy = MTR1_USART.DATA; // dummy read to clear the RXCIF bit
            MTR1_USART.STATUS = USART_RXCIF_bm;//flush any left over RX byte
            //MTR1_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_MED_gc;
            MTR1_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_LO_gc;
            simplePassthrough(&DownStreamConfig[4]);
            break;
        case MOTOR2_CHAINADDRESS:
            //startChain(MTR2,1,(uint8_t *)commandPacketBuffer);
            startMtrChain(5);
            for(int i=0;i<COMMAND_PACKET_SIZE;i++)
            {
                while(!(MTR2_USART.STATUS & USART_DREIF_bm));
                MTR2_USART.DATA = commandPacketBuffer[i];
            }
            while(!(MTR2_USART.STATUS & USART_DREIF_bm)); // make sure the byte goes out
            dummy = MTR2_USART.DATA; // dummy read to clear the RXCIF bit
            MTR2_USART.STATUS = USART_RXCIF_bm;//flush any left over RX byte
            //MTR2_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_MED_gc;
            MTR2_USART.CTRLA = USART_TXCINTLVL_OFF_gc | USART_RXCINTLVL_LO_gc;
            simplePassthrough(&DownStreamConfig[5]);
            break;
    }
}
