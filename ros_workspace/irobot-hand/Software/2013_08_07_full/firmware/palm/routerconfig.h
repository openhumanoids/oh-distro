/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-COMMON-0-routerconfig.h
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
00            03/06/12    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/



#ifndef C1482_SRC_PLMMCU_0_ROUTERCONFIG_H_
#define C1482_SRC_PLMMCU_0_ROUTERCONFIG_H_

#define TACT_USART USARTC0
#define TACT_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTC0_RXC_gc
#define TACT_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTC0_DRE_gc
#define TACT_USART_RX_vect USARTC0_RXC_vect
#define TACT_USART_TXDONE_vect USARTC0_TXC_vect

#define SET_TACT_OUTBOUND() PORTC.OUT=PORTC.OUT|0x02
#define SET_TACT_INBOUND() PORTC.OUT=PORTC.OUT&(~0x01)

#define PROX1_USART USARTD1
#define PROX1_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTD1_RXC_gc
#define PROX1_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTD1_DRE_gc
#define PROX1_USART_RX_vect USARTD1_RXC_vect
#define PROX1_USART_TXDONE_vect USARTD1_TXC_vect

#define SET_PROX1_OUTBOUND() PORTD.OUTSET=0x20
#define SET_PROX1_INBOUND() PORTD.OUTCLR=0x10

#define PROX2_USART USARTC1
#define PROX2_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTC1_RXC_gc
#define PROX2_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTC1_DRE_gc
#define PROX2_USART_RX_vect USARTC1_RXC_vect
#define PROX2_USART_TXDONE_vect USARTC1_TXC_vect

#define SET_PROX2_OUTBOUND() PORTC.OUT=PORTC.OUT|0x20
#define SET_PROX2_INBOUND() PORTC.OUT=PORTC.OUT&(~0x10)

#define PROX3_USART USARTD0
#define PROX3_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTD0_RXC_gc
#define PROX3_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTD0_DRE_gc
#define PROX3_USART_RX_vect USARTD0_RXC_vect
#define PROX3_USART_TXDONE_vect USARTD0_TXC_vect

#define SET_PROX3_OUTBOUND() PORTD.OUTSET=0x02
#define SET_PROX3_INBOUND() PORTD.OUTCLR=0x01

#define MTR1_USART USARTE0
#define MTR1_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTE0_RXC_gc
#define MTR1_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTE0_DRE_gc
#define MTR1_USART_RX_vect USARTE0_RXC_vect
#define MTR1_USART_TXDONE_vect USARTE0_TXC_vect

#define SET_MTR1_OUTBOUND() PORTE.OUT=PORTE.OUT|0x02
#define SET_MTR1_INBOUND() PORTE.OUT=PORTE.OUT&(~0x01)

#define MTR2_USART USARTE1
#define MTR2_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTE1_RXC_gc
#define MTR2_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTE1_DRE_gc
#define MTR2_USART_RX_vect USARTE1_RXC_vect
#define MTR2_USART_TXDONE_vect USARTE1_TXC_vect

#define SET_MTR2_OUTBOUND() PORTE.OUT=PORTE.OUT|0x20
#define SET_MTR2_INBOUND() PORTE.OUT=PORTE.OUT&(~0x10)

#define UPSTREAM_USART USARTF0
#define UPSTREAM_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTF0_RXC_gc
#define UPSTREAM_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTF0_DRE_gc
#define UPSTREAM_USART_RX_vect USARTF0_RXC_vect
#define UPSTREAM_USART_TXDONE_vect USARTF0_TXC_vect


#define DOWNSTREAM0_DMA DMA.CH0
#define DOWNSTREAM0_USART_DMA_vect DMA_CH0_vect

#define DOWNSTREAM1_DMA DMA.CH1
#define DOWNSTREAM1_USART_DMA_vect DMA_CH1_vect

#define DOWNSTREAM2_DMA DMA.CH2
#define DOWNSTREAM2_USART_DMA_vect DMA_CH2_vect

#define DOWNSTREAM3_DMA DMA.CH3
#define DOWNSTREAM3_USART_DMA_vect DMA_CH3_vect

#define ROUTER_TC TCD0
#define ROUTER_TC_vect TCD0_OVF_vect

#endif /* C1482_SRC_PLMMCU_0_ROUTERCONFIG_H_ */
