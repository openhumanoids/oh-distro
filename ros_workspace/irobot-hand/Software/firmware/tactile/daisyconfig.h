/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-COMMON-0-daisyconfig.h
 // Creation Date:    22 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Project-specific configuration for
 //                    common routines for daisy chaining

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
 * This module defines all of the configuration required for daisy chaining
 * It should be modified within each device of the ARM-H hardware to support
 * that particular device.
 ************************************************************************/

#ifndef C1482_SRC_COMMON_0_DAISYCONFIG_H_
#define C1482_SRC_COMMON_0_DAISYCONFIG_H_

#include <avr/io.h>



#define UPSTREAM_USART USARTC0
#define UPSTREAM_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTC0_RXC_gc
#define UPSTREAM_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTC0_DRE_gc
#define UPSTREAM_USART_RX_vect USARTC0_RXC_vect
#define UPSTREAM_USART_TXDONE_vect USARTC0_TXC_vect
#define UPSTREAM_DMA DMA.CH0
#define UPSTREAM_USART_DMA_vect DMA_CH0_vect

#define SET_UPSTREAM_OUTBOUND() PORTC.OUT=PORTC.OUT|0x02   // always set bit 1
#define SET_UPSTREAM_INBOUND() PORTC.OUT=PORTC.OUT&(~0x01) // always clear bit 0

#define USE_DOWNSTREAM 0

#define DAISY_TC TCC0
#define DAISY_TC_vect TCC0_OVF_vect

/*


#define DOWNSTREAM_USART USARTD1
#define DOWNSTREAM_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTD1_RXC_gc
#define DOWNSTREAM_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTD1_DRE_gc
#define DOWNSTREAM_USART_RX_vect USARTD1_RXC_vect
#define DOWNSTREAM_DMA DMA.CH1
#define DOWNSTREAM_USART_DMA_vect DMA_CH1_vect

#define SET_DOWNSTREAM_OUTBOUND() PORTD.OUT=PORTD.OUT|0x30
#define SET_DOWNSTREAM_INBOUND() PORTD.OUT=PORTD.OUT&(~0x30)
*/

#endif /* C1482_SRC_COMMON_0_DAISYCONFIG_H_ */
