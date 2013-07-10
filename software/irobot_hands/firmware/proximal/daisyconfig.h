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



#define UPSTREAM_USART USARTE0
#define UPSTREAM_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTE0_RXC_gc
#define UPSTREAM_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTE0_DRE_gc
#define UPSTREAM_USART_RX_vect USARTE0_RXC_vect
#define UPSTREAM_USART_TXDONE_vect USARTE0_TXC_vect
#define UPSTREAM_DMA DMA.CH0
#define UPSTREAM_USART_DMA_vect DMA_CH0_vect

#define DAISY_TC TCC0
#define DAISY_TC_vect TCC0_OVF_vect

#define DOWNSTREAM_USART USARTD0
#define DOWNSTREAM_USART_INBOUND_TRIGGER DMA_CH_TRIGSRC_USARTD0_RXC_gc
#define DOWNSTREAM_USART_OUTBOUND_TRIGGER DMA_CH_TRIGSRC_USARTD0_DRE_gc
#define DOWNSTREAM_USART_TXDONE_vect USARTD0_TXC_vect
#define DOWNSTREAM_USART_RX_vect USARTD0_RXC_vect

#warning "On the A4U chips, the header file is wrong for using DMA channel structs.  Use the address directly for now and check when the ASF is updated"

#define DOWNSTREAM_DMA (*(DMA_CH_t *) 0x0120)
#define DOWNSTREAM_USART_DMA_vect DMA_CH1_vect

#endif /* C1482_SRC_COMMON_0_DAISYCONFIG_H_ */
