/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0.h
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Main header file for Proximal/Distal boards

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_PXDMCU_0_H_
#define C1482_SRC_PXDMCU_0_H_

#include <stdint.h>

#define F_CPU 32000000

#include <util/delay.h>

extern uint8_t numPressureSensors;

void LEDon(void);
void LEDoff(void);
void LEDtoggle(void);

#endif /* C1482_SRC_PXDMCU_0_H_ */
