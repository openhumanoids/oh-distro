/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PLMMCU-0_encoder.h
 // Creation Date:    5 March, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Encoder sensor driver

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

#ifndef C1492_SRC_PLMMCU_0_ENCODER_H_
#define C1492_SRC_PLMMCU_0_ENCODER_H_

uint16_t readEncoder(void);

extern uint8_t encoderInitialized;
extern int16_t adjustedEncoder;
extern int16_t lastEncoder;
extern int16_t rawEncoder;

#endif /* C1492_SRC_PLMMCU_0_ENCODER_H_ */
