/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0_encoder.h
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

#ifndef C1492_SRC_PXDMCU_0_ENCODER_H_
#define C1492_SRC_PXDMCU_0_ENCODER_H_

int16_t readEncoder(void);
int16_t readRawEncoder(void);

extern int16_t encoderOffset;

#endif /* C1492_SRC_PXDMCU_0_ENCODER_H_ */
