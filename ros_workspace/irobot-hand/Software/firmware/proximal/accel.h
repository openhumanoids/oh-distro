/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0_tactsense.c
 // Creation Date:    5 March, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Accelerometer sensor driver

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_PXDMCU_0_ACCEL_H_
#define C1482_SRC_PXDMCU_0_ACCEL_H_

void configureSPIModulesAccel(void);
void readAxes(uint8_t *dataOut);
void configAccel(void);



#endif /* C1482_SRC_PXDMCU_0_ACCEL_H_ */