/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-TACMCU-0_adc.h
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    ADC driver

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_TACMCU_0_ADC_H_
#define C1482_SRC_TACMCU_0_ADC_H_

void configureADC(void);
void captureSweep(int16_t *outputData);

#endif /* C1482_SRC_TACMCU_0_ADC_H_ */