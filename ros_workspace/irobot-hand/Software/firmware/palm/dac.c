/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PLMMCU-0_dac.c
 // Creation Date:    9 March, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    DAC driver

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

#include <avr/io.h>

void configureDAC(void)
{
    DACB.TIMCTRL = DAC_CONINTVAL_32CLK_gc | DAC_REFRESH_OFF_gc;
    DACB.CTRLC = DAC_REFSEL_AREFA_gc;
    DACB.CTRLB = DAC_CHSEL_SINGLE_gc; // | DAC_CH0TRIG_bm;
    DACB.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
    
    // This is set again in handleFingerCommand()
    // before the motor is enabled, but set it here just in case.
    
    //DACB.CH0DATA = 0x0FFF;
    DACB.CH0DATA = 0x0000;
}
