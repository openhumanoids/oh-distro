/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PLMMCU-0_adc.c
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
00            03/06/12    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <string.h>

#include "adc.h"

static uint8_t ReadCalibrationByte(uint8_t index);

//static int16_t IMON_ADCOffsetCal;

/************************************************************************
* ReadCalibrationByte
*
* Library code from Atmel for reading the ADC calibration byte from the
* production signature row.  This is required for the ADC to achieve
* its full specification.
************************************************************************/
static uint8_t ReadCalibrationByte(uint8_t index)
{
    uint8_t result;

    // Load the NVM Command register to read the calibration row.
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    result = pgm_read_byte(index);

    //Clean up NVM Command register.
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;

    return result;
}

float Thermistor_RtoT(float MeasR)
{
    float T;

    T = log(MeasR / THERMISTOR_NOMINAL_R);
    T *= (1/THERMISTOR_B);
    T += (1/THERMISTOR_NOMINAL_T);
    T = (1/T);
    T -= 273.15;         //Convert from K to C

    return T;

}

float Thermistor_VtoR(float ThermistorVoltage)
{
    float ThermistorR;

    ThermistorR = ThermistorVoltage * THERMISTOR_FIXED_R;
    ThermistorR /= (THERMISTOR_PULLUP_VOLTAGE - ThermistorVoltage);

    return ThermistorR;

}

// /************************************************************************
// * calibrateIMON_ADCOffset()
// *
// * Captures a few samples of the motor current monitor to remove steady
// * state offsets
// *
// * The monitor should sit at midrange positive, or 1024
// ************************************************************************/
// void calibrateIMON_ADCOffset(void)
// {
//     int i=0;
//     int32_t calibrationAccumulator=0;
//     int16_t error;
//     int16_t reading;

//     ADCA_CH3_MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;

//     for(i=0;i<16;i++)
//     {
//         ADCA.CTRLA = (ADC_CH3START_bm | ADC_ENABLE_bm);

//         while(!(ADCA.INTFLAGS & ADC_CH3IF_bm));

//         ADCA.INTFLAGS = ADC_CH3IF_bm;
//         reading = ADCA.CH3RES;
//         error = reading - 1024;
//         calibrationAccumulator += error;
//     }
//     IMON_ADCOffsetCal = round((float) calibrationAccumulator / 16.0);
//     return;
// }

/************************************************************************
* captureSweep(int16_t *outputData)
*
* captures an ADC sweep of all connected sensors and places the output
* data in outputData.
************************************************************************/
void captureSweep(int16_t *outputData)
{
    //Begin sampling on the first wave
    ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
    ADCA_CH1_MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
    ADCA_CH2_MUXCTRL = ADC_CH_MUXPOS_PIN7_gc;
    //ADCA_CH3_MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
    ADCA_CH3_MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
    
    ADCA.CTRLA = (ADC_CH3START_bm | ADC_CH2START_bm | ADC_CH1START_bm | ADC_CH0START_bm | ADC_ENABLE_bm);

    //Now wait for the conclusion of the sampling
    while(!(ADCA.INTFLAGS & ADC_CH3IF_bm));
    while(!(ADCA.INTFLAGS & ADC_CH2IF_bm));
    while(!(ADCA.INTFLAGS & ADC_CH1IF_bm));
    while(!(ADCA.INTFLAGS & ADC_CH0IF_bm));

    //Sampling complete
    outputData[0] = ADCA.CH0RES;
    outputData[1] = ADCA.CH1RES;
    outputData[2] = ADCA.CH2RES;
    outputData[3] = 0;
    outputData[4] = ADCA.CH3RES;
    outputData[5] = 0;

    //This channel should be calibrated
    //outputData[3] = ADCA.CH3RES - IMON_ADCOffsetCal;

    ADCA.INTFLAGS = ADC_CH3IF_bm | ADC_CH2IF_bm | ADC_CH1IF_bm | ADC_CH0IF_bm;

    // //Begin sampling on the remaining
    // //ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
    // //ADCA_CH1_MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;

    // ADCA.CTRLA = (ADC_CH1START_bm | ADC_CH0START_bm | ADC_ENABLE_bm);

    // //Now wait for the conclusion of the sampling
    // while(!(ADCA.INTFLAGS & ADC_CH1IF_bm));
    // while(!(ADCA.INTFLAGS & ADC_CH0IF_bm));

    // //Sampling complete
    // outputData[4] = ADCA.CH0RES;
    // outputData[5] = ADCA.CH1RES;

    // ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;

    return;
}

/************************************************************************
* configureADC
*
* Configure the ADC to capture the analog inputs using the captureSweep routine
************************************************************************/
void configureADC(void)
{
    ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
    ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );


    //Maximum ADC clock is 2 MHz.  This requires a prescaler of 16 on a CPU clock of 32 MHz
    ADCA.PRESCALER = ADC_PRESCALER_DIV16_gc;

    ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;

#warning The ADC.CH structure is wrong in AU parts.  Just use the ADC_CH0_MUXCTRL, etc. defines instead

    //CH0 is 3.3VMON and EXTTMP_MON
    ADCA_CH0_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
    ADCA_CH0_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //CH1 is 12VMON and TempB5
    ADCA_CH1_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA_CH1_MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
    ADCA_CH1_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //CH2 is 48VMON
    ADCA_CH2_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA_CH2_MUXCTRL = ADC_CH_MUXPOS_PIN7_gc;
    ADCA_CH2_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //CH3 is IMONB5
    ADCA_CH3_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA_CH3_MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
    ADCA_CH3_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //Configure for Unsigned operation
    ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm;
    ADCA.CTRLA = ADC_ENABLE_bm;

}
