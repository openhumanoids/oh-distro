/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0_adc.c
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <avr/pgmspace.h>
#include <math.h>

//static void calibrateADC(void);
static uint8_t ReadCalibrationByte(uint8_t index);

//static int16_t ADCCurrentCalibration;



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

// /************************************************************************
// * calibrateCurrentADC
// *
// * Assumes that the motor is inactive.
// * Captures a few samples of the ADC, averages them, and stores an
// * appropriate offset estimation for calibration
// ************************************************************************/
// /*
// static void calibrateADC(void)
// {
//     int i=0;
//     int32_t calibrationAccumulator=0;
//     while(i<16)
//     {
//         if(ADCA.INTFLAGS & ADC_CH1IF_bm)
//         {
//             ADCA.INTFLAGS = ADC_CH1IF_bm;
//             calibrationAccumulator += ADCA.CH1RES;
//             i++;
//         }
//     }
//     ADCCurrentCalibration = round((float) calibrationAccumulator / 16.0);
//     return;
// }
// */

void captureSweep(int16_t *outputData)
{
    //Begin sampling on the Distal
    ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
    ADCA_CH1_MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
    ADCA_CH2_MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
    ADCA_CH3_MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;

    asm("nop"); // delay for mux switch
    asm("nop");

    ADCA.CTRLA = ADC_CH3START_bm | ADC_CH2START_bm | ADC_CH1START_bm | ADC_CH0START_bm | ADC_ENABLE_bm;

    //Now wait for the conclusion of the sampling
    while(!(ADCA.INTFLAGS & ADC_CH3IF_bm));
    while(!(ADCA.INTFLAGS & ADC_CH2IF_bm));
    while(!(ADCA.INTFLAGS & ADC_CH1IF_bm));
    while(!(ADCA.INTFLAGS & ADC_CH0IF_bm));

    //Sampling complete
    outputData[0] = ADCA.CH0RES;
    outputData[1] = ADCA.CH1RES;
    outputData[2] = ADCA.CH2RES;
    outputData[3] = ADCA.CH3RES;

    ADCA.INTFLAGS = ADC_CH3IF_bm | ADC_CH2IF_bm | ADC_CH1IF_bm | ADC_CH0IF_bm;

    // //Begin sampling on the Dynamic
    // ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN5_gc;
    // ADCA_CH1_MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
    // ADCA_CH2_MUXCTRL = ADC_CH_MUXPOS_PIN7_gc;

    // asm("nop"); // delay for mux switch
    // asm("nop");

    // ADCA.CTRLA = ADC_CH2START_bm | ADC_CH1START_bm | ADC_CH0START_bm | ADC_ENABLE_bm;

    // //Now wait for the conclusion of the sampling
    // while(!(ADCA.INTFLAGS & ADC_CH2IF_bm));
    // while(!(ADCA.INTFLAGS & ADC_CH1IF_bm));
    // while(!(ADCA.INTFLAGS & ADC_CH0IF_bm));

    // //Sampling complete
    // outputData[4] = ADCA.CH0RES;
    // outputData[5] = ADCA.CH1RES;
    // outputData[6] = ADCA.CH2RES;

    // ADCA.INTFLAGS = ADC_CH2IF_bm | ADC_CH1IF_bm | ADC_CH0IF_bm;

    return;
}

/************************************************************************
* configureADC
*
* Configure the ADC to capture the four analog inputs we care about
* in free-running sweep mode with no interrupts.
* When the PID loop is evaluated, the most recent acquisition is placed into
* a buffer for later use.
*
* Note that an internal reference is being used in this test setup.
* The production system will have an external one available.
************************************************************************/
void configureADC(void)
{
    ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
    ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );

    //CURRENT_ADC.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVACT_NONE_gc;

    //Use the Event System to actuate sweeps

    //EVSYS.CH3MUX = EVSYS_CHMUX_PRESCALER_8192_gc;
    //EVSYS.CH3CTRL = EVSYS_DIGFILT_1SAMPLE_gc;

    //ADCA.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_3456_gc | ADC_EVACT_SWEEP_gc;

    //Maximum ADC clock is 2 MHz.  This requires a prescaler of 16 on a CPU clock of 32 MHz
    ADCA.PRESCALER = ADC_PRESCALER_DIV16_gc;
    //CURRENT_ADC.PRESCALER = ADC_PRESCALER_DIV256_gc;

#ifdef USE_INTERNAL_REF
    ADCA.REFCTRL = ADC_REFSEL_VCC_gc;
#else
    ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;
#endif

#warning The ADC.CH structure is wrong in AU parts.  Just use the ADC_CH0_MUXCTRL, etc. defines instead

    //CH0 is Distal 1 and Dynamic 1
    ADCA_CH0_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
    ADCA_CH0_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //CH1 is Distal 2 and Dynamic 2
    ADCA_CH1_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA_CH1_MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
    ADCA_CH1_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //CH2 is Distal 3 and Dynamic 3
    ADCA_CH2_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA_CH2_MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
    ADCA_CH2_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //CH3 is Distal 4 only
    ADCA_CH3_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA_CH3_MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
    ADCA_CH3_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //Configure for signed operation
    ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm;
    ADCA.CTRLA = ADC_ENABLE_bm;

    //Calibrate the ADC
    //calibrateADC();

    //Just use manual triggering for now
    //ADC_TC.CTRLB = TC_WGMODE_NORMAL_gc;
    //ADC_TC.CTRLC = 0x00;
    //ADC_TC.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
    //ADC_TC.CTRLE = 0x00;
    //ADC_TC.PERBUF = 3125;
    //ADC_TC.INTCTRLA = TC_OVFINTLVL_LO_gc;
    //ADC_TC.CTRLA = TC_CLKSEL_DIV1024_gc;

}
