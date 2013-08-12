/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-currentADC.c
 // Creation Date:    28 October, 2011
 // Revision:        01
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC ADC control module

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/28/11    PDB            Initial Release
01            12/20/11    ZAC            Added documentation
-------------------------------------------------------------------------------

******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <avr/pgmspace.h>
#include <math.h>

#include "currentADC.h"
#include "thermalModel.h"

//Private Functions
static void calibrateCurrentADC(void);
static uint8_t ReadCalibrationByte(uint8_t index);
static float compensateCurrent(float inCurrent);

static int16_t ADCCurrentCalibration;

//Global variable to hold the motor current and temperature itself.
float motorCurrent = 0.0;
int16_t rawMotorCurrent = 0;
float statorTemperature = 25.0;

uint16_t tension[2] = {0,0};

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

/************************************************************************
* calibrateCurrentADC
*
* Assumes that the motor is inactive.
* Captures a few samples of the ADC, averages them, and stores an
* appropriate offset estimation for calibration
************************************************************************/
static void calibrateCurrentADC(void)
{
    int i=0;
    int32_t calibrationAccumulator=0;
    while(i<16)
    {
        if(CURRENT_ADC.INTFLAGS & ADC_CH1IF_bm)
        {
            CURRENT_ADC.INTFLAGS = ADC_CH1IF_bm;
            calibrationAccumulator += CURRENT_ADC.CH1RES;
            i++;
        }
    }
    ADCCurrentCalibration = round((float) calibrationAccumulator / 16.0);
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
    CURRENT_ADC.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
    CURRENT_ADC.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );

    //Use the Event System to actuate sweeps

    EVSYS.CH3MUX = EVSYS_CHMUX_PRESCALER_8192_gc;
    EVSYS.CH3CTRL = EVSYS_DIGFILT_1SAMPLE_gc;

    CURRENT_ADC.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_3456_gc | ADC_EVACT_SWEEP_gc;

    //Maximum ADC clock is 2 MHz.  This requires a prescaler of 16 on a CPU clock of 32 MHz
    CURRENT_ADC.PRESCALER = ADC_PRESCALER_DIV16_gc;

#ifdef USE_INTERNAL_REF
    CURRENT_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
#else
    CURRENT_ADC.REFCTRL = ADC_REFSEL_AREFA_gc;
#endif

    #warning The ADC.CH structure is wrong in AU parts.  Just use the ADC_CH0_MUXCTRL, etc. defines instead
    //CH0 is the temperature
    CURRENT_ADC_CH0_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    CURRENT_ADC_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
    CURRENT_ADC_CH0_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //CH1 is the current monitor.  It arrives inverted due to limitations in the part

    CURRENT_ADC_CH1_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
    CURRENT_ADC_CH1_MUXCTRL = ADC_CH_MUXPOS_PIN5_gc | ADC_CH_MUXNEG_PIN2_gc;
    CURRENT_ADC_CH1_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    // //CH2 is Tension sensor 1
    // CURRENT_ADC_CH2_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    // CURRENT_ADC_CH2_MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
    // CURRENT_ADC_CH2_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    // //CH3 is Tension sensor 2
    // CURRENT_ADC_CH3_CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
    // CURRENT_ADC_CH3_MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
    // CURRENT_ADC_CH3_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

    //Configure for Signed operation
    CURRENT_ADC.CTRLB = ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm;
    CURRENT_ADC.CTRLA = ADC_ENABLE_bm;

    //Calibrate the ADC
    calibrateCurrentADC();
}

/************************************************************************
* compensateCurrent(inCurrent)
*
* Compensates for duty cycle dependent errors in current measurements
* In the system, the power supply average current is measured. This does not
* equal the current in the windings when PWM is applied because the windings
* see continuous current.  The motor driver bridge inverts the polarity of the
* windings with respect to the power supply for the (1-D) portion of the
* PWM cycle.  In the limit of 50 percent, the power supply provides no net
* power because all current provided in the first half cycle is drained by the second.
* However, the windings did see current and did experience some heating because of it.
*
* The formula is: compensatedCurrent = measuredCurrent / (2 * D - 1).
*
* This formula explodes at 50 percent duty cycle because measured current is 0, but
* the compensated current should be non-zero.  Therefore, the value is capped
* when the duty cycle is less than the LOW_THRESHOLD_DUTY_CYCLE.
************************************************************************/
static float compensateCurrent(float inCurrent)
{
    float dutyCycle = (256.0 + (float) getOldControlValue()) / 512.0;

    if(dutyCycle < TEMPERATURE_CORRECTION_LOW_THRESHOLD_DUTY_CYCLE)
    {
        dutyCycle = TEMPERATURE_CORRECTION_LOW_THRESHOLD_DUTY_CYCLE;
    }
    return inCurrent / (dutyCycle*2.0 - 1.0);
}

/************************************************************************
* readCurrentSignals()
* Reads the most recent captures into appropriate buffers for each motor
************************************************************************/
float thermistorVoltage;
int rawReading;
void readCurrentSignals()
{

    float tempMotorCurrent;

    if(CURRENT_ADC.INTFLAGS & ADC_CH0IF_bm)
    {
        CURRENT_ADC.INTFLAGS = ADC_CH0IF_bm;
        rawReading = CURRENT_ADC.CH0RES;
        thermistorVoltage = ADC_CODES_TO_VOLTS_SIGNED(rawReading);
        statorTemperature = Thermistor_RtoT(Thermistor_VtoR(thermistorVoltage));
        if(statorTemperature < 0) statorTemperature = 0;
        if(statorTemperature > 130) statorTemperature = 130;
    }

    if(CURRENT_ADC.INTFLAGS & ADC_CH1IF_bm)
    {
        CURRENT_ADC.INTFLAGS = ADC_CH1IF_bm;
        rawMotorCurrent = CURRENT_ADC.CH1RES;
        rawMotorCurrent = rawMotorCurrent - ADCCurrentCalibration;
        tempMotorCurrent = compensateCurrent(ADC_CODES_TO_AMPERES((float)rawMotorCurrent));
        
        //motorCurrent = fabs(tempMotorCurrent);
        
        // only noise will be positive.
        if(tempMotorCurrent > 0.0)
        {
            motorCurrent = 0;
        } else {
            motorCurrent = -tempMotorCurrent;
        }

    }

    // if(CURRENT_ADC.INTFLAGS & ADC_CH2IF_bm)
    // {
    //     CURRENT_ADC.INTFLAGS = ADC_CH2IF_bm;
    //     tension[0] = CURRENT_ADC.CH2RES;
    // }

    // if(CURRENT_ADC.INTFLAGS & ADC_CH3IF_bm)
    // {
    //     CURRENT_ADC.INTFLAGS = ADC_CH3IF_bm;
    //     tension[1] = CURRENT_ADC.CH3RES;
    // }
}
