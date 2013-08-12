/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-currentADC.c
 // Creation Date:    28 October, 2011
 // Revision:        00
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC current monitoring module

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/28/11    PDB            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_FGRMCU_0_CURRENTADC_H_
#define C1482_SRC_FGRMCU_0_CURRENTADC_H_

//Header files are broken for the ADCA structure's CH members.  Define manually
#define CURRENT_ADC_CH0_CTRL ADCA_CH0_CTRL
#define CURRENT_ADC_CH1_CTRL ADCA_CH1_CTRL
#define CURRENT_ADC_CH2_CTRL ADCA_CH2_CTRL
#define CURRENT_ADC_CH3_CTRL ADCA_CH3_CTRL

#define CURRENT_ADC_CH0_MUXCTRL ADCA_CH0_MUXCTRL
#define CURRENT_ADC_CH1_MUXCTRL ADCA_CH1_MUXCTRL
#define CURRENT_ADC_CH2_MUXCTRL ADCA_CH2_MUXCTRL
#define CURRENT_ADC_CH3_MUXCTRL ADCA_CH3_MUXCTRL

#define CURRENT_ADC_CH0_INTCTRL ADCA_CH0_INTCTRL
#define CURRENT_ADC_CH1_INTCTRL ADCA_CH1_INTCTRL
#define CURRENT_ADC_CH2_INTCTRL ADCA_CH2_INTCTRL
#define CURRENT_ADC_CH3_INTCTRL ADCA_CH3_INTCTRL

#define CURRENT_ADC ADCA

//Not using full TC for actuating sweeps.  Instead using the Event System prescaler

#define CURRENT_VOLTAGE_OFFSET 0.0502
#define CURRENT_SENSE_GAIN 20.08
#define CURRENT_SENSE_RESISTOR 0.050

#ifdef USE_INTERNAL_REF
#define ADC_VREF (2.0625)
#else
#define ADC_VREF (2.5)
#endif
#define ADC_DELTAV (ADC_VREF * 0.05)
#define ADC_TOP_VALUE 2047

#define TEMPERATURE_CORRECTION_LOW_THRESHOLD_DUTY_CYCLE 0.5625

//For single ended unsigned conversions, the ADC is 12 bits with the following transfer function:
//RESULT = (VIN + deltaV) * TOP / VREF
//Define macros for low runtime overhead
//#define ADC_CODES_TO_VOLTS_UNSIGNED(C) (((C) * ADC_VREF/ADC_TOP_VALUE) - ADC_DELTAV)

//For signed differential mode without gain, the ADC has this transfer function:
//RESULT = (VINP - VINN) * (TOP + 1) / VREF
#define ADC_CODES_TO_VOLTS_SIGNED(C) (((float)C) * (float)ADC_VREF/((float)ADC_TOP_VALUE + 1.0))

//In the circuit, the differential input between the current reference and the current signal has a conversion of 1.7 Amps per measured Volt
//This is because the current is sensed over an 0.024 ohm sensor with a gain of 3 at the transducer.  This voltage is then gained by 8 in an op amp
//1 Amp becomes 24 mV * 3 = 72 mV
//Gained by 8 gives 576mV per Ampere, or 1.7 Amperes per Volt

//Conversion for old Finger Test Cell
//#define ADC_CODES_TO_AMPERES(C) ((ADC_CODES_TO_VOLTS_UNSIGNED(C)-CURRENT_VOLTAGE_OFFSET) / (CURRENT_SENSE_GAIN * CURRENT_SENSE_RESISTOR))

#define ADC_CODES_TO_AMPERES(C) (ADC_CODES_TO_VOLTS_SIGNED((float)C) * 1.7)

extern float motorCurrent;
extern int rawMotorCurrent;
extern float statorTemperature;
extern uint16_t tension[2];

void configureADC(void);
void readCurrentSignals(void);

#endif /* C1482_SRC_FGRMCU_0_CURRENTADC_H_ */
