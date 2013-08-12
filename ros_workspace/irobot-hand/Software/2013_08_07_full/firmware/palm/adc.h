/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PLMMCU-0_adc.h
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


#ifndef C1482_SRC_PXDMCU_0_ADC_H_
#define C1482_SRC_PXDMCU_0_ADC_H_

#define EXTERNAL_MONITOR_OFFSET 0
//#define MOTORCURRENT_MONITOR_OFFSET 3
#define EXTERNALTEMP_MONITOR_OFFSET 4
//#define MOTORTEMP_MONITOR_OFFSET 5


#define ADC_VREF (2.5)
//#define ADC_DELTAV (ADC_VREF * 0.05)
#define ADC_TOP_VALUE 2047.0

#define ADC_CODES_TO_VOLTS_SIGNED(C) ((C) * ADC_VREF/(ADC_TOP_VALUE+1))
//#define ADC_CODES_TO_VOLTS_UNSIGNED(C) (((C) * ADC_VREF/ADC_TOP_VALUE) - ADC_DELTAV)

#define THERMISTOR_B 3974.0
#define THERMISTOR_NOMINAL_R 10000.0
#define THERMISTOR_NOMINAL_T 298.15

#define THERMISTOR_FIXED_R 10000.0
#define THERMISTOR_PULLUP_VOLTAGE 2.5


void configureADC(void);
void captureSweep(int16_t *outputData);
float Thermistor_VtoR(float ThermistorVoltage);
float Thermistor_RtoT(float MeasR);
//void calibrateIMON_ADCOffset(void);

#endif /* C1482_SRC_PXDMCU_0_ADC_H_ */
