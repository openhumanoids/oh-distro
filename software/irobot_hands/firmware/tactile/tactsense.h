/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-TACMCU-0_tactsense.h
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Tactile sensor driver

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_TACMCU_0_TACTSENSE_H_
#define C1482_SRC_TACMCU_0_TACTSENSE_H_

#define NUMBER_OF_TACTILE_SENSORS 48

#define TACT_TC TCC1
#define TACT_TC_vect TCC1_OVF_vect

extern int16_t pressureData[NUMBER_OF_TACTILE_SENSORS];
extern int16_t pressureDataOffset[NUMBER_OF_TACTILE_SENSORS];
extern uint8_t pressureCalibrated[NUMBER_OF_TACTILE_SENSORS];
extern uint16_t lastPressure[NUMBER_OF_TACTILE_SENSORS];
extern int16_t adjustedPressure[NUMBER_OF_TACTILE_SENSORS];
extern int16_t pressureTempData[NUMBER_OF_TACTILE_SENSORS];

extern volatile uint8_t tactReady;

void initTactileModule(void);
void configureSPIModules(void);
void collectAllTactSensors(void);
void collectAllCalibrationValues(void);
void doTactSensors(void);

#endif /* C1482_SRC_TACMCU_0_TACTSENSE_H_ */
