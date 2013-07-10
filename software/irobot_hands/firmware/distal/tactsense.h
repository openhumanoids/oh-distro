/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PXDMCU-0_tactsense.h
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


#ifndef C1482_SRC_PXDMCU_0_TACTSENSE_H_
#define C1482_SRC_PXDMCU_0_TACTSENSE_H_

#define MAXIMUM_NUMBER_OF_TACTILE_SENSORS 12

#define TACT_TC TCC1
#define TACT_TC_vect TCC1_OVF_vect

extern int16_t pressureData[MAXIMUM_NUMBER_OF_TACTILE_SENSORS];
extern int16_t pressureDataOffset[MAXIMUM_NUMBER_OF_TACTILE_SENSORS];
extern uint8_t pressureCalibrated[MAXIMUM_NUMBER_OF_TACTILE_SENSORS];
extern uint16_t lastPressure[MAXIMUM_NUMBER_OF_TACTILE_SENSORS];
extern int16_t adjustedPressure[MAXIMUM_NUMBER_OF_TACTILE_SENSORS];
extern int16_t pressureTempData[MAXIMUM_NUMBER_OF_TACTILE_SENSORS];

extern volatile uint8_t tactReady;

void initTactileModule(void);
void configureSPIModulesPressure(void);
void collectAllTactSensors(void);
void collectAllCalibrationValues(void);
void doTactSensors(void);

#endif /* C1482_SRC_PXDMCU_0_TACTSENSE_H_ */
