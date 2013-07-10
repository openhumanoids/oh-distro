/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-thermalModel.h
 // Creation Date:    22 November, 2011
 // Revision:        01
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC Thermal Model header

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            11/22/11    PDB            Initial Release
01            12/20/11    ZAC            Adding Speed Constant
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_FGRMCU_0_THERMALMODEL_H_
#define C1482_SRC_FGRMCU_0_THERMALMODEL_H_

#include "PIDcontroller.h"

/*
#define DEFAULT_WINDING_TAU 1777.0
#define DEFAULT_THERMAL_R_TO_CASE 2.66
#define DEFAULT_T_PLUS 65.0
#define DEFAULT_T_MINUS 65.0
#define DEFAULT_T_MAX 85.0
#define DEFAULT_CU_ALPHA 0.0039
#define DEFAULT_T_OFF_RESET 5.0
#define DEFAULT_T_TARGET 65.0
#define DEFAULT_SPEED_CONSTANT 402
*/

#define THERMAL_DISCARD 0
#define THERMAL_DELAY 1
#define THERMAL_OK 2

#define THERMISTOR_B 3974.0
#define THERMISTOR_NOMINAL_R 10000.0
#define THERMISTOR_NOMINAL_T 298.15

#define THERMISTOR_FIXED_R 10000.0
#define THERMISTOR_PULLUP_VOLTAGE 2.5

extern float oldDeltaT;

float Thermistor_RtoT(float MeasR);
float Thermistor_VtoR(float ThermistorVoltage);

int thermalCheckSetpoint(PID_SCHEME_t newScheme, int newSetpoint);
void updateThermalModel(float newTemp,float newCurrent);
float getPowerSetpoint(void);
float getWindingResistance(void);


#endif /* C1482_SRC_FGRMCU_0_THERMALMODEL_H_ */
