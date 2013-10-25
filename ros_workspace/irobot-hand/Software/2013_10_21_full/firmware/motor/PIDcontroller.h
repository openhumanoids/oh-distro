/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-PIDcontroller.h
 // Creation Date:    28 October, 2011
 // Revision:        02
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC PID speed/torque controller header

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/28/11    PDB            Initial Release
01            12/16/11    ZAC            Made PIDPreset more generic
02            01/16/12    ZAC            Made maximum command a parameter
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_FGRMCU_0_PIDCONTROLLER_H_
#define C1482_SRC_FGRMCU_0_PIDCONTROLLER_H_

#include <stdint.h>
#include "hallctrl.h"
#include "motor.h"


#define CONTROL_LOOP_RTC RTC
#define CONTROL_LOOP_PERIOD_MS 5.8 // should be 5.0 but calculated to be about 5.8

// #define PID_SCALE_FACTOR_I 100.0
// #define PID_SCALE_FACTOR_V 100.0

#define OUTPUT_LOW_VAL 0.0
#define OUTPUT_HIGH_CLAMP 253.0

typedef enum PID_SCHEME_enum
{
    CONTROL_SCHEME_VOLTAGE,
    CONTROL_SCHEME_VELOCITY,
    CONTROL_SCHEME_CURRENT,
    CONTROL_SCHEME_POWER,
    CONTROL_SCHEME_POSITION
} PID_SCHEME_t;

typedef struct
{
    float integrationError;
    float previousError;
    PID_SCHEME_t controlScheme;
    uint16_t setpoint;
} PID_state_t;

extern int16_t positionSetPoint;
extern PID_state_t PID_state;
extern volatile uint8_t PID_runPID;

extern float maximumOutputCommand;


void resetPIDController(void);
void configurePIDController(void);
uint8_t runPIDController(void);
void PIDPreset(PID_SCHEME_t newScheme);
uint16_t positionControl(int16_t setpoint);

#endif /* C1482_SRC_FGRMCU_0_PIDCONTROLLER_H_ */
