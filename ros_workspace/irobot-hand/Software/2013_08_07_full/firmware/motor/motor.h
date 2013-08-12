/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0.h
 // Creation Date:    18 October, 2011
 // Revision:        03
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro Definitions


****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/18/11    PDB            Initial Release
01            12/15/11    ZAC            Added parameter for MAXIMUM_RPM
02            12/20/11    ZAC            Added parameter for SPEED_CONSTANT
03            01/16/12    ZAC            Added parameter for MAXIMUM_COMMAND
-------------------------------------------------------------------------------

******************************************************************************/

#ifndef C1482_SRC_FGRMCU_0_H_
#define C1482_SRC_FGRMCU_0_H_

#include <stdint.h>

// Global Macros
#define F_CPU 32000000UL

//Interrupt vectors
#define CONTROL_TIMER_vect RTC_OVF_vect
#define FAULT_TIMEOUT_COUNT 25

#define DEFAULT_WINDING_RESISTANCE 28.6

#define PARAMETER_TORQUE_KP 0
#define PARAMETER_TORQUE_KI 1
#define PARAMETER_TORQUE_KD 2

#define PARAMETER_VELOCITY_KP 3
#define PARAMETER_VELOCITY_KI 4
#define PARAMETER_VELOCITY_KD 5

#define PARAMETER_POWER_KP 6
#define PARAMETER_POWER_KI 7
#define PARAMETER_POWER_KD 8

#define PARAMETER_WINDING_R 9
#define PARAMETER_THERMAL_R 10
#define PARAMETER_T_PLUS 11
#define PARAMETER_T_MINUS 12
#define PARAMETER_WINDING_TAU 13
#define PARAMETER_T_MAX 14
#define PARAMETER_CU_ALPHA 15
#define PARAMETER_OFF_TIME 16
#define PARAMETER_T_TARGET 17
#define PARAMETER_MAXIMUM_RPM 18
#define PARAMETER_SPEED_CONSTANT 19
#define PARAMETER_MAXIMUM_PWM 20
#define PARAMETER_POSITION_KP 21
#define PARAMETER_POSITION_DEADBAND 22

#define INT_PARAMETER_START 25 //params 0-24 = float, params 25-31 = int

// See common/daisycomm.h for common int parameters:
// #define EEPROM_ADDRESS_FIRMWARE_VERSION 29
// #define EEPROM_ADDRESS_ID 30
// #define EEPROM_ADDRESS_LED 31
#define PARAM_EEPROM_INIT 32

//Just need to ensure some minimum off time
//DRV8312 datasheet recommends 50 ns off time per PWM cycle
//With a 62.5 kHz PWM cycle, this corresponds to a maximum duty cycle of
//about 99.6.  This leads to a maximum PWM signal of 510 out of 512.
//For margin, clamp at 509.  Removing the offset gives a final control signal of 253
//Scale that up to 16 bits by shifting 8 bits.
//Voltage command of 0 corresponds to a safe 50% duty cycle
//Voltage command of 0xFFFF would be an unsafe 100% duty cycle.
#define MAXIMUM_VOLTAGE_COMMAND 64768
#define MINIMUM_VOLTAGE_COMMAND 0 // zero is required here for the MOTOR_COMMAND_STOP direction

#define MAXIMUM_CURRENT_COMMAND 1500
#define MINIMUM_CURRENT_COMMAND 1

#define MAXIMUM_POSITION_COMMAND 7200
#define MINIMUM_POSITION_COMMAND -7200

// Parameter[PARAMETER_MAXIMUM_RPM] defines max velocity command
#define MINIMUM_VELOCITY_COMMAND 200

//extern int isThumb;
extern float Parameter[INT_PARAMETER_START];
//extern uint32_t ParameterI[32-INT_PARAMETER_START];

void LEDon(void);
void LEDoff(void);
void LEDtoggle(void);

#endif /* C1482_SRC_FGRMCU_0_H_ */
