/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-velcounter.h
 // Creation Date:    28 October, 2011
 // Revision:        01
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC velocity counter using Hall sensors header

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/28/11    PDB            Initial Release
01            12/15/11    ZAC            Added variable for maximum RPM
                                    Removed MINIMUM_HALL_PERIOD criteria
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_FGRMCU_0_VELCOUNTER_H_
#define C1482_SRC_FGRMCU_0_VELCOUNTER_H_

#include "PIDcontroller.h"
#include "motor.h"
#include <stdint.h>

//Need to compute the number of counts per millisecond
//The main CPU clock is 32000000 (F_CPU)
//It is being prescaled by 256 to yield the timer
//F_CPU / 256 is the frequency, so 256 / F_CPU is the period in seconds
//Need to convert period in MS to seconds by dividing by 1000
//Final formula below
#define COUNTS_PER_CONTROL_PERIOD 625

#define MOTOR1_EVENT_TIMER_TC TCE0
#define MOTOR1_EVENT_TIMER_vect TCE0_OVF_vect

#define EVENT_HALL_A1 EVSYS_CHMUX_PORTC_PIN4_gc
#define EVENT_HALL_B1 EVSYS_CHMUX_PORTC_PIN5_gc
#define EVENT_HALL_C1 EVSYS_CHMUX_PORTC_PIN6_gc

#define MOTOR1_HA_EN TC0_CCAEN_bm
#define MOTOR1_HB_EN TC0_CCBEN_bm
#define MOTOR1_HC_EN TC0_CCCEN_bm

#ifndef OME_MOTOR

#define EVENT_HALL_A2 EVSYS_CHMUX_PORTC_PIN3_gc
#define EVENT_HALL_B2 EVSYS_CHMUX_PORTC_PIN4_gc
#define EVENT_HALL_C2 EVSYS_CHMUX_PORTC_PIN5_gc

#define MOTOR2_EVENT_TIMER_TC TCF0
#define MOTOR2_EVENT_TIMER_vect TCF0_OVF_vect

#define MOTOR2_HA_EN TC0_CCAEN_bm
#define MOTOR2_HB_EN TC0_CCBEN_bm
#define MOTOR2_HC_EN TC0_CCCEN_bm

#endif

#define PERIOD_CONVERSION_CONSTANT 0.0000008
#define MOTOR_POLE_PAIRS 4

extern volatile uint16_t currentRelativeCount;
extern float averagePeriod;
extern uint16_t averageRPM;

//uint8_t isOverspeed(void);

void resetVelocityCounter(void);
void readVelocitySignal(void);
void configureMotorCounters(void);
void updateMotorCount(void);

#endif /* C1482_SRC_FGRMCU_0_VELCOUNTER_H_ */
