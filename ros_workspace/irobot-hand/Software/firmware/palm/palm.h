/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PLMMCU-0.h
 // Creation Date:    24 February, 2012
 // Revision:        01
 // Hardware:        ATxmega128A1
 // Description:    Main header file for Palm system

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            03/06/12    ZAC            Initial Release
01            03/07/12    ZAC            Changed back to PC bitrate of 115200
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_PLMMCU_0_H_
#define C1482_SRC_PLMMCU_0_H_

#include <stdint.h>
#include <avr/wdt.h>

    // Global Macros
#define F_CPU 32000000UL

#define USE_PC_BITRATE 1

#define FREERUN_TC TCC0
#define FREERUN_TC_vect TCC0_OVF_vect

#define SPREAD_TC TCC1
#define SPREAD_TC_vect TCC1_OVF_vect

void LEDon(void);
void LEDoff(void);
void LEDtoggle(void);

typedef enum SPREAD_CONTROL_MODE_enum
{
    SPREAD_MODE_NONE,
    SPREAD_MODE_POSITION
} SPREAD_CONTROL_MODE_t;

// typedef enum SPREAD_DIRECTION_enum
// {
//     SPREAD_DIRECTION_STOP,
//     SPREAD_DIRECTION_FORWARD,
//     SPREAD_DIRECTION_REVERSE
// } SPREAD_DIRECTION_t;

extern SPREAD_CONTROL_MODE_t spreadMotorMode;
//extern SPREAD_DIRECTION_t spreadMotorDirection;
extern int16_t targetEncoder;
extern int16_t spreadDeadband;
extern int16_t spreadP;

#endif /* C1482_SRC_PLMMCU_0_H_ */
