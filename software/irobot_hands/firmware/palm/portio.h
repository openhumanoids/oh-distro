/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-PLMMCU-0_portio.h
 // Creation Date:    24 February, 2012
 // Revision:        00
 // Hardware:        ATxmega128A1
 // Description:    Input/Output configuration for Palm board

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_PLMMCU_0_PORTIO_H_
#define C1482_SRC_PLMMCU_0_PORTIO_H_

void configurePortIO(void);

#define MOTOR_nDIR_BITMASK 0x02
#define MOTOR_DIR_BITMASK 0x01
//#define MOTOR_EN_BITMASK 0x01

#define MOTOR_DIR_PORT PORTF
//#define MOTOR_EN_PORT PORTK

#endif /* C1482_SRC_PLMMCU_0_PORTIO_H_ */
