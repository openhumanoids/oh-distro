/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-hallctrl.c
 // Creation Date:    28 October, 2011
 // Revision:        00
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC motor driver using Hall sensors

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/28/11    PDB            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/


#ifndef C1482_SRC_FGRMCU_0_HALLCTRL_H_
#define C1482_SRC_FGRMCU_0_HALLCTRL_H_

#include <stdint.h>


#define MOTOR1_INT_vect PORTC_INT0_vect
#define MOTOR1_HA_PINCTRL PORTC.PIN4CTRL
#define MOTOR1_HB_PINCTRL PORTC.PIN5CTRL
#define MOTOR1_HC_PINCTRL PORTC.PIN6CTRL
#define MOTOR1PORT PORTC

#define MOTOR1OTWPORT PORTR
#define MOTOR1FAULTPORT PORTR

#define MOTOR1_HA_PIN 4
#define MOTOR1_HB_PIN 5
#define MOTOR1_HC_PIN 6

#define MOTOR1_OTW_PIN 0
#define MOTOR1_FAULT_PIN 1

#define MOTOR1_TC TCC0

#define MOTOR1_PWMA_EN TC0_CCAEN_bm
#define MOTOR1_PWMB_EN TC0_CCBEN_bm
#define MOTOR1_PWMC_EN TC0_CCCEN_bm
#define MOTOR1_INVCURRENT_EN TC0_CCDEN_bm

#define MOTOR1_PWMA_PINCTRL PORTC.PIN0CTRL
#define MOTOR1_PWMB_PINCTRL PORTC.PIN1CTRL
#define MOTOR1_PWMC_PINCTRL PORTC.PIN2CTRL
#define MOTOR1_INVCURRENT_PINCTRL PORTC.PIN3CTRL

#define MOTOR1_OTW_PINCTRL    PORTR.PIN0CTRL
#define MOTOR1_FAULT_PINCTRL PORTR.PIN1CTRL

#define MOTOR1_RESETPORT PORTE
#define MOTOR1_RESETA_DIS 0x01
#define MOTOR1_RESETB_DIS 0x02

#define MOTOR1_RESETC_DIS 0x04
#define MOTOR1_RESET_bm 0x07

#define MOTOR1_PWMA_CCBUF TCC0.CCABUF
#define MOTOR1_PWMB_CCBUF TCC0.CCBBUF
#define MOTOR1_PWMC_CCBUF TCC0.CCCBUF
#define MOTOR1_INVCURRENT_CCBUF TCC0.CCDBUF

//#define MOTOR1_FAULT_vect PORTC_INT1_vect

typedef enum DIRECTION_MODE_enum
{
    DIRECTION_STOP,
    DIRECTION_FORWARD,
    DIRECTION_REVERSE,
    DIRECTION_NO_CHANGE
} DIRECTION_MODE_t;

void configureMotorPWMs(void);
void configurePinInterrupts(void);
void configurePortIO(void);
//void forceMotorUpdate(void);
void updateMotor(void);

void updateMotorVoltage(uint8_t controlSignal);
void updateMotorDirection(DIRECTION_MODE_t newDirection);
DIRECTION_MODE_t getMotorDirection(void);
uint8_t getOldControlValue(void);

//BA
// motor encoder reading
extern volatile int16_t encoder;

#endif /* C1482_SRC_FGRMCU_0_HALLCTRL_H_ */
