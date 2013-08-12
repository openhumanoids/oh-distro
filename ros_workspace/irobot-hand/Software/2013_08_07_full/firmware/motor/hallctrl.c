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

#include "hallctrl.h"
#include "velcounter.h"
#include "PIDcontroller.h"
#include <avr/io.h>
#include <avr/interrupt.h>

//Private module variables
static volatile DIRECTION_MODE_t direction = DIRECTION_STOP;
static uint8_t oldControlSignal = 0;
static volatile uint8_t lastHall = 0;
static volatile const int8_t HallIncrement[36] = { 0, -2, -1,  2,  1,  3,
                                                   2,  0,  1, -2, -3, -1,
                                                   1, -1,  0,  3,  2, -2,
                                                  -2,  2, -3,  0, -1,  1,
                                                  -1,  3, -2,  1,  0,  2,
                                                  -3,  1,  2, -1, -2,  0};

//Public variables
volatile int16_t encoder = 0;

//Private functions
//static void updateMotor1(void);

// Convert hall readings into +/- encoder value
static int8_t getHallIncrement(uint8_t last, uint8_t current)
{
    if (last == 0)
        return 0;
    return HallIncrement[6 * (last-1) + (current-1)];
}


/************************************************************************
 * Interrupt handlers
 ************************************************************************/

/************************************************************************
 * Interrupt handler for Motor 1 pin change.  When any Hall sensor changes
 * state, this will fire.  It updates the capture accumulator with the new
 * period and moves the motor to the next commutation state
 ************************************************************************/
ISR(MOTOR1_INT_vect)
{
    uint8_t currentHall = (MOTOR1PORT.IN & 0x70) >> 4;
    encoder += getHallIncrement(lastHall, currentHall);
    lastHall = currentHall;

    //ASSERT_DEBUG_LINE();
    updateMotorCount();
    updateMotor();
    //DEASSERT_DEBUG_LINE();
    return;
}

/************************************************************************
 * Motor 1 fault interrupt.  Shuts down the motor when either OTW or FAULT
 * assert.  Currently not used because of cycle-by-cycle limiting on the
 * driver chip.
 ************************************************************************/
//ISR(MOTOR1_FAULT_vect)
//{
//    resetPIDController(0);
//  updateMotorDirection(0,DIRECTION_STOP);
//    updateMotorVoltage(0,0x00);
//    forceMotorUpdate();
//}

/************************************************************************
 * getMotorDirection()
 *
 * Accessor for the current programmed motor direction
 ************************************************************************/
DIRECTION_MODE_t getMotorDirection(void)
{
    return direction;
}

/************************************************************************
 * getOldControlValue()
 *
 * Accessor for the current programmed motor direction
 ************************************************************************/
uint8_t getOldControlValue(void)
{
    return oldControlSignal;
}

/************************************************************************
 * forceMotorUpdate()
 *
 * Can be called from userspace at any time to make sure the motor is in
 * the correct state.  This is important for committing a change in direction
 * or if something goes wrong during commutation and Hall sensor bounce
 * selects the wrong state.  It is called by the PID loop just in case.
 ************************************************************************/
// void forceMotorUpdate(void)
// {
//     updateMotor1();
// }

/************************************************************************
 * updateMotorDirection(newDirection)
 *
 * Updates the motor in the direction of newDirection.  newDirection can have the value
 * DIRECTION_NO_CHANGE to ensure that no attempt is made to change the direction.
 * Otherwise, direction can be DIRECTION_FORWARD, DIRECTION_STOP, or DIRECTION_REVERSE
 ************************************************************************/
void updateMotorDirection(DIRECTION_MODE_t newDirection)
{
    if((newDirection == DIRECTION_STOP) || 
       (newDirection != DIRECTION_NO_CHANGE && direction != newDirection))
    {
        updateMotorVoltage(0);
        resetVelocityCounter();
        direction = newDirection;
    }
}



/************************************************************************
 * updateMotorVoltage(controlSignal)
 *
 * Changes Motor to have speed marked by controlSignal
 *
 ************************************************************************/
void updateMotorVoltage(uint8_t controlSignal)
{
    if (controlSignal > Parameter[PARAMETER_MAXIMUM_PWM])
        controlSignal = Parameter[PARAMETER_MAXIMUM_PWM];
    
    if(controlSignal != oldControlSignal)
    {
        MOTOR1_PWMA_CCBUF = controlSignal + 256;
        MOTOR1_PWMB_CCBUF = controlSignal + 256;
        MOTOR1_PWMC_CCBUF = controlSignal + 256;
#ifdef USE_SWITCHING
        MOTOR1_INVCURRENT_CCBUF = controlSignal + 256;
#endif
        oldControlSignal = controlSignal;
    }
}

/************************************************************************
 * configurePortIO()
 * This function configures the peripheral I/O ports.  The direction,
 * default out state, and drivers are set up here.
 ************************************************************************/
void configurePortIO(void)
{
    //First setup the GPIOs by port.
    //PA0 - VREF
    //PA1 - TEMP
    //PA2 - IMON
    //PA3 - TENSION1
    //PA4 - TENSION2
    //PA5 - IMON_REF
    //PA6 - NC
    //PA7 - NC
    PORTA.OUT = 0x00;
    PORTA.DIR = 0x00;

    //Port B is entirely no-connect, except for CPU debug load
    PORTB.OUT = 0x00;
    PORTB.DIR = 0x04;

    //PC0 - PWMA1
    //PC1 - PWMB1
    //PC2 - PWMC1
    //PC3 - SEL
    //PC4 - HA1
    //PC5 - HB1
    //PC6 - HC1
    //PC7 - LED/ID
    PORTC.OUT = 0x00;
    PORTC.DIR = 0x8F; //BA: used to be 0F

    //PD0 - RX_PROX1_nEN
    //PD1 - TX_PROX1_EN
    //PD2 - RX_PROX1 (also debug to PC)
    //PD3 - TX_PROX1 (also debug to PC)
    //PD4 - RX_PROX2_EN
    //PD5 - TX_PROX2_EN
    //PD6 - RX_PROX2
    //PD7 - TX_PROX2
    PORTD.OUT = 0x22;
    PORTD.DIR = 0xBB;

    //PE0 - nRESET_A1
    //PE1 - nRESET_B1
    //PE2 - nRESET_C1
    //PE3 - LED
    PORTE.OUT = 0x08;
    PORTE.DIR = 0x0F;

    //PR0 - nFAULT
    //PR1 - nOTW
    PORTR.OUT = 0x00;
    PORTR.DIR = 0x00;

    //Configure open collector outputs from the motor controller driver to have pullups
    PORTR.PIN0CTRL = PORT_OPC_PULLUP_gc;
    PORTR.PIN1CTRL = PORT_OPC_PULLUP_gc;
}

/************************************************************************
 * configurePinInterrupts()
 * configure the Hall sensors to generate an interrupt on pin changes.
 * Also handle the port change interrupts for FAULT conditions.
 ************************************************************************/
void configurePinInterrupts(void)
{
    //Set to interrupt on BOTH edges with TOTEM drivers and no slew rate control
    MOTOR1_HA_PINCTRL = PORT_ISC_BOTHEDGES_gc | PORT_OPC_TOTEM_gc;
    MOTOR1_HB_PINCTRL = PORT_ISC_BOTHEDGES_gc | PORT_OPC_TOTEM_gc;
    MOTOR1_HC_PINCTRL = PORT_ISC_BOTHEDGES_gc | PORT_OPC_TOTEM_gc;

    //Set to interrupt on FALLING edges and be with pullups as input
    MOTOR1_FAULT_PINCTRL = PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;
    MOTOR1_OTW_PINCTRL = PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;

    //Configure Hall sensor pins to interrupt group zero.
    MOTOR1PORT.INT0MASK = (1 << MOTOR1_HA_PIN) | (1 << MOTOR1_HB_PIN) | ( 1 << MOTOR1_HC_PIN);

    //Configure Fault sensor pins to interrupt group one.
    //MOTOR1PORT.INT1MASK = (1 << MOTOR1_FAULT_PIN) | (1 << MOTOR1_OTW_PIN);

    //Enable interrupt 0 on PORTC to priority LOW
    MOTOR1PORT.INTCTRL = PORT_INT0LVL_LO_gc;
    //MOTOR1PORT.INTCTRL = PORT_INT1LVL_LO_gc | PORT_INT0LVL_LO_gc;
}

/************************************************************************
 * configureMotorPWMs()
 *
 * Configure the PWM module that generate drive waveforms for the motors.
 * The output will be gated or not depending on the Hall sensor states.
 ************************************************************************/
void configureMotorPWMs(void)
{
    //Now configure the PWMs for the appropriate frequency and pin outputs
    //The clock will be the 32MHz RC Oscillator onboard the chip
    //No prescaling is required, and it will give better PWM duty cycle granularity
    direction = DIRECTION_STOP;

    //Shutdown all TC outputs before configuration
    MOTOR1_TC.CTRLC = 0x00;

    //Disable event actions
    MOTOR1_TC.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

    //Set all TC modules to 16-bit mode
    MOTOR1_TC.CTRLE = 0x00;

    //Set the period to 512 counts to make duty cycle increments work at 8 bits
    //Value is 511 because counter moves from 0 to the value and then back to zero on the following cycle.
    //This creates a frequency of 62.5 kHz, because 32MHz / 512 = 62.5 kHz.
    MOTOR1_TC.PERBUF = 511;

    //For now, initialize to a duty cycle of 50%, a command of 0.
    //Each pin can have different duty cycles, so each one is on its own
    MOTOR1_PWMA_CCBUF = 256;
    MOTOR1_PWMB_CCBUF = 256;
    MOTOR1_PWMC_CCBUF = 256;
#ifdef USE_SWITCHING
    MOTOR1_INVCURRENT_CCBUF = 256;
#endif

    MOTOR1_TC.CTRLB = TC_WGMODE_SS_gc;

    //Activate the Timer Counters
    MOTOR1_TC.CTRLA = TC_CLKSEL_DIV1_gc;
}

/***********************************************************************
updateMotor()
Takes as input the Hall sensors and generates the appropriate drive signals to Motor 1
This routine is blind to the desired PWM duty cycle, so it is safe to call any time.
In general, it should be called any time one of the Hall sensors changes state
or any time the direction needs to be updated.  This includes when the direction
is updated to STOP

The necessary drive signals were taken from TI's description of the DRV8132 and other sources.
The outputs are RESETA, RESETB, RESETC, PWMA, PWMB, and PWMC.
This module assumes that timer counters have already been configured with a duty cycle greater than
50% to PWMA, PWMB, and PWMC, so this function is mostly responsible for connecting and disconnecting the
underlying PWM source from the port pin.
In any given state, one PWM output is high, one is low, and the third is off.  A high output is accomplished
by connecting the underlying PWM to port pin.  Because the PWM has a greater than 50% duty cycle, this means
that the associated phase will have net positive current into it.
A low output is accomplished by feeding the same PWM signal to the low pin with its inversion bit set.
This ensures that the output is out of phase at the inverse duty cycle.  This will make its half-bridge a net low.
The off output is disconnected from the PWM module and driven low.  In addition, the associated RESET is driven low
to activate it.  This is required so that the half-bridge is tristated rather than pulled low.

The only difference between forward and reverse is that the high and low ports for that Hall state are reversed.
More complicated drive structures may be possible depending on the particular motor chosen, but this mechanism
seems to be the generally accepted one.
************************************************************************/
void updateMotor(void)
{

    DIRECTION_MODE_t thisDirection;
    uint8_t sensorState;

    //First get the state of the Hall Effect Sensors
    //sensorState.0 is 1, sensorState.1 is 2, and sensorState.2 is 3
    sensorState = (MOTOR1PORT.IN & 0x70) >> 4;
    thisDirection = direction;

    //Configure pin outputs to match the Hall state
    //In comments, H3-H2-H1 is given to match Hall Sensor states
    //Reverse just involves swapping A and B

    if(thisDirection == DIRECTION_STOP)
    {
        MOTOR1_RESETPORT.OUT = (MOTOR1_RESETPORT.OUT & ~MOTOR1_RESET_bm);
        MOTOR1_PWMA_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
        MOTOR1_PWMB_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
        MOTOR1_PWMC_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#ifdef USE_SWITCHING
        MOTOR1_INVCURRENT_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#endif
        MOTOR1_TC.CTRLB = TC_WGMODE_SS_gc;
        averagePeriod = 65534;
        averageRPM = 0;
        return;
    }
#ifdef USE_SWITCHING
    //The current inversion PWM is always active if the motor is active
    //It is inverted because of the required hookup of the ADC differential inputs
    MOTOR1_INVCURRENT_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#endif

    switch(sensorState)
    {
        case 0x01:
            //0-0-1 - A and -C with reset on B for reverse
            MOTOR1_RESETPORT.OUT = (MOTOR1_RESETPORT.OUT & ~MOTOR1_RESET_bm) | MOTOR1_RESETA_DIS | MOTOR1_RESETC_DIS;
            MOTOR1_PWMB_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#ifdef USE_SWITCHING
            MOTOR1_TC.CTRLB = MOTOR1_PWMA_EN | MOTOR1_PWMC_EN | MOTOR1_INVCURRENT_EN | TC_WGMODE_SS_gc;
#else
            MOTOR1_TC.CTRLB = MOTOR1_PWMA_EN | MOTOR1_PWMC_EN | TC_WGMODE_SS_gc;
#endif
            if(thisDirection == DIRECTION_REVERSE)
            {
                MOTOR1_PWMA_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMC_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            else
            {
                MOTOR1_PWMA_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMC_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            break;
        case 0x02:
            //0-1-0 - B and -A with reset on C for reverse
            MOTOR1_RESETPORT.OUT = (MOTOR1_RESETPORT.OUT & ~MOTOR1_RESET_bm) | MOTOR1_RESETA_DIS | MOTOR1_RESETB_DIS;
            MOTOR1_PWMC_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#ifdef USE_SWITCHING
            MOTOR1_TC.CTRLB = MOTOR1_PWMA_EN | MOTOR1_PWMB_EN | MOTOR1_INVCURRENT_EN | TC_WGMODE_SS_gc;
#else
            MOTOR1_TC.CTRLB = MOTOR1_PWMA_EN | MOTOR1_PWMB_EN | TC_WGMODE_SS_gc;
#endif
            if(thisDirection == DIRECTION_REVERSE)
            {
                MOTOR1_PWMA_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMB_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            else
            {
                MOTOR1_PWMA_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMB_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            break;
        case 0x03:
            //0-1-1 - B and -C with reset on A for reverse
            MOTOR1_RESETPORT.OUT = (MOTOR1_RESETPORT.OUT & ~MOTOR1_RESET_bm) | MOTOR1_RESETB_DIS | MOTOR1_RESETC_DIS;
            MOTOR1_PWMA_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#ifdef USE_SWITCHING
            MOTOR1_TC.CTRLB = MOTOR1_PWMB_EN | MOTOR1_PWMC_EN | MOTOR1_INVCURRENT_EN | TC_WGMODE_SS_gc;
#else
            MOTOR1_TC.CTRLB = MOTOR1_PWMB_EN | MOTOR1_PWMC_EN | TC_WGMODE_SS_gc;
#endif
            if(thisDirection == DIRECTION_REVERSE)
            {
                MOTOR1_PWMB_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMC_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            else
            {
                MOTOR1_PWMB_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMC_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            break;
        case 0x04:
            //1-0-0 - C and -B with reset on A for reverse
            MOTOR1_RESETPORT.OUT = (MOTOR1_RESETPORT.OUT & ~MOTOR1_RESET_bm) | MOTOR1_RESETB_DIS | MOTOR1_RESETC_DIS;
            MOTOR1_PWMA_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#ifdef USE_SWITCHING
            MOTOR1_TC.CTRLB = MOTOR1_PWMB_EN | MOTOR1_PWMC_EN | MOTOR1_INVCURRENT_EN | TC_WGMODE_SS_gc;
#else
            MOTOR1_TC.CTRLB = MOTOR1_PWMB_EN | MOTOR1_PWMC_EN | TC_WGMODE_SS_gc;
#endif
            if(thisDirection == DIRECTION_REVERSE)
            {
                MOTOR1_PWMB_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMC_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            else
            {
                MOTOR1_PWMB_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMC_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            break;
        case 0x05:
            //1-0-1 - A and -B with reset on C for reverse
            MOTOR1_RESETPORT.OUT = (MOTOR1_RESETPORT.OUT & ~MOTOR1_RESET_bm) | MOTOR1_RESETA_DIS | MOTOR1_RESETB_DIS;
            MOTOR1_PWMC_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#ifdef USE_SWITCHING
            MOTOR1_TC.CTRLB = MOTOR1_PWMA_EN | MOTOR1_PWMB_EN | MOTOR1_INVCURRENT_EN | TC_WGMODE_SS_gc;
#else
            MOTOR1_TC.CTRLB = MOTOR1_PWMA_EN | MOTOR1_PWMB_EN | TC_WGMODE_SS_gc;
#endif
            if(thisDirection == DIRECTION_REVERSE)
            {
                MOTOR1_PWMA_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMB_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            else
            {
                MOTOR1_PWMA_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMB_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            break;
        case 0x06:
            //1-1-0 - C and -A with reset on B
            MOTOR1_RESETPORT.OUT = (MOTOR1_RESETPORT.OUT & ~MOTOR1_RESET_bm) | MOTOR1_RESETA_DIS | MOTOR1_RESETC_DIS;
            MOTOR1_PWMB_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
#ifdef USE_SWITCHING
            MOTOR1_TC.CTRLB = MOTOR1_PWMA_EN | MOTOR1_PWMC_EN | MOTOR1_INVCURRENT_EN | TC_WGMODE_SS_gc;
#else
            MOTOR1_TC.CTRLB = MOTOR1_PWMA_EN | MOTOR1_PWMC_EN | TC_WGMODE_SS_gc;
#endif
            if(thisDirection == DIRECTION_REVERSE)
            {
                MOTOR1_PWMA_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMC_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            else
            {
                MOTOR1_PWMA_PINCTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
                MOTOR1_PWMC_PINCTRL = PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc | PORT_OPC_TOTEM_gc;
            }
            break;

        default:
            //Invalid settings.  Leave things alone and hope the motor resolves
            break;
    }

    return;

}

