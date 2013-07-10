/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-PIDcontroller.c
 // Creation Date:    28 October, 2011
 // Revision:        02
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC PID speed/torque controller

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/28/11    PDB            Initial Release
01            12/16/11    ZAC            Made PIDPreset more generic
02            01/16/12    ZAC            Made module outputs have variable clamps
-------------------------------------------------------------------------------

******************************************************************************/

#include "PIDcontroller.h"
#include "velcounter.h"
#include "currentADC.h"
#include "thermalModel.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>

//Public global variables
float maximumOutputCommand = OUTPUT_HIGH_CLAMP;
int16_t positionSetPoint = 0;
PID_state_t PID_state;
volatile uint8_t PID_runPID = 0;

//Private functions
static void PID_Clamp(float *val, float low, float high);

/************************************************************************
* PID_Clamp(*val,low,high)
*
* Clamps val to between low and high.
************************************************************************/
static void PID_Clamp(float *val, float low, float high)
{
    if(*val<low)    {*val = low;}
    if(*val>high)    {*val = high;}
    if(low == high)    {*val = low;}
}

/************************************************************************
* resetPIDController()
*
* Resets the chosen PID controller such that the device is effectively
* disconnected.  This does not stop the motor.  It just stops the PID
* controller.
************************************************************************/
void resetPIDController(void)
{
    PID_state.integrationError = 0;
    PID_state.previousError = 0;
}

// do position control on the hall effect sensor and return the targetRPM
// suitable for feeding into the velocity control control scheme.
uint16_t positionControl(int16_t setpoint)
{
    // position control
    int16_t perr = setpoint - encoder;
    if (abs(perr) < Parameter[PARAMETER_POSITION_DEADBAND])
        perr = 0;
    
    float targetRPM = perr * Parameter[PARAMETER_POSITION_KP];
    
    // cap between (-MAX, -MIN) and (MIN, MAX).
    if (targetRPM > Parameter[PARAMETER_MAXIMUM_RPM])
        targetRPM = Parameter[PARAMETER_MAXIMUM_RPM];
    if (targetRPM < -Parameter[PARAMETER_MAXIMUM_RPM])
        targetRPM = -Parameter[PARAMETER_MAXIMUM_RPM];
    if (targetRPM > 0 && targetRPM < MINIMUM_VELOCITY_COMMAND)
        targetRPM = MINIMUM_VELOCITY_COMMAND;
    if (targetRPM < 0 && targetRPM > -MINIMUM_VELOCITY_COMMAND)
        targetRPM = -MINIMUM_VELOCITY_COMMAND;
    
    // switch direction if required
    DIRECTION_MODE_t newDirection = getMotorDirection();
    if (targetRPM > 0)
        newDirection = DIRECTION_FORWARD;
    else if (targetRPM < 0)
        newDirection = DIRECTION_REVERSE;
    else
        newDirection = DIRECTION_STOP;
    
    updateMotorDirection(newDirection);
    
    return abs(targetRPM);
}

/************************************************************************
* PIDPreset( newScheme)
*
* Presets the PID controller so that it can smoothly transition to a new
* control mode.  This is done by initializing the integration error to
* a value that would yield the current control value being delivered
* to the motor.
*
* This function assumes that the PID controller's new setpoint is already
* set for Velocity and Current control.  Since the setpoint for Power
* is created dynamically, this function does not care in that case.  The
* PID controller does not run in Voltage mode, so that is also ignored.
************************************************************************/
void PIDPreset(PID_SCHEME_t newScheme)
{
    uint8_t oldValue;

    //float desiredVoltage;
    float inputSignal;
    float proportionalGain;
    float currentError;
    float powerLimit;
    float residualError;
    //float PID_KD; // BA: there seems to be no D control, so commenting this out to remove compiler warning
    float PID_KI;
    float PID_KP;

    //Need to compute the proper integral error such that in power control mode the computed control value is the same as before
    switch(newScheme)
    {
        case CONTROL_SCHEME_VELOCITY:
            //Want to drive a voltage consistent with the expected setpoint under no-load conditions
            //Use the motor speed constant to predict a desired voltage.
            //desiredVoltage = (float) PID_state.setpoint / speedConstant[0];
            //PID_Clamp(&desiredVoltage,0.0,48.0);
            //oldValue = floor(desiredVoltage * 255.0 / 48.0);

            //For now, just drive the oldValue as before.  We can experiment with the new system as time allows
            oldValue = getOldControlValue();
            inputSignal = averageRPM;
            currentError = PID_state.setpoint - inputSignal;
            //PID_KD = Parameter[PARAMETER_VELOCITY_KD];
            PID_KI = Parameter[PARAMETER_VELOCITY_KI];
            PID_KP = Parameter[PARAMETER_VELOCITY_KP];
            break;
        case CONTROL_SCHEME_CURRENT:
            //Read a current
            //In Current control mode, it would be nice to operate on raw ADC values for less floating point overhead
            //For now. just operate in terms of milliamperes
            oldValue = getOldControlValue();
            inputSignal = motorCurrent * 1000;
            currentError = PID_state.setpoint - inputSignal;
            //PID_KD = Parameter[PARAMETER_TORQUE_KD];
            PID_KI = Parameter[PARAMETER_TORQUE_KI];
            PID_KP = Parameter[PARAMETER_TORQUE_KP];
            break;
        case CONTROL_SCHEME_POWER:
            oldValue = getOldControlValue();
            powerLimit = getPowerSetpoint();
            inputSignal = getWindingResistance() * pow(motorCurrent,2);
            currentError = powerLimit - inputSignal;
            //PID_KD = Parameter[PARAMETER_POWER_KD];
            PID_KI = Parameter[PARAMETER_POWER_KI];
            PID_KP = Parameter[PARAMETER_POWER_KP];
            break;
        case CONTROL_SCHEME_POSITION:
        {
            oldValue = getOldControlValue();
            inputSignal = averageRPM;
            PID_state.setpoint = positionControl(positionSetPoint);
            currentError = PID_state.setpoint - inputSignal;
            //PID_KD = Parameter[PARAMETER_VELOCITY_KD];
            PID_KI = Parameter[PARAMETER_VELOCITY_KI];
            PID_KP = Parameter[PARAMETER_VELOCITY_KP];
            break;
        }
        case CONTROL_SCHEME_VOLTAGE:
        default:
            //Invalid selection.  Do nothing
            return;
        }

    //Now the error has been computed for the destination control mode and the proper PID constants have been loaded

    //the control signal is just Kp * powerError + Ki * integrationError
    proportionalGain = currentError * PID_KP;
    //PID_Clamp(&proportionalGain,OUTPUT_LOW_VAL,OUTPUT_HIGH_VAL);
    PID_Clamp(&proportionalGain, -maximumOutputCommand, maximumOutputCommand);
    residualError = oldValue - proportionalGain;

    //Now residualError = Ki * integration error
    PID_state.integrationError = residualError / PID_KI;
    PID_Clamp(&(PID_state.integrationError), (float)(OUTPUT_LOW_VAL) / PID_KI, maximumOutputCommand / PID_KI);
    PID_state.previousError = 0;
}

/************************************************************************
* configurePIDController()
*
* Configures the PID controller timer and resets the PID state.
* The RTC is used in order to conserve valuable Timer Counter resources
* for other uses.
************************************************************************/
void configurePIDController(void)
{
    resetPIDController();

    //Configure the Control Loop counter for about 5 ms
    while(CONTROL_LOOP_RTC.STATUS & RTC_SYNCBUSY_bm);    //Wait for sync with RTC clock domain
    CONTROL_LOOP_RTC.PER = 5;                            //Set the period
    while(CONTROL_LOOP_RTC.STATUS & RTC_SYNCBUSY_bm);    //Wait for sync again
    CONTROL_LOOP_RTC.CNT = 0;                            //Reset the count
    CONTROL_LOOP_RTC.COMP = 10;                            //Ensure it will not trigger a compare
    CONTROL_LOOP_RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;
    CONTROL_LOOP_RTC.CTRL = RTC_PRESCALER_DIV1_gc;        //Activate the clock first

    //Uncomment to change back to a full Timer Counter for the Control Loop
    //Also fix the interrupt vector in the header file
    //CONTROL_LOOP_TC.CTRLB = TC_WGMODE_NORMAL_gc;
    //CONTROL_LOOP_TC.CTRLC = 0x00;
    //CONTROL_LOOP_TC.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
    //CONTROL_LOOP_TC.CTRLE = 0x00;
    //CONTROL_LOOP_TC.PERBUF = 3125;
    //CONTROL_LOOP_TC.INTCTRLA = TC_OVFINTLVL_LO_gc;
    //CONTROL_LOOP_TC.CTRLA = TC_CLKSEL_DIV1024_gc;
}




/************************************************************************
* runPIDController()
*
* Uses the PID structure to determine how it should interact with a motor
* It assumes the values in averagePeriod and motorCurrent have been recently
* refreshed by the caller.
*
* This outputs a control signal suitable for passing to updateMotorVoltage()
************************************************************************/
uint8_t runPIDController(void)
{
    float currentError;
    float inputSignal;
    float temp;
    float sum;
    float PID_KD;
    float PID_KI;
    float PID_KP;
    float powerLimit;
    uint8_t controlSignal;

    powerLimit = getPowerSetpoint();
    if(powerLimit >= 0)
    {
        //System in power control mode.  Override
        inputSignal = getWindingResistance() * pow(motorCurrent,2);
        currentError = powerLimit - inputSignal;
        PID_KD = Parameter[PARAMETER_POWER_KD];
        PID_KI = Parameter[PARAMETER_POWER_KI];
        PID_KP = Parameter[PARAMETER_POWER_KP];
    } else
    {
        //System OK.  Run the normal loop.

        switch(PID_state.controlScheme)
        {
            case CONTROL_SCHEME_VELOCITY:
                //The setpoint is in terms of Hall state period length in ticks of the main timer-counter.
                //inputSignal = averagePeriod;
                //currentError = inputSignal - PID_state.setpoint;
                inputSignal = averageRPM;
                currentError = PID_state.setpoint - inputSignal;
                PID_KD = Parameter[PARAMETER_VELOCITY_KD];
                PID_KI = Parameter[PARAMETER_VELOCITY_KI];
                PID_KP = Parameter[PARAMETER_VELOCITY_KP];
                break;
            case CONTROL_SCHEME_CURRENT:
                //Read a current
                //In Current control mode, it would be nice to operate on raw ADC values for less floating point overhead
                //For now. just operate in terms of milliamperes
                inputSignal = motorCurrent * 1000;
                currentError = PID_state.setpoint - inputSignal;
                PID_KD = Parameter[PARAMETER_TORQUE_KD];
                PID_KI = Parameter[PARAMETER_TORQUE_KI];
                PID_KP = Parameter[PARAMETER_TORQUE_KP];
                break;
            case CONTROL_SCHEME_POSITION:
            {
                inputSignal = averageRPM;
                PID_state.setpoint = positionControl(positionSetPoint);
                currentError = PID_state.setpoint - inputSignal;
                PID_KD = Parameter[PARAMETER_VELOCITY_KD];
                PID_KI = Parameter[PARAMETER_VELOCITY_KI];
                PID_KP = Parameter[PARAMETER_VELOCITY_KP];
                break;
            }
            case CONTROL_SCHEME_VOLTAGE:
                //Voltage setpoints are the byte directly suitable for passing to updateMotorVoltage.
                //These 8 bits are stored in the upper word of the 16 bit PID setpoint.
            default:
                controlSignal = PID_state.setpoint / 256;
                return controlSignal;
        }
    }

    temp = currentError * PID_KP;
    PID_Clamp(&temp,-maximumOutputCommand, maximumOutputCommand);
    sum = temp;

    PID_state.integrationError += currentError;
    if(PID_KI == 0)
    {
        PID_state.integrationError = 0;
    } else {
        PID_Clamp(&(PID_state.integrationError),(float)(OUTPUT_LOW_VAL) / PID_KI, maximumOutputCommand / PID_KI);
    }
    sum += PID_state.integrationError * PID_KI;

    temp = currentError - PID_state.previousError;
    PID_state.previousError = currentError;
    temp = temp * PID_KD;
    PID_Clamp(&temp, -maximumOutputCommand, maximumOutputCommand);
    sum += temp;

    PID_Clamp(&sum, OUTPUT_LOW_VAL, maximumOutputCommand);

    controlSignal = ceil(sum);
    return controlSignal;
}
