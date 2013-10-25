/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-thermalModel.c
 // Creation Date:    22 November, 2011
 // Revision:        02
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC Thermal Model

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            11/22/11    PDB            Initial Release
01            12/16/11    ZAC            Update to add Velocity override for thermal control
02            1/5/12        ZAC            Added slack around velocity override
-------------------------------------------------------------------------------

******************************************************************************/

#include <math.h>
#include "thermalModel.h"
#include "PIDcontroller.h"
#include "currentADC.h"
#include "velcounter.h"

//public globals
float windingTemperature = 0.0;
float statorTemperature = 0.0;

//Private functions
static void updateTemperatureEstimate(float newTemp, float newCurrent);
static uint8_t isValidOverride(uint16_t setpoint, PID_SCHEME_t scheme);

//Private variables
static float realWindingResistance = DEFAULT_WINDING_RESISTANCE;
static float exp_minus_t_over_tau_winding = 0.0;
static float exp_minus_t_over_tau_stator = 0.0;
static float deltaT_winding_to_stator = 0.0;
static float deltaT_stator_to_housing = 0.0;
static protection_state_t protectionState = MANDATORY_COOLDOWN;
static float cooldownTimeRemaining = 0;

// must come after initializeParameters()
void initializeThermalModel(void)
{
    exp_minus_t_over_tau_winding = exp(-CONTROL_LOOP_PERIOD_MS/(Parameter[PARAMETER_TAU_WINDING_TO_STATOR]));
    exp_minus_t_over_tau_stator = exp(-CONTROL_LOOP_PERIOD_MS/(Parameter[PARAMETER_TAU_STATOR_TO_CASE]));
    windingTemperature = housingTemperature;
    statorTemperature = housingTemperature;
};

/************************************************************************
* thermalCheckSetpoint(newScheme,newSetpoint)
*
* Called when a command arrives.  It updates state if appropriate and
* tells the caller whether to accept the setpoint.
*
* At present it does not need to look at the actual new setpoint because its
* decision is entirely based on state.  The state machine itself operates
* on the setpoint and will decide what to do.
*
* return THERMAL_DISCARD for delete entirely
* return THERMAL_DELAY for apply later
* return THERMAL_OK for OK
************************************************************************/
thermal_state_t thermalCheckSetpoint(PID_SCHEME_t newScheme, int newSetpoint)
{
    switch(protectionState)
    {
        case NORMAL_OPERATION: //Fallthrough intentional
        case NORMAL_COOLDOWN:  //Fallthrough intentional
        case STOPPED:
            //Any command is OK
            return THERMAL_OK;
        case THERMAL_WARNING:
            //This forces us out of thermal warning mode unconditionally
            //Any command is OK
            protectionState = NORMAL_OPERATION;
            return THERMAL_OK;
        case MANDATORY_COOLDOWN:
            //No command is valid.  In addition, do not store the command for application after cooldown
            return THERMAL_DISCARD;
        case THERMAL_OVERLOAD:
            if(isValidOverride(newSetpoint, newScheme) == 1)
            {
                //The is a valid override
                protectionState = THERMAL_OVERLOAD_OVERRIDE;
                return THERMAL_OK;
            } else {
                //Delay application.
                return THERMAL_DELAY;
            }
        case THERMAL_OVERLOAD_OVERRIDE:
            if(windingTemperature < Parameter[PARAMETER_T_PLUS])
            {
                protectionState = NORMAL_OPERATION;
                return THERMAL_OK;
            } else if(isValidOverride(newSetpoint, newScheme) == 1)
            {
                //The motor is still in a valid override, so apply immediately
                return THERMAL_OK;
            } else {
                //Not a valid override.  Go back to normal OVERLOAD and delay
                protectionState = THERMAL_OVERLOAD;
                PIDPreset(CONTROL_SCHEME_POWER);
                return THERMAL_DELAY;
            }
        case MISSING_THERMISTOR:
            return THERMAL_ERROR;
        default:
            return THERMAL_DISCARD;
    }
}
/************************************************************************
* getWindingResistance()
*
* Returns the current winding resistance estimate
************************************************************************/
float getWindingResistance(void)
{
    return realWindingResistance;
}

/************************************************************************
* getPowerSetpoint()
*
* Returns the power setpoint appropriate for the motor if in THERMAL_OVERLOAD
* Will return a negative number if power should not be used.
************************************************************************/
float getPowerSetpoint(void)
{
    if((protectionState == THERMAL_OVERLOAD) || \
       (protectionState == THERMAL_WARNING)) 
    {
        float maximumDeltaD = Parameter[PARAMETER_T_TARGET] - statorTemperature;
        float powerSetpoint = maximumDeltaD / Parameter[PARAMETER_THERMAL_R_WINDING_TO_STATOR];
        if (powerSetpoint < 0)
        {
            return 0.0;
        } else {
            return powerSetpoint;
        }
    } else {
        return -1.0;
    }
}

/************************************************************************
* isValidOverride( uint16_t setpoint, PID_SCHEME_t scheme)
* returns 1 if the configured setpoint is less than the power limit
* 0 otherwise.
************************************************************************/
static uint8_t isValidOverride(uint16_t setpoint, PID_SCHEME_t scheme)
{
    float maximumCurrent;
    float maximumDeltaT;
    if(scheme == CONTROL_SCHEME_CURRENT)
    {
        //The maximum current is backed out from the power that would would hit tTarget
        maximumDeltaT = Parameter[PARAMETER_T_TARGET] - statorTemperature;
        maximumCurrent = sqrt(maximumDeltaT / (Parameter[PARAMETER_THERMAL_R_WINDING_TO_STATOR] * realWindingResistance));
        //Setpoint is in milliamperes.
        if(setpoint < maximumCurrent * 1000)
        {
            return 1;
        }
    }
    
    if(scheme == CONTROL_SCHEME_VELOCITY)
    {
        //For velocity control to take over, it must be asking for less speed out of the motor while the Power controller is asking for more power
        //In that case, the velocity controller is definitely the lighter load.

        //If the power controller is trying to pull power down, let it have control unconditionally.  If the lower power later stabilizes and the velocity controller
        //wants less speed, let it take over at that point.  In this way the power controller ceiling is always applied unless the motor is at that ceiling and the
        //velocity controller wants to go lower.  Some margin is applied to the power controller ceiling to prevent oscillations.  This means that the motor can
        //run steady state at a low velocity setpoint that is heavily loaded.  If this condition is maintained, motor temperature will continue to climb until reaching
        //Tmax.

        //The maximum current is backed out from the power that would would hit tTarget
        maximumDeltaT = Parameter[PARAMETER_T_TARGET] - statorTemperature;
        maximumCurrent = sqrt(maximumDeltaT / (Parameter[PARAMETER_THERMAL_R_WINDING_TO_STATOR] * realWindingResistance));
        
        if((motorCurrent <= (maximumCurrent * 1.25)) && (averageRPM >= setpoint))
        {
            return 1;
        }
        return 0;
    }
    return 0;
}

/************************************************************************
* updateThermalModel(newTemp,newCurrent)
*
* Updates the running temperature estimate and updates a thermal state
* variable.  This state can override the PID controller with a power
* limiting mode of operation
************************************************************************/

void updateThermalModel(float newTemp, float newCurrent)
{
    updateTemperatureEstimate(newTemp, newCurrent);
    
    // regardless of mode
    if (housingTemperature < 5.0)
    {
        updateMotorDirection(DIRECTION_STOP);
        protectionState = MISSING_THERMISTOR;
        return;
    }

    //float estimatedWindingTemperature;
    switch(protectionState)
    {
        case MANDATORY_COOLDOWN:
            //It is startup or an over-temperature condition. Decrement the cooldown timer until the motor can power up
            //updateTemperatureEsimate(newTemp, newCurrent);
            if(cooldownTimeRemaining > 0)
            {
                cooldownTimeRemaining = cooldownTimeRemaining - CONTROL_LOOP_PERIOD_MS;
            } else {
                //Cooldown done.  Go to stopped
                //deltaT_winding_to_stator = 0.0;
                //deltaT_stator_to_housing = 0.0;
                protectionState = STOPPED;
            }
            break;
        case NORMAL_COOLDOWN:
            //The motor has been stopped and is cooling down.  Decrement the counter unless it has been started again
            //updateTemperatureEsimate(newTemp, newCurrent);
            if(cooldownTimeRemaining <= 0)
            {
                //Cooldown complete.  Force temperature to zero
                //deltaT_winding_to_stator = 0.0;
                //deltaT_stator_to_housing = 0.0;
                protectionState = STOPPED;
            } else if(getMotorDirection() == DIRECTION_STOP)
            {
                cooldownTimeRemaining = cooldownTimeRemaining - CONTROL_LOOP_PERIOD_MS;
            } else {
                //No longer cooling down.
                protectionState = NORMAL_OPERATION;
            }
            break;
        case MISSING_THERMISTOR:
        case NORMAL_OPERATION:
            //Check for over temperature or cooldown conditions
            //estimatedWindingTemperature = updateTemperatureEsimate(newTemp, newCurrent);
            if(getMotorDirection() == DIRECTION_STOP)
            {
                //Begin cooldown
                protectionState = NORMAL_COOLDOWN;
                cooldownTimeRemaining = Parameter[PARAMETER_OFF_TIME] * Parameter[PARAMETER_TAU_WINDING_TO_STATOR];
            } else if(windingTemperature >= Parameter[PARAMETER_T_MAX])
            {
                protectionState = MANDATORY_COOLDOWN;
                cooldownTimeRemaining = Parameter[PARAMETER_OFF_TIME] * Parameter[PARAMETER_TAU_WINDING_TO_STATOR];
                updateMotorDirection(DIRECTION_STOP);
            } else if(windingTemperature >= Parameter[PARAMETER_T_PLUS])
            {
                //Switch to overload mode.  PID loop will catch it
                protectionState = THERMAL_OVERLOAD;
                PIDPreset(CONTROL_SCHEME_POWER);
            }
            break;
        case THERMAL_OVERLOAD:
            //Check for a low enough temperature to exit overload or stopped operation
            //estimatedWindingTemperature = updateTemperatureEsimate(newTemp, newCurrent);
            if(getMotorDirection() == DIRECTION_STOP)
            {
                //Begin cooldown
                protectionState = NORMAL_COOLDOWN;
                cooldownTimeRemaining = Parameter[PARAMETER_OFF_TIME] * Parameter[PARAMETER_TAU_WINDING_TO_STATOR];
            } else if(windingTemperature >= Parameter[PARAMETER_T_MAX])
            {
                protectionState = MANDATORY_COOLDOWN;
                cooldownTimeRemaining = Parameter[PARAMETER_OFF_TIME] * Parameter[PARAMETER_TAU_WINDING_TO_STATOR];
                updateMotorDirection(DIRECTION_STOP);
            } else if(windingTemperature < Parameter[PARAMETER_T_PLUS])
            {
                //Switch to warning mode.  PID loop will catch it
                protectionState = THERMAL_WARNING;
            } else if(isValidOverride(PID_state.setpoint,PID_state.controlScheme) == 1)
            {
                //The setpoint is appropriate for OVERRIDE
                //Preset the PID
                PIDPreset(PID_state.controlScheme);
                protectionState = THERMAL_OVERLOAD_OVERRIDE;
            }
            break;
        case THERMAL_OVERLOAD_OVERRIDE:
            //Check for a low enough temperature to exit overload or stopped operation
            //estimatedWindingTemperature = updateTemperatureEsimate(newTemp, newCurrent);
            if(getMotorDirection() == DIRECTION_STOP)
            {
                //Begin cooldown
                protectionState = NORMAL_COOLDOWN;
                cooldownTimeRemaining = Parameter[PARAMETER_OFF_TIME] * Parameter[PARAMETER_TAU_WINDING_TO_STATOR];
            } else if(windingTemperature >= Parameter[PARAMETER_T_MAX])
            {
                protectionState = MANDATORY_COOLDOWN;
                cooldownTimeRemaining = Parameter[PARAMETER_OFF_TIME] * Parameter[PARAMETER_TAU_WINDING_TO_STATOR];
                updateMotorDirection(DIRECTION_STOP);
            } else if(windingTemperature < Parameter[PARAMETER_T_MINUS])
            {
                //Switch to normal mode.  PID loop will catch it
                protectionState = NORMAL_OPERATION;
                PIDPreset(PID_state.controlScheme);
            } else if(isValidOverride(PID_state.setpoint,PID_state.controlScheme) == 0)
            {
                //The setpoint is not appropriate for OVERRIDE.  Go back to OVERLOAD
                protectionState = THERMAL_OVERLOAD;
                PIDPreset(CONTROL_SCHEME_POWER);
            }
            break;
        case THERMAL_WARNING:
            //Check for entrance into overload or warning
            //estimatedWindingTemperature = updateTemperatureEsimate(newTemp, newCurrent);
            if(getMotorDirection() == DIRECTION_STOP)
            {
                //Begin cooldown
                protectionState = NORMAL_COOLDOWN;
                cooldownTimeRemaining = Parameter[PARAMETER_OFF_TIME] * Parameter[PARAMETER_TAU_WINDING_TO_STATOR];
            } else if(windingTemperature >= Parameter[PARAMETER_T_MAX])
            {
                protectionState = MANDATORY_COOLDOWN;
                cooldownTimeRemaining = Parameter[PARAMETER_OFF_TIME] * Parameter[PARAMETER_TAU_WINDING_TO_STATOR];
                updateMotorDirection(DIRECTION_STOP);
            } else if(windingTemperature >= Parameter[PARAMETER_T_PLUS])
            {
                //Switch to overload mode.  No need to preset because was already in power control
                protectionState = THERMAL_OVERLOAD;
            } else if(windingTemperature < Parameter[PARAMETER_T_MINUS])
            {
                //Full exit from thermal control
                protectionState = NORMAL_OPERATION;
                PIDPreset(PID_state.controlScheme);
            }
            break;
        case STOPPED:
            //Do not update the temperature with the exponential.
            if(getMotorDirection() != DIRECTION_STOP)
            {
                //Motor has started
                protectionState = NORMAL_OPERATION;
            }
            break;
        default:
            break;
    }
    return;
}

/************************************************************************
* updateTemperatureEstimate(newTemp,newCurrent)
*
* Updates the system estimate of winding temperature
* newTemp is the new stator temperature in degrees Celsius, and newCurrent is
* the new motor current in Amperes
*
* returns the estimated winding temperature
************************************************************************/
static void updateTemperatureEstimate(float newTemp, float newCurrent)
{
    // first compute the updated temperature assuming the old deltaTs with
    // the new armature reading
    statorTemperature = deltaT_stator_to_housing + newTemp;
    windingTemperature = deltaT_winding_to_stator + newTemp;
    
    //Compute winding electrical resistance taking the temperature reading into account
    float realWindingResistance = Parameter[PARAMETER_WINDING_R] * 
        (1.0 + Parameter[PARAMETER_CU_ALPHA] * (windingTemperature - 25.0));
    
    //Compute the power supplied to the motor
    float powerSupplied = newCurrent * newCurrent * realWindingResistance;
    
    //Now compute the final temperature delta that would be reached if this power were applied for a long time
    float deltaTFinalWinding = powerSupplied * Parameter[PARAMETER_THERMAL_R_WINDING_TO_STATOR] + deltaT_stator_to_housing;
    //float deltaTFinalWinding = powerSupplied * (Parameter[PARAMETER_THERMAL_R_WINDING_TO_STATOR] + Parameter[PARAMETER_THERMAL_R_STATOR_TO_CASE]);
    
    deltaT_winding_to_stator = (deltaT_winding_to_stator - deltaTFinalWinding) * exp_minus_t_over_tau_winding + deltaTFinalWinding;
    
    // float deltaTFinalStator = deltaT_winding_to_stator * 
    //     Parameter[PARAMETER_THERMAL_R_STATOR_TO_CASE] / Parameter[PARAMETER_THERMAL_R_WINDING_TO_STATOR];

    //float deltaTFinalStator = (deltaT_winding_to_stator * Parameter[PARAMETER_THERMAL_R_STATOR_TO_CASE] + 
    //                           newTemp * Parameter[PARAMETER_THERMAL_R_WINDING_TO_STATOR]) / 
    //    (Parameter[PARAMETER_THERMAL_R_STATOR_TO_CASE] + Parameter[PARAMETER_THERMAL_R_WINDING_TO_STATOR]);

    float deltaTFinalStator = powerSupplied * Parameter[PARAMETER_THERMAL_R_STATOR_TO_CASE];

    deltaT_stator_to_housing = (deltaT_stator_to_housing - deltaTFinalStator) * exp_minus_t_over_tau_stator + deltaTFinalStator;
    
    statorTemperature = deltaT_stator_to_housing + newTemp;
    windingTemperature = deltaT_winding_to_stator + newTemp;
}



/************************************************************************
* FUNCTION:    Thermistor_VtoR
*
* Description: Converts a thermistor voltage to a resistance.
*                Takes thermistor voltage, pullup voltage,
*                pullup fixed R inputs.
*                Vt = Vp * (Rt / (Rt + Rp))
*                -> Rt = (Vt*Rp)/(Vp - Vt)
************************************************************************/
float Thermistor_VtoR(float ThermistorVoltage)
{
    float ThermistorR;

    ThermistorR = ThermistorVoltage * THERMISTOR_FIXED_R;
    ThermistorR /= (THERMISTOR_PULLUP_VOLTAGE - ThermistorVoltage);

    return ThermistorR;

}    // End of Thermistor_VtoR //



/************************************************************************
* FUNCTION:    Thermistor_RtoT
*
* Description: Converts a thermistor resistance to a temperature.
*                Takes measured thermistor resistance, nominal resistance,
*                nominal temperature, and thermistor B constant as inputs.
*                (1/T) = (1/Tnom) + (1/B)* ln(R/Rnom)
************************************************************************/
float Thermistor_RtoT(float MeasR)
{
    float T;

    T = log(MeasR / THERMISTOR_NOMINAL_R);
    T *= (1/THERMISTOR_B);
    T += (1/THERMISTOR_NOMINAL_T);
    T = (1/T);
    T -= 273.15;         //Convert from K to C

    return T;

}    // End of Thermistor_RtoT //
