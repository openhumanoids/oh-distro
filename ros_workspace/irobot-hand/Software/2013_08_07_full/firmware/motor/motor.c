/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0.c
 // Creation Date:    18 October, 2011
 // Revision:        04
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro Top Level Implementation

 ****************************************************/

/******************************************************************************
    Project Build Summary:
-------------------------------------------------------------------------------
Source File                                Cur. Rev. Level        Prev. Rev. Level
--------------------------------        ----------------    ----------------
C1482-SRC-FGRMCU-0.c                    04                    03
C1482-SRC-FGRMCU-0.h                    03                    02
C1482-SRC-FGRMCU-0-currentADC.c            01                    00
C1482-SRC-FGRMCU-0-currentADC.h            00                    N/A
C1482-SRC-FGRMCU-0-hallctrl.c            00                    N/A
C1482-SRC-FGRMCU-0-hallctrl.h            00                    N/A
C1482-SRC-FGRMCU-0-PIDcontroller.c        02                    01
C1482-SRC-FGRMCU-0-PIDcontroller.h        02                    01
C1482-SRC-FGRMCU-0-thermalModel.c        02                    01
C1482-SRC-FGRMCU-0-thermalModel.h        01                    00
C1482-SRC-FGRMCU-0-USART.c                00                    N/A
C1482-SRC-FGRMCU-0-USART.h                01                    00
C1482-SRC-FGRMCU-0-velcounter.c            01                    00
C1482-SRC-FGRMCU-0-velcounter.h            01                    00

-------------------------------------------------------------------------------

    Project Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/18/11    PDB            Initial Release
                                    (Started from "ARMH Roadkill.avrgccproj")
01            12/15/11    ZAC            Added support for parametrized MAX RPM
                                    Added extra debug output for motor case temp
02            12/20/11    ZAC            Added velocity override functionality
                                    Improved PID loop processing
03            1/5/12        ZAC            Fixed case temperature output
                                    Added slack around velocity override function
04            01/16/12    ZAC            Added MAXIMUM_COMMAND parameter and bounds checking
05            03/12/12    ZAC            Rewrite of USART code to match new protocol.
-------------------------------------------------------------------------------

    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/18/11    PDB            Initial Release
                                    (Started from "ARMH Roadkill.c")
01            12/15/11    ZAC            Added MAX RPM to command parser
                                    Added debug output for motor case temp
02            12/20/11    ZAC            Added preset to command parser and SPEED CONSTANT
03            01/05/12    ZAC            Fixed case temperature output
04            01/16/12    ZAC            Added parameter for MAXIMUM_COMMAND
-------------------------------------------------------------------------------


    Firmware Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
100         2012-11-08  BA          Initial version
101         2012-12-03  BA          Fix motor and encoder directions, fix bug in thermal model
102         2013-02-20  BA          Remove LED toggle in main loop

******************************************************************************/

#include "motor.h"
#include "velcounter.h"
#include "currentADC.h"
#include "hallctrl.h"
#include "PIDcontroller.h"
#include "thermalModel.h"
#include "../common/daisycomm.h"
#include "daisyconfig.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <math.h>
#include <string.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "../common/nvm.h"
//#include <avr/fuse.h>

#define FIRMWARE_VERSION 302

//Global variables
//int isThumb;
float Parameter[INT_PARAMETER_START];

//Private Functions
static void configureClocks(void);
//static void detectFingerThumb(void);
static float convertArrayToFloat(uint8_t *packedInteger);
static void convertFloatToArray(float inputFloat, uint8_t *outputArray);
static void WriteIntToEEPROM(uint8_t address, uint8_t* value);
static void ReadIntFromEEPROM(uint8_t address, uint8_t* destination);
static void verifyVersion(void);
static void initStateFromEEPROM(void);

// FUSES = 
//     {
//         .low = LFUSE_DEFAULT,
//         .high = (FUSE_BOOTSZ0 & FUSE_BOOTSZ1 & FUSE_EESAVE & FUSE_SPIEN & FUSE_JTAGEN),
//         .extended = EFUSE_DEFAULT,
//     };

// __fuse_t __fuse __attribute__((section (".fuse"))) = 
// {
//     .low = LFUSE_DEFAULT,
//     .high = (FUSE_BOOTSZ0 & FUSE_BOOTSZ1 & FUSE_EESAVE & FUSE_SPIEN & FUSE_JTAGEN),
//     .extended = EFUSE_DEFAULT,
// };

// FUSES = 
//     {
//         LFUSE_DEFAULT, // .low
//         (FUSE_BOOTSZ0 & FUSE_BOOTSZ1 & FUSE_EESAVE & FUSE_SPIEN & FUSE_JTAGEN), // .high
//         EFUSE_DEFAULT, // .extended
//     };

//Private variables
// #ifdef USE_FAULT_PIN
// static uint8_t faultCounter = 0;
// #endif

/************************************************************************
 * Debug macros to control a line for viewing with an oscilloscope and
 * benchmarking the CPU load
 ************************************************************************/
#define ASSERT_DEBUG_LINE() PORTB.OUTSET = 0x04
#define DEASSERT_DEBUG_LINE() PORTB.OUTCLR = 0x04

/******************************************************************************
 * Interrupt service routines
 *******************************************************************************/

/************************************************************************
 * The timer tick interrupt.
 * this fires every 5 ms to increment a USART timeout register and signal the
 * PID control loop to fire in a non-interrupt context
 *************************************************************************/
ISR(CONTROL_TIMER_vect)
{
    cli();
    PID_runPID = 1;
// #ifdef USE_FAULT_PIN
//     if((MOTOR1FAULTPORT.IN & (1 << MOTOR1_FAULT_PIN)) == 0 ||
//        (MOTOR1OTWPORT.IN & (1 << MOTOR1_OTW_PIN)) == 0)
//     {
//         if (faultCounter == FAULT_TIMEOUT_COUNT)
//         {
//             resetPIDController(0);
//             updateMotorDirection(DIRECTION_STOP);
//             updateMotorVoltage(0x00);
//             updateMotor(); //forceMotorUpdate();
//             faultCounter = 0;
//         } else {
//             faultCounter++;
//         }
//     } else {
//         faultCounter = 0;
//     }
// #endif
}

/************************************************************************
 * Private function definitions
 ************************************************************************/

/************************************************************************
 * configureClocks()
 *
 * Configure system clocks
 * The RTC is being used to generate PID ticks, so ensure that the 32 kHz oscillator
 * is running.  This will feed a 1.024 kHz clock to the RTC.
 * The system clock will use the onboard 32MHz oscillator
 ************************************************************************/
static void configureClocks(void)
{
    OSC.CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;            //enable the 32MHz ring oscillator and the 32kHz RTC oscillator
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));                //wait for 32MHz to stabilize
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));                //wait for 32kHz to stabilize

    DFLLRC32M.CTRL = DFLL_ENABLE_bm;
    DFLLRC2M.CTRL = DFLL_ENABLE_bm;

    CCP = CCP_IOREG_gc;                                    //clock is protected by Configuration Change Protection - must be properly disabled to change clock source
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;                    //switch to 32MHz oscillator
    CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;    //Select 1.024 kHz from internal oscillator for RTC clock source
}

// /************************************************************************
// * detectFingerThumb()
// * Sets the global variable isThumb to 1 if this is the Thumb and 0 if
// * this is the Finger
// ************************************************************************/
// static void detectFingerThumb(void)
// {
//     PORTR.DIR = 0x00;
//     isThumb = ((PORTR.IN & 0x01) ? 1 : 0);
// }

/************************************************************************
 * convertArrayToFloat(int *packedInteger)
 *
 * Blindly assumes that packedInteger is really a pointer to a float.
 * This is broken out as a function in case more complex manipulation is
 * required for potential other host architectures.
 *
 * Currently only used in USART communication with a host PC or other
 * micro.
 ************************************************************************/
static float convertArrayToFloat(uint8_t *packedInteger)
{
    return *(float *)packedInteger;
}

/************************************************************************
 * convertArrayToFloat(float inputFloat, uint8_t *outputArray)
 *
 * Blindly places the bytes from inputFloat into outputArray.
 * This is broken out as a function in case more complex manipulation is
 * required for potential other host architectures.
 ************************************************************************/
static void convertFloatToArray(float inputFloat, uint8_t *outputArray)
{
    memcpy(outputArray,&inputFloat,4);
    return;
}

static void initStateFromEEPROM(void)
{
    uint32_t zero_check = 0;
    ReadIntFromEEPROM(EEPROM_ADDRESS_LED, (uint8_t*)&zero_check);
    
    if(zero_check)
        LEDon();
    else
        LEDoff();
}


void WriteFloatToEEPROM(uint8_t address, float val)
{
    // Write to EEPROM
    uint32_t data;
    memcpy(&data,&val,4);
    // little endian
    eeprom_write_byte((uint8_t *) (address*4+0), data & 0xff);
    data >>= 8;
    eeprom_write_byte((uint8_t *) (address*4+1), data & 0xff);
    data >>= 8;
    eeprom_write_byte((uint8_t *) (address*4+2), data & 0xff);
    data >>= 8;
    eeprom_write_byte((uint8_t *) (address*4+3), data & 0xff);
}

static void WriteIntToEEPROM(uint8_t address, uint8_t* value)
{
    for(uint8_t i=0; i<4; i++)
        eeprom_write_byte((uint8_t *) (address*4+i), *(value+i));
}

static void ReadIntFromEEPROM(uint8_t address, uint8_t* destination)
{
    for(uint8_t i=0; i<4;i++)
        *(destination+i) = eeprom_read_byte((uint8_t *) (address*4+i));
}

static void verifyVersion()
{
    uint32_t readversion = 0;
    ReadIntFromEEPROM(EEPROM_ADDRESS_FIRMWARE_VERSION, (uint8_t*)&readversion);
    if (readversion != FIRMWARE_VERSION)
    {
        readversion = FIRMWARE_VERSION;
        WriteIntToEEPROM(EEPROM_ADDRESS_FIRMWARE_VERSION, (uint8_t*)&readversion);
    }
}

void initializeEEPROM(void)
{
    // default values for parameters
    Parameter[PARAMETER_TORQUE_KP] = 0.03;
    Parameter[PARAMETER_TORQUE_KI] = 0.01;
    Parameter[PARAMETER_TORQUE_KD] = 0.0;
    Parameter[PARAMETER_VELOCITY_KP] = 0.005;
    Parameter[PARAMETER_VELOCITY_KI] = 0.0025;
    Parameter[PARAMETER_VELOCITY_KD] = 0.0;
    Parameter[PARAMETER_POWER_KP] = 1.0;
    Parameter[PARAMETER_POWER_KI] = 0.3;
    Parameter[PARAMETER_POWER_KD] = 0.0;
    Parameter[PARAMETER_WINDING_R] = DEFAULT_WINDING_RESISTANCE; // winding resitance
    Parameter[PARAMETER_THERMAL_R] = 2.66; // Thermal restance to case
    Parameter[PARAMETER_T_PLUS] = 90.0;
    Parameter[PARAMETER_T_MINUS] = 90.0;
    Parameter[PARAMETER_WINDING_TAU] = 1777.0;
    Parameter[PARAMETER_T_MAX] = 110.0;
    Parameter[PARAMETER_CU_ALPHA] = 0.0039;
    Parameter[PARAMETER_OFF_TIME] = 5.0; // number of time constants until reset
    Parameter[PARAMETER_T_TARGET] = 90.0; 
    Parameter[PARAMETER_MAXIMUM_RPM] = 12000.0;
    Parameter[PARAMETER_SPEED_CONSTANT] = 402.0;
    Parameter[PARAMETER_MAXIMUM_PWM] = 150.0;
    Parameter[PARAMETER_POSITION_KP] = 15.0;
    Parameter[PARAMETER_POSITION_DEADBAND] = 25.0;
    //Parameter[PARAMETER_LED] = 1;

    // Write to EEPROM
    for(uint8_t i=0; i<INT_PARAMETER_START; i++) 
    {
        WriteFloatToEEPROM(i, Parameter[i]);
    }
    
    // set the magic number to indicate initialized EEPROM
    eeprom_write_byte((uint8_t *) (PARAM_EEPROM_INIT*4+0), 0);
    eeprom_write_byte((uint8_t *) (PARAM_EEPROM_INIT*4+1), PARAM_EEPROM_INIT);
}

/*************************************
 // initializeParameters()
 // check to see if EEPROM has bee initilailized
 // if yes, read the params from it
 // if not, set default values into EEPROM, then read them out
 ************************************/
void initializeParameters(void)
{
    // check to see if EEPROM is initialized
    if((0 != eeprom_read_byte((uint8_t *) (PARAM_EEPROM_INIT*4+0))) || 
       (PARAM_EEPROM_INIT != eeprom_read_byte((uint8_t *) (PARAM_EEPROM_INIT*4+1)))) 
    {
        initializeEEPROM();
    }
    
    // read out the Parameters
    for(uint8_t i=0; i<INT_PARAMETER_START; i++)
    {
        uint32_t data;
        // little endian
        data = eeprom_read_byte((uint8_t *) (i*4+3));
        data <<= 8;
        data |= eeprom_read_byte((uint8_t *) (i*4+2));
        data <<= 8;
        data |= eeprom_read_byte((uint8_t *) (i*4+1));
        data <<= 8;
        data |= eeprom_read_byte((uint8_t *) (i*4+0));
        memcpy(&Parameter[i], &data, 4);
    }
}

static int handleCollectionCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint8_t responseSize = 0;
    uint16_t collectionBitfield;
    uint16_t controlSignal;

    //collectionBitfield = (uint16_t) (commandPacket[PAYLOAD_OFFSET]);
    memcpy(&collectionBitfield,&commandPacket[PAYLOAD_OFFSET],2);

    if(collectionBitfield & DATA_COLLECTION_TENSION_BITMASK)
    {
        memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize],&tension,4);
        responseSize += 4;
    }

    if(collectionBitfield & DATA_COLLECTION_MOTORCURRENT_BITMASK)
    {
        controlSignal = round(motorCurrent * 1000.0); //convert to mA
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize] = controlSignal & 0x00FF;
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize+1] = (controlSignal & 0xFF00) >> 8;
        responseSize += 2;
    }

    if(collectionBitfield & DATA_COLLECTION_MOTORSTATORTEMP_BITMASK)
    {
        controlSignal = round(statorTemperature * 100.0); //convert to hundredths of a degree
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize] = controlSignal & 0x00FF;
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize+1] = (controlSignal & 0xFF00) >> 8;
        responseSize += 2;
    }

    if(collectionBitfield & DATA_COLLECTION_MOTORVELOCITY_BITMASK)
    {
        controlSignal = averageRPM;
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize] = controlSignal & 0x00FF;
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize+1] = (controlSignal & 0xFF00) >> 8;
        responseSize += 2;
    }

    if(collectionBitfield & DATA_COLLECTION_MOTORWINDINGTEMP_BITMASK)
    {
        controlSignal = round((oldDeltaT + statorTemperature) * 100.0); //convert to hundredths of a degree
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize] = controlSignal & 0x00FF;
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize+1] = (controlSignal & 0xFF00) >> 8;
        responseSize += 2;
    }

    if(collectionBitfield & DATA_COLLECTION_MOTORHALL_BITMASK)
    {
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize] = encoder & 0x00FF;
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize+1] = (encoder & 0xFF00) >> 8;
        responseSize += 2;
    }

    if(collectionBitfield & DATA_COLLECTION_DEBUG_BITMASK)
    {
        //memcpy(&outputBuffer[RESPONSE_PAYLOAD_OFFSET+responseSize],&RxCheckSumErrCnt,4);
        // responseSize += 4;
        
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+0] = nvm_fuses_read(FUSEBYTE0);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+1] = nvm_fuses_read(FUSEBYTE1);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+2] = nvm_fuses_read(FUSEBYTE2);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+3] = nvm_fuses_read(FUSEBYTE4);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+4] = nvm_fuses_read(FUSEBYTE5);
        outputBuffer[RESPONSE_PAYLOAD_OFFSET+5] = NVM_LOCKBITS;
        responseSize += 6;
    }

    outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3+responseSize;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = DATA_COLLECTION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
    outputBuffer[3+responseSize] = computeChecksum(outputBuffer,3+responseSize); //this is the checksum
    return 4+responseSize;

}

/************************************************************************
 * handleParameterCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
 *
 * handles the Parameter Commands
 ************************************************************************/
static int handleParameterCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint8_t opcode = commandPacket[COMMAND_OFFSET] & OPCODE_BITMASK;
    uint8_t selectedParameter = commandPacket[COMMAND_OFFSET] & PARAMETER_ADDRESS_BITMASK;
    
    if(selectedParameter < INT_PARAMETER_START) // float parameter
    {
        float* parameterPointer = &Parameter[selectedParameter];
        
        //Finally check for READ/WRITE direction
        if((opcode == MOTOR_PARAMETER_RE_H_OPCODE) || (opcode == MOTOR_PARAMETER_RE_L_OPCODE)) //read
        {
            convertFloatToArray(*parameterPointer, &outputBuffer[RESPONSE_PAYLOAD_OFFSET]);
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 7;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = opcode;
            outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
            outputBuffer[7] = computeChecksum(outputBuffer,7); //this is the checksum
            return 8;
        }
        else //write
        {
            float convertedData = convertArrayToFloat(&commandPacket[PAYLOAD_OFFSET]);
            *parameterPointer = convertedData;
            WriteFloatToEEPROM(selectedParameter, convertedData);

            outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = opcode;
            outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
            outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
            return 4;
        }
    }
    else // int parameter
    {
        //Finally check for READ/WRITE direction
        if((opcode == MOTOR_PARAMETER_RE_H_OPCODE) || (opcode == MOTOR_PARAMETER_RE_L_OPCODE)) //read
        {
            ReadIntFromEEPROM(selectedParameter, &outputBuffer[RESPONSE_PAYLOAD_OFFSET]);
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 7;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = opcode;
            outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
            outputBuffer[7] = computeChecksum(outputBuffer, 7); //this is the checksum
            return 8;
        }
        else //write
        {
            WriteIntToEEPROM(selectedParameter, &commandPacket[PAYLOAD_OFFSET]);
            
            if(selectedParameter == EEPROM_ADDRESS_LED) 
            {
                if(commandPacket[PAYLOAD_OFFSET+0] |
                   commandPacket[PAYLOAD_OFFSET+1] |
                   commandPacket[PAYLOAD_OFFSET+2] |
                   commandPacket[PAYLOAD_OFFSET+3])
                    LEDon();
                else
                    LEDoff();
            }

            outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = opcode;
            outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
            outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
            return 4;
        }
    }
}

static int handleMotorCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint8_t motorDirection;
    uint8_t controlScheme;

    DIRECTION_MODE_t newDirection;
    PID_SCHEME_t newScheme;

    uint16_t setpoint;
    int applySetpoint = THERMAL_DISCARD;
    int rejectSetpoint;

    motorDirection = commandPacket[COMMAND_OFFSET] & MOTOR_COMMAND_DIRECTION_BITMASK;
    controlScheme = commandPacket[COMMAND_OFFSET] & MOTOR_COMMAND_SCHEME_BITMASK;

    switch(controlScheme)
    {
        case MOTOR_COMMAND_VELOCITY:
            newScheme = CONTROL_SCHEME_VELOCITY;
            break;
        case MOTOR_COMMAND_CURRENT:
            newScheme = CONTROL_SCHEME_CURRENT;
            break;
        case MOTOR_COMMAND_VOLTAGE:
            newScheme = CONTROL_SCHEME_VOLTAGE;
            break;
        case MOTOR_COMMAND_POSITION:
            newScheme = CONTROL_SCHEME_POSITION;
            break;
        default:
            newScheme = CONTROL_SCHEME_VOLTAGE;
            break;
    }


    switch(motorDirection)
    {
        case MOTOR_COMMAND_FORWARD:
            newDirection = DIRECTION_FORWARD;
            break;
        case MOTOR_COMMAND_REVERSE:
            newDirection = DIRECTION_REVERSE;
            break;
        case MOTOR_COMMAND_STOP:
            newDirection = DIRECTION_STOP;
            newScheme = CONTROL_SCHEME_VOLTAGE;
            break;
        default:
            newDirection = DIRECTION_STOP;
            newScheme = CONTROL_SCHEME_VOLTAGE;
            break;
    }

    //Always apply direction commands independent of anything else if it is a stop command.
    if(newDirection == DIRECTION_STOP)
    {
        updateMotorDirection(newDirection);
        //resetVelocityCounter(); //moved inside updateMotorDirection()
        //updateMotorVoltage(0x00);// inside updateMotorDirection()
    }
    else if (getMotorDirection() == DIRECTION_STOP)
    {
        //Start from a clean motor voltage if just starting up
        //Later logic also resets if a control scheme has changed.
        resetPIDController();
        updateMotorVoltage(0x00);
    }
    
    //Cast the 16 bit setpoint to a uint16.  Incoming data should be little endian to match
    //this processor's compiler convention
    //This will allow conversion to fixed point more easily if necessary
    setpoint = *(uint16_t *)&commandPacket[PAYLOAD_OFFSET];
    rejectSetpoint = 0;

    switch(newScheme)
    {
        case CONTROL_SCHEME_VOLTAGE:
            cli();
            if((setpoint > MAXIMUM_VOLTAGE_COMMAND) || (setpoint < MINIMUM_VOLTAGE_COMMAND))
            {
                rejectSetpoint = 1;
                break;
            }
            applySetpoint = thermalCheckSetpoint(CONTROL_SCHEME_VOLTAGE, setpoint);
            if((applySetpoint == THERMAL_DELAY) || (applySetpoint == THERMAL_OK))
            {
                PID_state.controlScheme = CONTROL_SCHEME_VOLTAGE;
                PID_state.setpoint = setpoint;
                updateMotorDirection(newDirection);
                // if(newDirection != getMotorDirection())//moved inside updateMotorDirection(
                // {
                //     updateMotorDirection(newDirection);
                //     //resetVelocityCounter(); //moved inside updateMotorDirection()
                // }
            }
            if(applySetpoint == THERMAL_OK)
            {
                //Setpoint must be scaled down because the actual control signal is only 8 bits, not 16.
                updateMotorVoltage(setpoint / 256);
            }

            break;

        case CONTROL_SCHEME_VELOCITY:
            cli();
            if((setpoint > Parameter[PARAMETER_MAXIMUM_RPM]) || (setpoint < MINIMUM_VELOCITY_COMMAND))
            {
                rejectSetpoint = 1;
                break;
            }
            applySetpoint = thermalCheckSetpoint(CONTROL_SCHEME_VELOCITY, setpoint);

            if((applySetpoint == THERMAL_DELAY) || (applySetpoint == THERMAL_OK))
            {
                //Configure the setpoint before presetting so that it will know how to preset.
                //Configure the control scheme afterwards so the following if check knows if the control
                //scheme has changed.
                PID_state.setpoint = setpoint;
                if((applySetpoint == THERMAL_OK) && (PID_state.controlScheme != CONTROL_SCHEME_VELOCITY) && (getMotorDirection() != DIRECTION_STOP))
                {
                    //Only preset the motor if the motor control mode is being switched mid-rotation
                    PIDPreset(CONTROL_SCHEME_VELOCITY);
                }
                PID_state.controlScheme = CONTROL_SCHEME_VELOCITY;
                updateMotorDirection(newDirection);
                // if(newDirection != getMotorDirection())//moved inside updateMotorDirection()
                // {
                //     updateMotorDirection(newDirection);
                //     resetVelocityCounter();//moved inside updateMotorDirection()
                // }
            }
            break;

        case CONTROL_SCHEME_CURRENT:
            cli();
            if((setpoint > MAXIMUM_CURRENT_COMMAND) || (setpoint < MINIMUM_CURRENT_COMMAND))
            {
                rejectSetpoint = 1;
                break;
            }
            applySetpoint = thermalCheckSetpoint(CONTROL_SCHEME_CURRENT, setpoint);
            if((applySetpoint == THERMAL_DELAY) || (applySetpoint == THERMAL_OK))
            {
                //Configure the setpoint before presetting so that it will know how to preset.
                //Configure the control scheme afterwards so the following if check knows if the control
                //scheme has changed.
                PID_state.setpoint = setpoint;
                if((applySetpoint == THERMAL_OK) && (PID_state.controlScheme != CONTROL_SCHEME_CURRENT) && (getMotorDirection() != DIRECTION_STOP))
                {
                    //Only preset the motor if the motor control mode is being switched mid-rotation
                    PIDPreset(CONTROL_SCHEME_CURRENT);
                }
                PID_state.controlScheme = CONTROL_SCHEME_CURRENT;
                updateMotorDirection(newDirection);
                // if(newDirection != getMotorDirection())//moved inside updateMotorDirection()
                // {
                //     updateMotorDirection(newDirection);
                //     resetVelocityCounter();//moved inside updateMotorDirection()
                // }
            }
            break;

        case CONTROL_SCHEME_POSITION:
            cli();
            positionSetPoint = (int16_t)setpoint;
            if((positionSetPoint > MAXIMUM_POSITION_COMMAND) || (positionSetPoint < MINIMUM_POSITION_COMMAND))
            {
                rejectSetpoint = 1;
                break;
            }

            setpoint = positionControl(positionSetPoint);
            
            applySetpoint = thermalCheckSetpoint(CONTROL_SCHEME_VELOCITY, setpoint);

            if((applySetpoint == THERMAL_DELAY) || (applySetpoint == THERMAL_OK))
            {
                //Configure the setpoint before presetting so that it will know how to preset.
                //Configure the control scheme afterwards so the following if check knows if the control
                //scheme has changed.
                PID_state.setpoint = setpoint;
                //if((applySetpoint == THERMAL_OK) && (PID_state.controlScheme != CONTROL_SCHEME_POSITION) && (getMotorDirection() != DIRECTION_STOP))
                //{
                //    //Only preset the motor if the motor control mode is being switched mid-rotation
                //    PIDPreset(CONTROL_SCHEME_POSITION);
                //}
                PID_state.controlScheme = CONTROL_SCHEME_POSITION;
                //updateMotorDirection(newDirection);
                // if(newDirection != getMotorDirection())//moved inside updateMotorDirection()
                // {
                //     updateMotorDirection(newDirection);
                //     resetVelocityCounter();//moved inside updateMotorDirection()
                // }
            }

            break;

        default:
            break;
    }
    sei();
    
    updateMotor(); //forceMotorUpdate();
    
    if(newDirection == DIRECTION_STOP)
    {
        outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
    } 
    else if(rejectSetpoint) 
    {
        outputBuffer[RESPONSE_STATUSCODE_OFFSET] = OUT_OF_RANGE;
    } 
    else
    {
        switch(applySetpoint)
        {
            case THERMAL_OK:
                outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
                break;
            case THERMAL_DELAY:
                outputBuffer[RESPONSE_STATUSCODE_OFFSET] = DELAYED_ERROR;
                break;
            case THERMAL_DISCARD:
                outputBuffer[RESPONSE_STATUSCODE_OFFSET] = MANDATORY_COOLDOWN;
                break;
            default:
                outputBuffer[RESPONSE_STATUSCODE_OFFSET] = MANDATORY_COOLDOWN;
                break;
        }
    }

    outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = MOTOR_COMMAND_OPCODE;
    //Status code filled in above
    outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
    return 4;
}


static int handleCalibrationCommand(uint8_t *commandPacket, uint8_t *outputBuffer)
{
    uint16_t collectionBitfield;

    memcpy(&collectionBitfield,&commandPacket[PAYLOAD_OFFSET],2);

    if (collectionBitfield & DATA_COLLECTION_DEBUG_BITMASK)
    {
        RxCheckSumErrCnt[0] = 0;
        RxCheckSumErrCnt[1] = 0;
    }

    if (collectionBitfield & DATA_COLLECTION_MOTORHALL_BITMASK)
    {
        cli();
        encoder = 0;
        sei();
    }
    
    outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
    outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = CALIBRATION_OPCODE;
    outputBuffer[RESPONSE_STATUSCODE_OFFSET] = STATUS_OK;
    outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
    return 4;
}

void BootRelay(uint8_t *commandPacket)
{
    // overview: 1)send a bootload command to the target
    // overview: 2)configure for 115200 baud passthrough
    // overview: 3)start a 20 second timer
    // overview: 4)do the serial passthrough, blink LED
    // overview: 5)wait for timer to finish, then reset/reboot

    cli(); // disable all interrupts, nothing else going on
    DMA.CTRL = 0; // disable the DMA controller
  
    // repeat the command to the target
    DOWNSTREAM_USART.CTRLA = 0x00; // disable interrupts
    DOWNSTREAM_USART.CTRLB = USART_RXEN_bm | USART_TXEN_bm; //enable RX and TX
    for(int i=0;i<COMMAND_PACKET_SIZE;i++)
    {
        while(!(DOWNSTREAM_USART.STATUS & USART_DREIF_bm));
        DOWNSTREAM_USART.DATA = commandPacket[i];
    }
    // dont bother waiting for a response verification
    while(!(DOWNSTREAM_USART.STATUS & USART_DREIF_bm)); // wait for the data to be read
    DOWNSTREAM_USART.STATUS = USART_TXCIF_bm; // clear the TX done flag
    while(!(DOWNSTREAM_USART.STATUS & USART_TXCIF_bm)); // wait for the TX to be done
    _delay_ms(1);
  
    // Reconfigure the serial ports
    //int bsel = 1047;       // 115200 @ 32Mhz as calculated from ProtoTalk.net
    //uint8_t bscale = 10;      // 115200 @ 32Mhz as calculated from ProtoTalk.net
    int bsel = 3269;       // 38,400 @ 32Mhz as calculated from ProtoTalk.net
    uint8_t bscale = -6;      // 38,400 @ 32Mhz as calculated from ProtoTalk.net
    UPSTREAM_USART.CTRLA = 0x00; // disable interrupts
    UPSTREAM_USART.BAUDCTRLA = (uint8_t) bsel;
    UPSTREAM_USART.BAUDCTRLB = (bscale << 4) | (bsel >> 8);
    UPSTREAM_USART.CTRLB = USART_RXEN_bm | USART_TXEN_bm;    //enable RX and TX
    DOWNSTREAM_USART.CTRLA = 0x00; // disable interrupts
    DOWNSTREAM_USART.BAUDCTRLA = (uint8_t) bsel;
    DOWNSTREAM_USART.BAUDCTRLB = (bscale << 4) | (bsel >> 8);
    DOWNSTREAM_USART.CTRLB = USART_RXEN_bm | USART_TXEN_bm;    //enable RX and TX
  
    int sec_cnt=0;
    uint32_t loop_cnt=0;
    uint8_t up_fifo[16];
    uint8_t up_head = 0;
    uint8_t up_tail = 0;
    uint8_t down_fifo[16];
    uint8_t down_head = 0;
    uint8_t down_tail = 0;

    // relay chars while waiting for timeout
    while(sec_cnt < 200) {
        loop_cnt++;
        if(loop_cnt > 90000L) { // approximately 0.1sec experimentally
            loop_cnt=0;
            sec_cnt++;
            if((sec_cnt&0x03)==0x03)
                LEDon();
            else
                LEDoff();
        }
        if (DOWNSTREAM_USART.STATUS & USART_RXCIF_bm) {
            up_fifo[up_head++] = DOWNSTREAM_USART.DATA;
            up_head %= sizeof(up_fifo);
        }
        if (UPSTREAM_USART.STATUS & USART_RXCIF_bm) {
            down_fifo[down_head++] = UPSTREAM_USART.DATA;
            down_head %= sizeof(down_fifo);
        }
        if (up_head != up_tail && (UPSTREAM_USART.STATUS & USART_DREIF_bm)) {
            UPSTREAM_USART.DATA = up_fifo[up_tail++];
            up_tail %= sizeof(up_fifo);
        }
        if (down_head != down_tail && (DOWNSTREAM_USART.STATUS & USART_DREIF_bm)) {
            DOWNSTREAM_USART.DATA = down_fifo[down_tail++];
            down_tail %= sizeof(down_fifo);
        }
    }

    // 30sec timer finished, reset/reboot
    wdt_enable(WDT_PER_8CLK_gc);
    while(1);  // use watchdog timer to trigger a reset
}

static int handleBootloaderCommand(uint8_t *commandPacket)
{
    if(0x0f & commandPacket[PAYLOAD_OFFSET]){
        commandPacket[PAYLOAD_OFFSET] = (0xf0 & commandPacket[PAYLOAD_OFFSET]); // mask off chain index
        commandPacket[CHECKSUM_OFFSET]=computeChecksum((uint8_t *)commandPacket,COMMAND_PACKET_SIZE-1);
        BootRelay(commandPacket);
    }
    else {
        // self bootload
        wdt_enable(WDT_PER_8CLK_gc);
        while(1);  // use watchdog timer to trigger a reset
    }
    return 5; // never get here, but keep compiler happy
}


/************************************************************************
 * void parseCommand(uint8_t *packetBuffer, uint8_t bytesReceived)
 *
 * Parse the command delivered in *packetBuffer with length bytesReceived
 * This routine performs the appropriate action and generates a response
 * packet if necessary.
 * Parity checking at the byte level has already been done in the RX
 * interrupt.  It has not been done in the case of DMA-driven reception
 * due to hardware limitations.
 ************************************************************************/
int processCommand(uint8_t *commandPacket,uint8_t *outputBuffer)
{

    uint8_t opcode;

    opcode = commandPacket[COMMAND_OFFSET] & OPCODE_BITMASK;

    switch(opcode)
    {
        case MOTOR_PARAMETER_RE_L_OPCODE: //Fallthrough intentional
        case MOTOR_PARAMETER_RE_H_OPCODE: //Fallthrough intentional
        case MOTOR_PARAMETER_WR_L_OPCODE: //Fallthrough intentional
        case MOTOR_PARAMETER_WR_H_OPCODE:
            return handleParameterCommand(commandPacket,outputBuffer);

        case DATA_COLLECTION_OPCODE:
            return handleCollectionCommand(commandPacket,outputBuffer);

        case MOTOR_COMMAND_OPCODE:
            return handleMotorCommand(commandPacket,outputBuffer);

        case CALIBRATION_OPCODE:
            return handleCalibrationCommand(commandPacket, outputBuffer);

        case BOOTLOADER_OPCODE:
            return handleBootloaderCommand(commandPacket);

        default:
            outputBuffer[RESPONSE_PACKETSIZE_OFFSET] = 3;
            outputBuffer[RESPONSE_REFLECTEDOPCODE_OFFSET] = opcode;
            outputBuffer[RESPONSE_STATUSCODE_OFFSET] = UNKNOWN_COMMAND;
            outputBuffer[3] = computeChecksum(outputBuffer,3); //this is the checksum
            return 4;
    }
}

void LEDon()
{
    PORTC.OUTSET = 0x80;
}
void LEDoff()
{
    PORTC.OUTCLR = 0x80;
}
void LEDtoggle()
{
    PORTC.OUTTGL = 0x80;
}

/************************************************************************
 * MAIN function.
 *
 * This intitializes the other port modules and polls a few volatile status
 * bits set in ISRs.
 ************************************************************************/
int main(void)
{
    uint8_t controlSignal;
    cli();                                    //disable all interrupts for clock reset
    //detectFingerThumb();
    _delay_ms(1); // for stability of supplies
    configureClocks();
    _delay_ms(1); // for stability of clocks
    configurePortIO();
    initializeParameters();

    // No need to delay due to bootloader
    // Need to delay remaining configuration until the board (and reference) have powered up fully
    //_delay_ms(4500);
    //LEDon();
    //_delay_ms(500);
    initStateFromEEPROM();
    verifyVersion();

    configureMotorPWMs();
    configureMotorCounters();
    configureDaisyUSART();

    configureADC();
    configurePinInterrupts();
    configurePIDController();

    PMIC.CTRL |= PMIC_LOLVLEN_bm;            //tell event system to pay attention to low-priority interrupts
    
    nvm_blbb_lock_bits_write(NVM_BLBB_WLOCK_gc); // lock bootloader from being written
    
    sei();

    updateMotor(); //forceMotorUpdate();

    while(1)
    {
        if(notifyDaisy)
        {
            doDaisyTask();
        }
        handleTC();

        if(PID_runPID == 1)
        {
            //Run the PID control loop
            //Interrupts are disabled while the velocity signals are read
            //This is to prevent the interrupt handler from changing them underneath while they are being
            //averaged
            ASSERT_DEBUG_LINE();
            cli();
            PID_runPID = 0;
            readVelocitySignal();
            sei();

            //Current signals do not need reading in an interrupt-free context because the ADC is free-running
            //Store the most recent acquisition on all channels
            readCurrentSignals();
            //LEDtoggle();
            updateThermalModel(statorTemperature, motorCurrent);
            controlSignal = runPIDController();
// #ifdef USE_OVERSPEED_BACKOFF
//             if(isOverspeed())
//             {
//                 updateMotorVoltage(controlSignal/4);
//             } else {
//                 updateMotorVoltage(controlSignal);
//             }
// #else
            updateMotorVoltage(controlSignal);
// #endif
            updateMotor(); //forceMotorUpdate();
            DEASSERT_DEBUG_LINE();
        }
    }
    return 0;
}
