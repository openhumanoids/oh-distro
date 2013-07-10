/****************************************************

 // Author:            Paul Butler
 // File Name:        C1482-SRC-FGRMCU-0-velcounter.c
 // Creation Date:    28 October, 2011
 // Revision:        01
 // Hardware:        ATxmega32A4
 // Description:    Palm Motor Controller Micro BLDC velocity counter using Hall sensors

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            10/28/11    PDB            Initial Release
01            12/15/11    ZAC            Added variable to capture maximum RPM
-------------------------------------------------------------------------------

******************************************************************************/

#include "velcounter.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

//Public variables
volatile uint16_t currentRelativeCount = 0;
float averagePeriod = 65534;
uint16_t averageRPM = 0;

//Private variables
static volatile uint16_t periodAccumulator = 0;
static volatile uint8_t samplesAccumulated = 0;
static volatile uint8_t accumulationOverflow = 0;
static volatile uint16_t previousAbsoluteCount = 0;
static volatile uint8_t stallCounter = 0;
//static volatile uint8_t overspeedCounter = 0;

static uint16_t periodToRPM(float period);

// uint8_t isOverspeed()
// {
//     return overspeedCounter;
// }

/************************************************************************
* resetVelocityCounter()
*
* Resets the velocity counter so that the next period
* measurement starts from zero
************************************************************************/
void resetVelocityCounter(void)
{
    accumulationOverflow = 0;
    periodAccumulator = 0;
    samplesAccumulated = 0;
    averagePeriod = 65534;
    averageRPM = 0;
    //overspeedCounter = 0;
    stallCounter = 0;
}

/************************************************************************
* uint16_t periodToRPM(period)
*
* Converts the measured Hall sensor period to RPM
************************************************************************/
static uint16_t periodToRPM(float period)
{
    float rawRPM = 1 / (PERIOD_CONVERSION_CONSTANT*MOTOR_POLE_PAIRS*period);
    if (rawRPM > 65535) return 65535;
    if (rawRPM < 0) return 0;
    return rawRPM;
}

/************************************************************************
* readVelocitySignal()
*
* Consumes the period accumulator signal and averages it.  If nothing has
* been accumulated since its last run, it leaves averagePeriod alone.
* This causes the PID loop to get the same input it received previously.
************************************************************************/
void readVelocitySignal(void)
{
    float stallTime;
    //Average the accumulator if it is OK.
    //Otherwise fake a period measurement using the stall counter
    //The timer completely overflows with a period of 500 ms, but the control loop is polling at 5 ms.
    //At really slow speeds (or a stall), an appropriate velocity needs to be synthesized.
    //Reset accumulation regardless
    if((accumulationOverflow == 0) && (samplesAccumulated != 0))
    {
        averagePeriod = (float) periodAccumulator / (float) samplesAccumulated;
        stallCounter = 0;
    } else {
        //Nothing happened since the last check.  Increment the stall counter
        stallCounter++;
        if(stallCounter >= 100)
        {
            averagePeriod = 65534;
        } else {
            stallTime = stallCounter * COUNTS_PER_CONTROL_PERIOD;
            if(stallTime > averagePeriod)
            {
                averagePeriod = stallTime;
            }
        }
    }
    averageRPM = periodToRPM(averagePeriod);
/*    if(averageRPM[selectedMotor] >= maxRPM[selectedMotor])
    {
        overspeedCounter[selectedMotor]++;
        //The motor is being overdriven.  Increment the counter and be prepared for shutdown
        if(overspeedCounter[selectedMotor] > 5)
        {
            updateMotorDirection(selectedMotor,DIRECTION_STOP);
            updateMotorVoltage(0x00);
            overspeedCounter[selectedMotor] = 0;
        }
    } else {
        overspeedCounter[selectedMotor] = 0;
    }
*/
    accumulationOverflow = 0;
    periodAccumulator = 0;
    samplesAccumulated = 0;
    return;
}

/************************************************************************
* configureMotorCounter()
* This function configures a Timer/Counter to time the width of Hall sensor edges
* It makes heavy use of the event system to accurately time captures.  An
* interrupt tied to the key pins records the value captured before it is lost.
************************************************************************/
void configureMotorCounters(void)
{
    //Configure the Event System to route the pin change signals for the Hall Effect sensors to the Timer Counters

    EVSYS.CH0MUX = EVENT_HALL_A1;
    EVSYS.CH1MUX = EVENT_HALL_B1;
    EVSYS.CH2MUX = EVENT_HALL_C1;

    EVSYS.CH0CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
    EVSYS.CH1CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
    EVSYS.CH2CTRL = EVSYS_DIGFILT_1SAMPLE_gc;

    //Now make the final timer/counter sensitive to these events
    //The counter should have a period of 500 ms for now using a prescaler of 256.
    //For the Maxon EC-20, this results in a minimum velocity of 5 RPM
    //24 transitions per revolution (6 transitions per electrical cycle and four cycles per revolution)
    //Each transition at 500 ms yields 5 RPM.
    MOTOR1_EVENT_TIMER_TC.CTRLB = TC_WGMODE_NORMAL_gc | MOTOR1_HA_EN | MOTOR1_HB_EN | MOTOR1_HC_EN ;
    MOTOR1_EVENT_TIMER_TC.CTRLC = 0x00;
    MOTOR1_EVENT_TIMER_TC.CTRLD = TC_EVACT_CAPT_gc | TC_EVSEL_CH0_gc;
    MOTOR1_EVENT_TIMER_TC.CTRLE = 0x00;
    MOTOR1_EVENT_TIMER_TC.PERBUF = 0xFFFF;
    MOTOR1_EVENT_TIMER_TC.CNT = 0x0000;
    MOTOR1_EVENT_TIMER_TC.CTRLA = TC_CLKSEL_DIV256_gc;
}

/************************************************************************
* updateMotorCount()
*
* Reads the velocity as a period.  It is called when
* a pin change interrupt fires on the Hall sensors.  At the same time, an
* event triggers a capture of the current state of the period counter to
* a timer register.  This routine picks out the appropriate capture register
* based on the interrupt flags.  It then subtracts the previously recorded
* absolute capture time to get a width of this Hall state in terms of counter
* pulses.  This value is accumulated with other measurements.  The accumulator
* is averaged and emptied by the PID tick.
************************************************************************/
void updateMotorCount(void)
{
    //Grab the most recent velocity measurement

    TC0_t *motorCounter;
    uint16_t sum;

    motorCounter = &MOTOR1_EVENT_TIMER_TC;

    uint8_t captureStatus = motorCounter->CTRLGSET & (TC0_CCABV_bm | TC0_CCBBV_bm | TC0_CCCBV_bm);
    uint16_t currentCapture;

    switch(captureStatus)
    {
        case TC0_CCABV_bm:
            currentCapture = motorCounter->CCABUF;
            break;
        case TC0_CCBBV_bm:
            currentCapture = motorCounter->CCBBUF;
            break;
        case TC0_CCCBV_bm:
            currentCapture = motorCounter->CCCBUF;
            break;
        default:
            //Multiple or no captures.  Clear everything and hope for resynchronization next time around.
            motorCounter->CTRLGCLR = TC0_CCABV_bm | TC0_CCBBV_bm | TC0_CCCBV_bm;

            //Skip remaining processing and dump this sample
            motorCounter->CNT = 0x0000;
            previousAbsoluteCount = 0;
            return;
    }

    //Simplify.  For now, just do the straight subtraction with no information about overflow.  Unless the motor is
    //moving very slowly, this will give the correct answer
    currentRelativeCount = currentCapture - previousAbsoluteCount;
    previousAbsoluteCount = currentCapture;

    //Accumulate
    sum = periodAccumulator + currentRelativeCount;
    if((samplesAccumulated == 255) || \
       (sum < periodAccumulator))
    {
        //Overflow error
        accumulationOverflow = 1;
        periodAccumulator = 0;
        samplesAccumulated = 0;
    } else {
        periodAccumulator = sum;
        samplesAccumulated++;
    }
    return;
}
