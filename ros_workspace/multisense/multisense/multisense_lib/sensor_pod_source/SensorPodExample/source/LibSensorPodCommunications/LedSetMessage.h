/**
 * @file LibSensorPodCommunications/LedSetMessage.h
 *
 * This message contains the current camera configuration.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-07-17, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_LedSetMessage
#define LibSensorPodCommunications_LedSetMessage

#include "AbstractSerializedMessage.h"

class LedSetMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_LED_SET};

    //
    // Parameters representing the desired LED configuration.
    //

    static const int MAX_LEDS = 8;

    // bit mask selecting which LEDs to update
    uint8_t mask;

    // LED duty cycles; 0 = off; 255 = 100%
    uint8_t intensity[MAX_LEDS];

    // if non-zero, LEDs are only on while sensors are exposing
    uint8_t flash;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_LED_SET);

        message << mask;
        for(int i=0;i<MAX_LEDS;i++) {
            message << intensity[i];
        }
        message << flash;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_LED_SET);
        
        message >> mask;
        for(int i=0;i<MAX_LEDS;i++) {
            message >> intensity[i];
        }
        message >> flash;
    }
};

#endif
