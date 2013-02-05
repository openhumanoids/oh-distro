/**
 * @file LibSensorPodCommunications/LedStatusMessage.h
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

#ifndef LibSensorPodCommunications_LedStatusMessage
#define LibSensorPodCommunications_LedStatusMessage

#include "AbstractSerializedMessage.h"

class LedStatusMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = SP_LED_STATUS};

    //
    // Parameters representing the current LED configuration.
    //

    static const int MAX_LEDS = 8;

    // bit mask indicating which LEDs are implemented
    uint8_t available;

    // current LED duty cycles; 0 = off; 255 = 100%
    uint8_t intensity[MAX_LEDS];

    // if non-zero, LEDs are only on while sensors are exposing
    uint8_t flash;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_LED_STATUS);

        message << available;
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
        message.confirmType(SP_LED_STATUS);
        
        message >> available;
        for(int i=0;i<MAX_LEDS;i++) {
            message >> intensity[i];
        }
        message >> flash;
    }
};

#endif
