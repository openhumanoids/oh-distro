/**
 * @file LibSensorPodCommunications/LedGetStatusMessage.h
 *
 * This message contains a request for camera configuration.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 *   2012-07-17, dstrother@carnegierobotics.com, RD1020, Changed to be LedGetStatus
 **/

#ifndef LibSensorPodCommunications_LedGetStatusMessage
#define LibSensorPodCommunications_LedGetStatusMessage

#include "AbstractSerializedMessage.h"

class LedGetStatusMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_LED_GET_STATUS};

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_LED_GET_STATUS);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_LED_GET_STATUS);
    }
};

#endif
