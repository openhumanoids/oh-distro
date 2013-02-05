/**
 * @file LibSensorPodCommunications/StatusRequestMessage.h
 *
 * This message contains a request for status information.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_StatusRequestMessage
#define LibSensorPodCommunications_StatusRequestMessage

#include "AbstractSerializedMessage.h"

class StatusRequestMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_STATUS_REQUEST};

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_STATUS_REQUEST);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_STATUS_REQUEST);
    }
};

#endif
