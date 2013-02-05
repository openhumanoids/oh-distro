/**
 * @file LibSensorPodCommunications/VersionRequestMessage.h
 *
 * This message contains a request for versioning information.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-11, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_VersionRequestMessage
#define LibSensorPodCommunications_VersionRequestMessage

#include "AbstractSerializedMessage.h"

class VersionRequestMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_VERSION_REQUEST};

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_VERSION_REQUEST);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_VERSION_REQUEST);
    }
};

#endif
