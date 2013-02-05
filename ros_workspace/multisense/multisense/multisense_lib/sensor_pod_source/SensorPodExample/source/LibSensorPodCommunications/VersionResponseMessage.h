/**
 * @file LibSensorPodCommunications/VersionResponseMessage.h
 *
 * This message contains versioning information.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-11, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_VersionResponseMessage
#define LibSensorPodCommunications_VersionResponseMessage

#include "AbstractSerializedMessage.h"

class VersionResponseMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = SP_VERSION_RESPONSE};

    //
    // Just an identifier representing the version
    // number of the remote device.
    //

    uint16_t versionIdentifier;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_VERSION_RESPONSE);

        message << versionIdentifier;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(SP_VERSION_RESPONSE);

        message >> versionIdentifier;
    }
};

#endif
