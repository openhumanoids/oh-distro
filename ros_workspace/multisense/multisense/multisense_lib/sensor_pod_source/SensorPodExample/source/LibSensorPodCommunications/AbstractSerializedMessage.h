/**
 * @file LibSensorPodCommunications/AbstractSerializedMessage.h
 *
 * A base class for handling serialization/deserialization.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-11, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_AbstractSerializedMessage
#define LibSensorPodCommunications_AbstractSerializedMessage

#include "SensorPodMessageBuffer.h"
#include "MessageIdentifiers.h"

class AbstractSerializedMessage
{
public:

    virtual void serialize(SensorPodMessageBuffer& message) = 0;
    virtual void deserialize(SensorPodMessageBuffer& message) = 0;
};

#endif
