/**
 * @file LibSensorPodCommunications/SysSetMtuMessage.h
 *
 * This message sets the SENSOR_POD_MTU_SIZE field on the sensor head.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-09-26, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_SysSetMtuMessage
#define LibSensorPodCommunications_SysSetMtuMessage

#include "AbstractSerializedMessage.h"

class SysSetMtuMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_SYS_SET_MTU};

    //
    // Parameters
    //

    uint32_t mtu;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_SYS_SET_MTU);

        message << mtu;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_SYS_SET_MTU);

        message >> mtu;
    }
};

#endif
