/**
 * @file LibSensorPodCommunications/CamGetConfigMessage.h
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
 **/

#ifndef LibSensorPodCommunications_CamGetConfigMessage
#define LibSensorPodCommunications_CamGetConfigMessage

#include "AbstractSerializedMessage.h"

class CamGetConfigMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_CAM_GET_CONFIG};

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_CAM_GET_CONFIG);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_CAM_GET_CONFIG);
    }
};

#endif
