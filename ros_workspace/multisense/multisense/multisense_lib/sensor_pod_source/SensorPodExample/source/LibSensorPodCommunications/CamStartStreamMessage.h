/**
 * @file LibSensorPodCommunications/CamStartStreamMessage.h
 *
 * This message contains status information.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-14, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamStartStreamMessage
#define LibSensorPodCommunications_CamStartStreamMessage

#include "AbstractSerializedMessage.h"

//
// The message definition.
//

class CamStartStreamMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_CAM_START_STREAM};

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_CAM_START_STREAM);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_CAM_START_STREAM);
    }
};

#endif
