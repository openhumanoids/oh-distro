/**
 * @file LibSensorPodCommunications/CamStopImageStreamMessage.h
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

#ifndef LibSensorPodCommunications_CamStopImageStreamMessage
#define LibSensorPodCommunications_CamStopImageStreamMessage

#include "AbstractSerializedMessage.h"

//
// The message definition.
//

class CamStopImageStreamMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_CAM_STOP_IMAGE_STREAM};

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_CAM_STOP_IMAGE_STREAM);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_CAM_STOP_IMAGE_STREAM);
    }
};

#endif
