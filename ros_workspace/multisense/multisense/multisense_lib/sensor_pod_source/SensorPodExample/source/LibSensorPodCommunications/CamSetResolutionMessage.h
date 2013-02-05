/**
 * @file LibSensorPodCommunications/CamSetResolutionMessage.h
 *
 * This message sets the output resolution of the camera.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-10-19, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamSetResolutionMessage
#define LibSensorPodCommunications_CamSetResolutionMessage

#include "AbstractSerializedMessage.h"

class CamSetResolutionMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_CAM_SET_RESOLUTION};

    //
    // Parameters
    //

    uint32_t    width;
    uint32_t    height;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_CAM_SET_RESOLUTION);

        message << width;
        message << height;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_CAM_SET_RESOLUTION);

        message >> width;
        message >> height;
    }
};

#endif
