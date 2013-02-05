/**
 * @file LibSensorPodCommunications/CamStartImageStreamMessage.h
 *
 * This message contains status information.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-05-02, dlr@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamStartImageStreamMessage
#define LibSensorPodCommunications_CamStartImageStreamMessage

#include "AbstractSerializedMessage.h"

//
// The message definition.
//

class CamStartImageStreamMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_CAM_START_IMAGE_STREAM};

    // Set if you want to receive rectified images instead of raw,
    // unrectified images.

    bool sendRectifiedImages;

    /*
     * Constructor. Does nothing but initialize locals.
     */
    CamStartImageStreamMessage()
      : sendRectifiedImages(false)
    {
      // Empty.
    }

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_CAM_START_IMAGE_STREAM);

        message << sendRectifiedImages;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_CAM_START_IMAGE_STREAM);

        message >> sendRectifiedImages;
    }
};

#endif
