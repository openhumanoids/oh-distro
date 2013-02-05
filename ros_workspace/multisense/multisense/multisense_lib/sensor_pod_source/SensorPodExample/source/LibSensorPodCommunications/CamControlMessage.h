/**
 * @file LibSensorPodCommunications/CamControlMessage.h
 *
 * This message contains the current camera configuration.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-14, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamControlMessage
#define LibSensorPodCommunications_CamControlMessage

#include "AbstractSerializedMessage.h"

class CamControlMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_CAM_CONTROL};

    //
    // Parameters representing the current camera configuration.
    //

    float framesPerSecond;
    float gain;

    uint32_t exposureTime;

    // 
    // Constructor.
    // 

    CamControlMessage() 
      : framesPerSecond(0.0),
        gain(0.0),
        exposureTime(0.0) 
    {}

    // 
    // Destructor.
    // 

    ~CamControlMessage() {}

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_CAM_CONTROL);

        message << framesPerSecond;
        message << gain;
        message << exposureTime;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_CAM_CONTROL);

        message >> framesPerSecond;
		message >> gain;
		message >> exposureTime;
    }
};

#endif
