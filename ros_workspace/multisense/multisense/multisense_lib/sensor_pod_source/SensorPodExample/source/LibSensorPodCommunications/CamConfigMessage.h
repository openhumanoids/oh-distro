/**
 * @file LibSensorPodCommunications/CamConfigMessage.h
 *
 * This message contains the current camera configuration.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamConfigMessage
#define LibSensorPodCommunications_CamConfigMessage

#include "AbstractSerializedMessage.h"

class CamConfigMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = SP_CAM_CONFIG};

    //
    // Parameters representing the current camera configuration.
    //

    uint16_t width;
    uint16_t height;

    float framesPerSecond;
    float gain;

    uint32_t exposureTime;

    float fx, fy;
    float cx, cy;

    float tx, ty, tz;
    float roll, pitch, yaw;

    // Constructor.
    CamConfigMessage()
      : width(0),
        height(0),
        framesPerSecond(0),
        gain(0.0),
        exposureTime(0),
        fx(0.0),
        fy(0.0),
        cx(0.0),
        cy(0.0),
        tx(0.0),
        ty(0.0),
        tz(0.0),
        roll(0.0), 
        pitch(0.0), 
        yaw(0.0) 
     {}


    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_CAM_CONFIG);

        message << width;
        message << height;

        message << framesPerSecond;
        message << gain;
        message << exposureTime;

        message << fx;
        message << fy;

        message << cx;
        message << cy;

        message << tx;
        message << ty;
        message << tz;

        message << roll;
        message << pitch;
        message << yaw;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(SP_CAM_CONFIG);

        message >> width;
        message >> height;

        message >> framesPerSecond;
		message >> gain;
		message >> exposureTime;

        message >> fx;
        message >> fy;

        message >> cx;
        message >> cy;

        message >> tx;
        message >> ty;
        message >> tz;

        message >> roll;
        message >> pitch;
        message >> yaw;
    }
};

#endif
