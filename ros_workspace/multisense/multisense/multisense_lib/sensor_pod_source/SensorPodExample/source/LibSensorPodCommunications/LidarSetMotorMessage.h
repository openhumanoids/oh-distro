/**
 * @file LibSensorPodCommunications/LidarSetMotorMessage.h
 *
 * This message contains the current camera configuration.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-07-19, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_LidarSetMotorMessage
#define LibSensorPodCommunications_LidarSetMotorMessage

#include "AbstractSerializedMessage.h"

class LidarSetMotorMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_LIDAR_SET_MOTOR};

    //
    // Parameters representing the desired motor configuration.
    //

    // desired output RPM
    float rpm;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_LIDAR_SET_MOTOR);

        message << rpm;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_LIDAR_SET_MOTOR);
        
        message >> rpm;
    }
};

#endif
