/**
 * @file LibSensorPodCommunications/LidarStopScanMessage.h
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

#ifndef LibSensorPodCommunications_LidarStopScanMessage
#define LibSensorPodCommunications_LidarStopScanMessage

#include "AbstractSerializedMessage.h"

//
// The message definition.
//

class LidarStopScanMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_LIDAR_STOP_SCAN};

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_LIDAR_STOP_SCAN);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_LIDAR_STOP_SCAN);
    }
};

#endif
