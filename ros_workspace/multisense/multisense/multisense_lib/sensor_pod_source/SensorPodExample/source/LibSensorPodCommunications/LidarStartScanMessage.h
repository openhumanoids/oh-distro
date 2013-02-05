/**
 * @file LibSensorPodCommunications/LidarStartScanMessage.h
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

#ifndef LibSensorPodCommunications_LidarStartScanMessage
#define LibSensorPodCommunications_LidarStartScanMessage

#include "AbstractSerializedMessage.h"

//
// The message definition.
//

class LidarStartScanMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_LIDAR_START_SCAN};

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_LIDAR_START_SCAN);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_LIDAR_START_SCAN);
    }
};

#endif
