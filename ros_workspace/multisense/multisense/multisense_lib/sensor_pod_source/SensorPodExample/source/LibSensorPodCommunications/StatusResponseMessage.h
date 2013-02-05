/**
 * @file LibSensorPodCommunications/StatusResponseMessage.h
 *
 * This message contains status information.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_StatusResponseMessage
#define LibSensorPodCommunications_StatusResponseMessage

#include "AbstractSerializedMessage.h"

//
// Status bits.
//

#define STATUS_GENERAL_OK       0x01
#define STATUS_LASER_OK         0x02
#define STATUS_LASER_MOTOR_OK   0x04
#define STATUS_CAMERAS_OK       0x08

//
// The message definition.
//

class StatusResponseMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = SP_STATUS_RESPONSE};

    //
    // The reported uptime for the system.
    //

    crl::TimeStamp uptime;

    //
    // Status bits.
    //

    uint32_t status;

    //
    // Internal temperature measurements (in degrees C).
    //

    float temperature0;
    float temperature1;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& messageBuffer)
    {
        messageBuffer.setType(SP_STATUS_RESPONSE);

        messageBuffer << uptime;
        messageBuffer << status;
        messageBuffer << temperature0;
        messageBuffer << temperature1;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& messageBuffer)
    {
        messageBuffer.confirmType(SP_STATUS_RESPONSE);

        messageBuffer >> uptime;
        messageBuffer >> status;
        messageBuffer >> temperature0;
        messageBuffer >> temperature1;
    }
};

#endif
