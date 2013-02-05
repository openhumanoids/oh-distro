/**
 * @file LibSensorPodCommunications/CamSetHdrAckMessage.h
 *
 * This message contains status information.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-10-03, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamSetHdrAckMessage
#define LibSensorPodCommunications_CamSetHdrAckMessage

#include "AbstractSerializedMessage.h"

//
// The message definition.
//

class CamSetHdrAckMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = SP_CAM_SET_HDR_ACK};

    //
    // Status code -- zero is success, anything else is
    // an error.
    //

    uint16_t status;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_CAM_SET_HDR_ACK);

        message << status;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(SP_CAM_SET_HDR_ACK);

        message >> status;
    }
};

#endif
