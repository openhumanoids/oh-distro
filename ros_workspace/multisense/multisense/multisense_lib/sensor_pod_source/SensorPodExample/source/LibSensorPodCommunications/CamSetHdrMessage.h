/**
 * @file LibSensorPodCommunications/CamSetHdrMessage.h
 *
 * This message sets HDR parameters on the image sensor.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-10-03, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamSetHdrMessage
#define LibSensorPodCommunications_CamSetHdrMessage

#include "AbstractSerializedMessage.h"

class CamSetHdrMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_CAM_SET_HDR};

    //
    // Parameters
    //

    // TODO: Currently specific to CMV2000; abstract it.

    float   exp_kp2;    // relative to overall exposure length (0.0-1.0)
    float   exp_kp1;    // ""

    uint8_t vlow3;      // 64~128
    uint8_t vlow2;      // ""

    uint8_t nr_slopes;  // 1-3

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_CAM_SET_HDR);

        message << exp_kp2;
        message << exp_kp1;
        message << vlow3;
        message << vlow2;
        message << nr_slopes;
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_CAM_SET_HDR);

        message >> exp_kp2;
        message >> exp_kp1;
        message >> vlow3;
        message >> vlow2;
        message >> nr_slopes;
    }
};

#endif
