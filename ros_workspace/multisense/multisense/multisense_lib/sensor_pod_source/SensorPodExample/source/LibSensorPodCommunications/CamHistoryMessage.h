/**
 * @file LibSensorPodCommunications/CamHistoryMessage.h
 *
 * This message contains the a history of what camera frames were sent
 * recently.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamHistoryMessage
#define LibSensorPodCommunications_CamHistoryMessage

#include "AbstractSerializedMessage.h"

class CamHistoryMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = SP_CAM_HISTORY};

    //
    // A list of recently-sent frame numbers.
    //

    int32_t history[SENSOR_POD_PUBLICATION_HISTORY_LENGTH];

  
    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_CAM_HISTORY);

        for(unsigned int ii = 0; ii < SENSOR_POD_PUBLICATION_HISTORY_LENGTH;
            ++ii) {

            message << history[ii];

        }

    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(SP_CAM_HISTORY);

        for(unsigned int ii = 0; ii < SENSOR_POD_PUBLICATION_HISTORY_LENGTH;
            ++ii) {
            
            message >> history[ii];
            
        }
    }
};

#endif
