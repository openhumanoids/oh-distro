/**
 * @file LibSensorPodCommunications/SysFlashOpAckMessage.h
 *
 * This message contains status information about requested or ongoing flash
 * operations.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-08-03, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_SysFlashOpAckMessage
#define LibSensorPodCommunications_SysFlashOpAckMessage

#include "AbstractSerializedMessage.h"

class SysFlashOpAckMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = SP_SYS_FLASH_OP_ACK};

    //
    // Parameters representing the desired flash operation
    //

    enum status_type {
        IDLE                = 0,    // no operation in progress or requested
        SUCCESS             = 1,    // requested operation succeeded/started
        FAILURE             = 2,    // requested operation failed
        ERASE_IN_PROGRESS   = 3     // erase in progress (requested operation not currently possible)
    };

    status_type status;

    // only for ERASE_IN_PROGRESS status:
    int erase_progress;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_SYS_FLASH_OP_ACK);

        message << status;

        switch(status) {
            case IDLE:
            case SUCCESS:
            case FAILURE:
                break;
            case ERASE_IN_PROGRESS:
                message << erase_progress;
                break;
            default:
                CRL_STANDARD_EXCEPTION("unknown status (%d)", (int)status);
        }
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(SP_SYS_FLASH_OP_ACK);

        message >> status;

        switch(status) {
            case IDLE:
            case SUCCESS:
            case FAILURE:
                break;
            case ERASE_IN_PROGRESS:
                message >> erase_progress;
                break;
            default:
                CRL_STANDARD_EXCEPTION("unknown status (%d)", (int)status);
        }
    }
};

#endif
