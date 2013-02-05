/**
 * @file LibSensorPodCommunications/SysFlashOpMessage.h
 *
 * This message contains information on the number and layout of non-volatile
 * memory devices in the system.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-08-02, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_SysFlashOpMessage
#define LibSensorPodCommunications_SysFlashOpMessage

#include "AbstractSerializedMessage.h"

class SysFlashOpMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = CP_SYS_FLASH_OP};

    //
    // Parameters representing the desired flash operation
    //

    enum operation_type {
        OP_STATUS       = 0,    // just check status
        OP_ERASE        = 1,    // erase entire region
        OP_PROGRAM      = 2,    // program/verify specified chunk within region
        OP_VERIFY       = 3     // just verify specified chunk within region
    };

    operation_type operation;

    enum region_type {
        RGN_BITSTREAM   = 0,    // FPGA configuration bitstream
        RGN_FIRMWARE    = 1     // Microblaze firmware
    };

    region_type region;

    // remaining fields are only used for OP_PROGRAM and OP_VERIFY:

    uint32_t    start_address;  // start address of chunk to program or verify (naturally aligned)
    uint32_t    length;         // size of chunk to program or verify (power-of-2)

    static const unsigned int MAX_LENGTH = 1024;

    uint8_t     data[MAX_LENGTH];

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(CP_SYS_FLASH_OP);

        message << operation;
        message << region;

        switch(operation) {
            case OP_PROGRAM:
            case OP_VERIFY:
                message << start_address;
                message << length;
                if(length > MAX_LENGTH) {
                    CRL_STANDARD_EXCEPTION("length (%u) exceeds MAX_LENGTH (%u)", length, MAX_LENGTH);
                }
                message.write(data, length);
                break;
            case OP_STATUS:
            case OP_ERASE:
                // start/length/data not required
                break;
            default:
                CRL_STANDARD_EXCEPTION("unknown operation (%d)", (int)operation);
        }

        switch(region) {
            case RGN_BITSTREAM:
            case RGN_FIRMWARE:
                break;
            default:
                CRL_STANDARD_EXCEPTION("unknown region (%d)", (int)region);
        }
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(CP_SYS_FLASH_OP);

        message >> operation;
        message >> region;

        switch(operation) {
            case OP_PROGRAM:
            case OP_VERIFY:
                message >> start_address;
                message >> length;
                if(length > MAX_LENGTH) {
                    CRL_STANDARD_EXCEPTION("length (%u) exceeds MAX_LENGTH (%u)", length, MAX_LENGTH);
                }
                message.read(data, length);
                break;
            case OP_STATUS:
            case OP_ERASE:
                // start/length/data not required
                break;
            default:
                CRL_STANDARD_EXCEPTION("unknown operation (%d)", (int)operation);
        }

        switch(region) {
            case RGN_BITSTREAM:
            case RGN_FIRMWARE:
                break;
            default:
                CRL_STANDARD_EXCEPTION("unknown region (%d)", (int)region);
        }
    }
};

#endif
