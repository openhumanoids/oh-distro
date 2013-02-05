/**
 * @file LibSensorPodCommunications/LidarDataMessage.h
 *
 * This message contains raw LIDAR data.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_LidarDataMessage
#define LibSensorPodCommunications_LidarDataMessage

#include "AbstractSerializedMessage.h"

class LidarDataMessage : AbstractSerializedMessage
{
public:
    enum {MSG_ID = SP_LIDAR_DATA};

    static const uint32_t HOKUYO_MAX_POINTS  = 1801*4; // 3 echoes
    static const uint32_t HOKUYO_ECHO_MARKER = (1 << 31);

    //
    // Metadata about this scan.
    //

    uint32_t scanCount;

    crl::TimeStamp timeStart;
    crl::TimeStamp timeEnd;

    // Microradians.
    int32_t angleStart;
    int32_t angleEnd;

    //
    // Raw scans from the Hokuyo.
    //

    uint32_t distance[HOKUYO_MAX_POINTS];
    uint32_t intensity[HOKUYO_MAX_POINTS];
    uint32_t points;

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_LIDAR_DATA);

        //
        // Write metadata.
        //

        message << scanCount;

        message << timeStart;
        message << timeEnd;

        message << angleStart;
        message << angleEnd;

        message << points;

        //
        // Write the raw blobs here.
        //

        message.write((const unsigned char*) distance, sizeof(uint32_t) * points);
        message.write((const unsigned char*) intensity, sizeof(uint32_t) * points);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(SP_LIDAR_DATA);

        //
        // Read metadata.
        //

        message >> scanCount;

        message >> timeStart;
        message >> timeEnd;

        message >> angleStart;
        message >> angleEnd;

        message >> points;

        //
        // Read the raw blobs here.
        //

        message.read((unsigned char*) distance, sizeof(uint32_t) * points);
        message.read((unsigned char*) intensity, sizeof(uint32_t) * points);
    }
};

#endif
