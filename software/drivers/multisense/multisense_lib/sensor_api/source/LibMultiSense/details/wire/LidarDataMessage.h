/**
 * @file LibMultiSense/LidarDataMessage.h
 *
 * This message contains raw LIDAR data.
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Significant history (date, user, job code, action):
 *   2013-05-08, ekratzer@carnegierobotics.com, PR1044, Significant rewrite.
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_LidarDataMessage
#define LibMultiSense_LidarDataMessage

#include <typeinfo>

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ LidarDataHeader {
public:

    static const IdType      ID          = ID_DATA_LIDAR_SCAN;
    static const VersionType VERSION     = 1;
    static const uint32_t    SCAN_POINTS = 1081;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORDPOD_FIRMWARE

    uint32_t           scanCount;
    uint32_t           timeStartSeconds;
    uint32_t           timeStartMicroSeconds;
    uint32_t           timeEndSeconds;
    uint32_t           timeEndMicroSeconds;
    int32_t            angleStart; // microradians
    int32_t            angleEnd;
    uint32_t           points;
#ifdef SENSORPOD_FIRMWARE
    uint32_t           distanceP[SCAN_POINTS];   // millimeters
    uint32_t           intensityP[SCAN_POINTS];
#else
    uint32_t          *distanceP;
    uint32_t          *intensityP;
#endif // SENSORPOD_FIRMWARE

    LidarDataHeader()
        :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        points(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class LidarData : public LidarDataHeader {
public:

    //
    // Constructors

    LidarData(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    LidarData() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & scanCount;
        message & timeStartSeconds;
        message & timeStartMicroSeconds;
        message & timeEndSeconds;
        message & timeEndMicroSeconds;
        message & angleStart;
        message & angleEnd;
        message & points;

	const uint32_t rangeSize     = sizeof(uint32_t) * points;
	const uint32_t intensitySize = sizeof(uint32_t) * points;

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {

            message.write(distanceP, rangeSize);
            message.write(intensityP, intensitySize);

        } else {

            distanceP = (uint32_t *) message.peek();
            message.seek(message.tell() + rangeSize);

            intensityP = (uint32_t *) message.peek();
            message.seek(message.tell() + intensitySize);
        }
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}}; // namespaces

#endif
