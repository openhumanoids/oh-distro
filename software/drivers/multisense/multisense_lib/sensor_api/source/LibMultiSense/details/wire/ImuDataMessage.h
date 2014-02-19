/**
 * @file LibMultiSense/ImuDataMessage.h
 *
 * This message contains raw IMU data.
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
 *   2013-11-07, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_ImuDataMessage
#define LibMultiSense_ImuDataMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ ImuSample {
public:
    static const VersionType VERSION    = 1;
    static const uint16_t    TYPE_ACCEL = 1;
    static const uint16_t    TYPE_GYRO  = 2;
    static const uint16_t    TYPE_MAG   = 3;

    uint16_t type;
    int64_t  timeNanoSeconds;
    float    x, y, z;

#ifndef SENSORPOD_FIRMWARE
    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & type;
        message & timeNanoSeconds;
        message & x;
        message & y;
        message & z;
    }
#endif // !SENSORPOD_FIRMWARE
};

class ImuData  {
public:
    static const IdType      ID      = ID_DATA_IMU;
    static const VersionType VERSION = 1;

    uint32_t               sequence;
    std::vector<ImuSample> samples;

#ifndef SENSORPOD_FIRMWARE

    //
    // Constructors

    ImuData(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    ImuData() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & sequence;
        message & samples;
    }
#endif // !SENSORPOD_FIRMWARE

};

}}}}; // namespaces

#endif
