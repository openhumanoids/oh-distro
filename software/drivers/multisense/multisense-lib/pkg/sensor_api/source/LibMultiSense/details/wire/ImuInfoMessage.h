/**
 * @file LibMultiSense/ImuInfoMessage.h
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
 *   2013-11-08, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_ImuInfoMessage
#define LibMultiSense_ImuInfoMessage

#include <typeinfo>

namespace crl {
namespace multisense {
namespace details {
namespace wire {
namespace imu {

class RateType {
public:
    static const VersionType VERSION = 1;

    float sampleRate;
    float bandwidthCutoff;

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & sampleRate;
        message & bandwidthCutoff;
    }
};

class RangeType {
public:
    static const VersionType VERSION = 1;

    float range;
    float resolution;

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & range;
        message & resolution;
    }
};

class Details {
public:
    static const VersionType VERSION = 1;

    std::string            name;
    std::string            device;
    std::string            units;
    std::vector<RateType>  rates;
    std::vector<RangeType> ranges;

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & name;
        message & device;
        message & units;
        message & rates;
        message & ranges;
    }
};

} // namespace imu

class ImuInfo {
public:
    static const IdType      ID      = ID_DATA_IMU_INFO;
    static const VersionType VERSION = 1;

    //
    // IMU details per sensor

    uint32_t                  maxSamplesPerMessage;
    std::vector<imu::Details> details;

    //
    // Constructors

    ImuInfo(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    ImuInfo() {};
        
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & maxSamplesPerMessage;
        message & details;
    }
};

}}}}; // namespaces

#endif
