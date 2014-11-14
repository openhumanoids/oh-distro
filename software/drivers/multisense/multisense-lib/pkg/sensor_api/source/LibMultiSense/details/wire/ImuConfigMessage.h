/**
 * @file LibMultiSense/ImuConfigMessage.h
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

#ifndef LibMultiSense_ImuConfigMessage
#define LibMultiSense_ImuConfigMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {
namespace imu {

class Config {
public:
    static const VersionType VERSION       = 1;
    static const uint32_t    FLAGS_ENABLED = (1<<0);

    std::string name;
    uint32_t    flags;
    uint32_t    rateTableIndex;
    uint32_t    rangeTableIndex;

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & name;
        message & flags;
        message & rateTableIndex;
        message & rangeTableIndex;
    }
};

} // namespace imu

class ImuConfig {
public:
    static const IdType      ID      = ID_DATA_IMU_CONFIG;
    static const VersionType VERSION = 1;

    uint8_t                  storeSettingsInFlash;  // boolean
    uint32_t                 samplesPerMessage;     // 0 to ignore
    std::vector<imu::Config> configs;

    //
    // Constructors

    ImuConfig(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    ImuConfig() : storeSettingsInFlash(0), samplesPerMessage(0) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & storeSettingsInFlash;
        message & samplesPerMessage;
        message & configs;
    }
};

}}}}; // namespaces

#endif
