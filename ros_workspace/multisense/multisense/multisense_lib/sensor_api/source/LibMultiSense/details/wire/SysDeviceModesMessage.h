/**
 * @file LibMultiSense/SysDeviceModesMessage.h
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
 *   2013-06-17, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysDeviceModesMessage
#define LibMultiSense_SysDeviceModesMessage

#include <typeinfo>

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class DeviceMode {
public:

    uint32_t width;
    uint32_t height;
    uint32_t supportedDataSources;
    uint32_t flags;

    DeviceMode(uint32_t w=0,
               uint32_t h=0,
               uint32_t d=0,
               uint32_t f=0) :
        width(w),
        height(h),
        supportedDataSources(d),
        flags(f) {};

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & width;
        message & height;
        message & supportedDataSources;
        message & flags;
    }
    
};

class SysDeviceModes {
public:
    static const IdType      ID      = ID_DATA_SYS_DEVICE_MODES;
    static const VersionType VERSION = 1;

    //
    // Available formats

    std::vector<DeviceMode> modes;

    //
    // Constructors

    SysDeviceModes(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysDeviceModes() {};
        
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & modes;
    }
};

}}}}; // namespaces

#endif
